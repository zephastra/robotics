#!/usr/bin/env bash
# P6 dashboard: check the HTTP surface for real, over a socket.
#
#   bash scripts/check_p6_dashboard.sh
#
# Exit codes: 0 = all checks passed, 1 = a check failed, 3 = environment problem.
#
# Why this exists as well as tests/test_p6_display.py: that file tests the DECISION logic
# (`display.py`, pure Python) and covers staleness, disconnection and unknown resources. It
# cannot tell you whether the server actually serves those decisions, refuses writes, or
# refuses to bind somewhere other than loopback. Those are properties of the socket, so
# they are checked over the socket.
#
# Everything here runs with NO fleet. That is deliberate: "no fleet" is the state in which
# a display is most likely to lie, and `may_show_ok` must be false. A page that shows green
# because it has not heard otherwise is the specific failure P6 names.

set -uo pipefail

SELF="${BASH_SOURCE[0]:-$0}"
ROOT="$(cd "$(dirname "$SELF")/.." && pwd)"
cd "$ROOT" || exit 3

PORT="${P6_PORT:-8099}"
LOG="${TMPDIR:-/tmp}/p6_dashboard.log"

if [ ! -d install/fleet_tools ]; then
  echo "check_p6_dashboard.sh: install/ is missing. Run scripts/build.sh first." >&2
  exit 3
fi

# This check asserts "no fleet => not green". With a fleet up, the dashboard is SUPPOSED to
# show data and every assertion below would fail for the right reason and the wrong label.
if pgrep -f "$ROOT/instal[l]" >/dev/null 2>&1; then
  echo "check_p6_dashboard.sh: a fleet is running under $ROOT." >&2
  echo "  This check needs an empty world, because it asserts that no data means no green." >&2
  echo "  Stop the fleet first: bash scripts/stop_demo.sh" >&2
  exit 3
fi

# shellcheck disable=SC1091
set +u
source scripts/env.sh
set -u

cleanup() {
  [ -n "${DASH_PID:-}" ] && kill -INT "$DASH_PID" 2>/dev/null
  sleep 1
  [ -n "${DASH_PID:-}" ] && kill -KILL "$DASH_PID" 2>/dev/null
}
trap cleanup EXIT

echo "=== starting the dashboard with no fleet on port $PORT ==="
ros2 run fleet_tools fleet_dashboard --ros-args -p "port:=$PORT" > "$LOG" 2>&1 &
DASH_PID=$!

for _ in $(seq 1 40); do
  if /usr/bin/python3 - "$PORT" <<'PY' 2>/dev/null
import socket, sys
s = socket.socket()
try:
    s.connect(("127.0.0.1", int(sys.argv[1])))
except OSError:
    raise SystemExit(1)
finally:
    s.close()
PY
  then
    break
  fi
  sleep 1
done

echo "dashboard pid $DASH_PID; log: $LOG"
echo

/usr/bin/python3 - "$PORT" <<'PY'
"""Assert the HTTP surface. Every check says what it is protecting."""

import json
import sys
import urllib.error
import urllib.request

PORT = int(sys.argv[1])
HOST = "127.0.0.1"
BASE = f"http://{HOST}:{PORT}"
failures: list[str] = []
checks = 0


class _Headers(dict):
    """Case-insensitive lookups, because urllib used to hand back one of these."""

    def get(self, key, default=None):
        for name, value in self.items():
            if name.lower() == str(key).lower():
                return value
        return default


def call(method: str, path: str):
    """One request over a raw socket, read by hand.

    `urllib` was removed here after a direct A/B at a single moment against a single server:
    a hand-read socket got 200 and the full body ELEVEN times in a row, while urllib got it
    twice and then raised ConnectionResetError on every later attempt -- while the socket
    kept working. The server is not the failing half, so the checker stops using the client
    that cannot read it. This project reached the same conclusion once before; this time the
    raw socket is the instrument rather than a one-off probe.

    WHY urllib latches on this page is NOT explained. That is recorded as an open question
    instead of being hidden behind a retry: the earlier retry-based fix did not help because
    the failure is not transient.
    """
    import socket  # local: the rest of this script has no other need for it

    last = "no attempt made"
    for attempt in range(1, 3):
        try:
            sock = socket.create_connection((HOST, PORT), timeout=10.0)
        except OSError as exc:
            last = f"connect: {type(exc).__name__}: {exc}"
            if attempt < 2:
                print(f"  NOTE  {last} on {method} {path}; retry 1/1")
            continue
        try:
            request = (
                f"{method} {path} HTTP/1.1\r\n"
                f"Host: {HOST}\r\n"
                "Connection: close\r\n"
                "Content-Length: 0\r\n\r\n"
            )
            sock.sendall(request.encode())
            chunks: list[bytes] = []
            while True:
                data = sock.recv(65536)
                if not data:
                    break
                chunks.append(data)
        except OSError as exc:
            last = f"{type(exc).__name__}: {exc}"
            if attempt < 2:
                print(f"  NOTE  {last} on {method} {path}; retry 1/1")
            continue
        finally:
            sock.close()

        blob = b"".join(chunks)
        head, _, body = blob.partition(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        status = 0
        parts = lines[0].split()
        if len(parts) >= 2 and parts[1].isdigit():
            status = int(parts[1])
        headers = _Headers()
        for line in lines[1:]:
            if ":" in line:
                name, _, value = line.partition(":")
                headers[name.strip()] = value.strip()
        declared = headers.get("Content-Length")
        if declared and declared.isdigit() and int(declared) != len(body):
            print(f"  WARN  {method} {path}: declared {declared} bytes, received "
                  f"{len(body)} -- the page is truncated")
        if status:
            return status, body, headers
        last = f"no status line in {len(blob)} bytes: {blob[:60]!r}"

    print(f"  WARN  {method} {path} failed twice: {last}")
    return 0, last.encode("utf-8"), _Headers()


def check(label: str, ok: bool, detail="") -> None:
    global checks
    checks += 1
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        shown = detail if isinstance(detail, str) else repr(detail)
        failures.append(f"{label}: {shown}" if shown else label)


# --- it is up ------------------------------------------------------------------ #
status, body, _ = call("GET", "/healthz")
check("GET /healthz answers 200", status == 200, f"got {status}")
check("GET /healthz says ok", b'"ok":true' in body, body[:120].decode("utf-8", "replace"))

# --- and it does not claim to be healthy about the fleet ----------------------- #
status, body, _ = call("GET", "/api/snapshot")
check("GET /api/snapshot answers 200", status == 200, f"got {status}")
try:
    payload = json.loads(body)
except Exception as exc:  # noqa: BLE001
    payload = None
    check("GET /api/snapshot returns JSON", False, str(exc))
if payload is not None:
    # The key path is asserted, not guessed. `summarise` nests the verdict under
    # `liveness`; reading a flat `may_show_ok` would have made this check report a failure
    # in the dashboard when the mistake was in the check. The whole key set is printed
    # when it does not match, so the next person does not have to probe for it.
    check("the snapshot carries a liveness verdict",
          isinstance(payload.get("liveness"), dict),
          f"got {sorted(payload)}")
    live = payload.get("liveness") or {}
    check("may_show_ok is FALSE with no fleet",
          live.get("may_show_ok") is False,
          f"got {live.get('may_show_ok')!r} -- a display with no data must not be green")
    check("the liveness state is DISCONNECTED, not LIVE",
          live.get("state") == "DISCONNECTED", f"got {live.get('state')!r}")
    check("robots are withheld, not defaulted", payload.get("robots") == [],
          str(payload.get("robots")))
    check("tasks are withheld, not defaulted", payload.get("tasks") == [],
          str(payload.get("tasks")))
    check("resources are withheld, not defaulted", payload.get("resources") == [],
          str(payload.get("resources")))
    check("no backend is claimed without data",
          payload.get("backend") == "UNKNOWN", f"got {payload.get('backend')!r}")

# --- the page says the same thing --------------------------------------------- #
status, body, headers = call("GET", "/")
text = body.decode("utf-8", "replace")
check("GET / answers 200", status == 200, f"got {status}")
check("GET / is HTML", "text/html" in headers.get("Content-Type", ""),
      headers.get("Content-Type", ""))
markers = ("NOT LIVE", "DISCONNECTED", "STALE", "NO DATA")
check("the page shows a not-live marker rather than a robot table",
      any(m in text for m in markers), f"none of {markers} in {len(text)} bytes")
check("the page does not claim OK for a robot",
      " OK " not in text, "found a bare OK token in a page with no fleet")

# --- read-only means read-only ------------------------------------------------- #
for method in ("POST", "PUT", "DELETE", "PATCH"):
    status, body, _ = call(method, "/")
    check(f"{method} / is refused with 405", status == 405, f"got {status}")
    check(f"{method} / says why", b"read-only" in body, body[:120].decode("utf-8", "replace"))

status, _, _ = call("GET", "/somewhere-else")
check("an unknown path is 404, not 200", status == 404, f"got {status}")

print()
print(f"  checks: {checks}")
if failures:
    print(f"P6_DASHBOARD_FAILED ({len(failures)} problem(s)):")
    for f in failures:
        print(f"    - {f}")
    raise SystemExit(1)
print("P6_DASHBOARD_OK: the HTTP surface behaves")
PY
RC=$?
echo

echo "=== it must refuse to bind anywhere but loopback ==="
set +e
timeout 25 ros2 run fleet_tools fleet_dashboard \
  --ros-args -p host:=0.0.0.0 -p port:=$((PORT + 1)) > "${LOG}.bind" 2>&1
BIND_RC=$?
set -e
echo "exit code: $BIND_RC"
if [ "$BIND_RC" -eq 0 ]; then
  echo "  [FAIL] it accepted a non-loopback bind; the page has no authentication"
  RC=1
elif grep -q "refusing to bind" "${LOG}.bind"; then
  echo "  PASS  refused, and said why in plain words"
else
  echo "  [FAIL] it exited, but not because it refused the host -- read ${LOG}.bind"
  RC=1
fi

echo
if [ "$RC" -ne 0 ]; then
  exit 1
fi
echo "check_p6_dashboard.sh: all checks passed (exit 0)"
