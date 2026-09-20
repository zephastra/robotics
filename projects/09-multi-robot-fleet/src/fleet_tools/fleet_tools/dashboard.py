"""P6 read-only dashboard: a localhost web page plus RViz markers.

Two hard constraints from AI_EXECUTION_PROMPTS item 6, both enforced rather than
documented:

  * **Read-only.** The HTTP server implements GET on three paths and nothing else.
    Every other method is 405 and every other path is 404, and there is no code path
    that reaches the fleet. Submission, cancellation and fault injection stay in
    `fleet_cli`, which is the only thing allowed to change fleet state by hand. A web
    page that could move a robot would be a second control surface that nothing in
    this project's safety story accounts for.
  * **Loopback only.** Binding is refused for anything that is not a loopback address,
    at start-up, before the socket is opened. "Defaults to localhost" is not the same
    guarantee as "cannot be told to listen on the network".

The data comes from the coordinator's and the task service's own reported state --
never from Gazebo's truth stream, which is the evaluator's input and explicitly must
not reach anything that a human might steer by.

What the page is allowed to claim is decided in `display.py`, which has no ROS import
and is unit-tested, including the case that matters most: it must not keep showing
green after the readings stop arriving.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from fleet_core import load_traffic_config

from .display import FRESH_S, STALE_S, summarise

READ_ONLY_PATHS = ("/", "/api/snapshot", "/healthz")

#: A localhost-only bind is a safety property, so it is checked rather than trusted.
LOOPBACK = ("127.0.0.1", "localhost", "::1")

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>009 fleet - read-only</title>
<style>
 body{font:13px/1.6 system-ui,sans-serif;margin:0;padding:18px 22px;background:#f7f7f5;color:#1c1c1a}
 h1{font-size:15px;font-weight:500;margin:0 0 4px}
 h2{font-size:14px;font-weight:500;margin:22px 0 8px}
 .banner{padding:10px 12px;border-radius:8px;margin:10px 0 16px;border:1px solid #999}
 .LIVE{background:#e8f4e8;border-color:#4a7a4a}
 .AGING{background:#fdf3dd;border-color:#a37a20}
 .STALE,.DISCONNECTED{background:#fbe6e6;border-color:#a03030;font-weight:500}
 table{border-collapse:collapse;width:100%;background:#fff}
 th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #e6e6e2;vertical-align:top}
 th{font-weight:500;color:#555}
 .ok{color:#1d6b1d}.warn{color:#8a6100}.bad{color:#a02020}.unknown{color:#666}
 .muted{color:#666}
 .note{font-size:12px;color:#666;margin-top:4px}
 code{background:#efefe9;padding:1px 4px;border-radius:4px}
</style></head><body>
<h1>009 fleet &mdash; read-only</h1>
<div class="note" id="readonly"></div>
<div class="banner" id="banner">loading&hellip;</div>
<div id="body"></div>
<script>
const T = {ok:"ok",warn:"warn",bad:"bad",unknown:"unknown"};
function esc(s){return String(s==null?"":s).replace(/[&<>]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[c]));}
function cls(v){return T[v]||T.unknown;}
async function tick(){
  let d;
  try{ d = await (await fetch("/api/snapshot",{cache:"no-store"})).json(); }
  catch(e){
    document.getElementById("banner").className="banner DISCONNECTED";
    document.getElementById("banner").textContent =
      "DISCONNECTED - the page cannot reach its own backend. Nothing here is current.";
    return;
  }
  const b=document.getElementById("banner");
  b.className="banner "+d.liveness.state;
  b.textContent=d.liveness.banner;
  document.getElementById("readonly").textContent=d.read_only;
  const r=d.robots||[], t=d.tasks||[], p=d.payloads||[], res=d.resources||[];
  let h="";
  h+="<h2>robots</h2><table><tr><th>robot</th><th>state</th><th>battery</th><th>band</th><th>task</th><th>pose</th><th>notes</th></tr>";
  for(const x of r){
    h+="<tr><td>"+esc(x.robot_id)+"</td><td class='"+cls(x.display_state)+"'>"+cls(x.display_state)+"</td>"
      +"<td>"+(x.battery_fraction==null?"&mdash;":(100*x.battery_fraction).toFixed(0)+"%")+"</td>"
      +"<td>"+esc(x.battery_band)+"</td><td><code>"+esc(x.task_id)+"</code></td>"
      +"<td>"+(x.position?x.position[0].toFixed(2)+", "+x.position[1].toFixed(2):"&mdash;")+"</td>"
      +"<td class='muted'>"+x.notes.map(esc).join("; ")+"</td></tr>";
  }
  h+="</table>";
  h+="<h2>resources (owner / generation / queue)</h2><table><tr><th>resource</th><th>state</th><th>owner</th><th>gen</th><th>queue</th></tr>";
  if(!res.length){ h+="<tr><td colspan=5 class='muted'>no reading from the coordinator</td></tr>"; }
  for(const x of res){
    h+="<tr><td>"+esc(x.name)+"</td><td class='"+cls(x.display_state)+"'>"+esc(x.state)+"</td>"
      +"<td>"+esc(x.owner)+"</td><td>"+x.generation+"</td><td>"+esc(x.queue.join(", "))+"</td></tr>";
  }
  h+="</table>";
  h+="<h2>tasks</h2><table><tr><th>task</th><th>state</th><th>kind</th><th>robot</th><th>leg</th><th>attempts</th><th>detail</th></tr>";
  if(!t.length){ h+="<tr><td colspan=7 class='muted'>no tasks, or no live reading</td></tr>"; }
  for(const x of t){
    h+="<tr><td><code>"+esc(x.task_id)+"</code></td><td>"+esc(x.state)+"</td><td>"+esc(x.kind)+"</td>"
      +"<td>"+esc(x.robot_id)+"</td><td>"+esc(x.leg)+"</td><td>"+x.attempts+"</td>"
      +"<td class='muted'>"+esc(x.detail)+"</td></tr>";
  }
  h+="</table>";
  h+="<h2>payloads</h2><table><tr><th>payload</th><th>state</th><th>holder</th></tr>";
  if(!p.length){ h+="<tr><td colspan=3 class='muted'>none</td></tr>"; }
  for(const x of p){
    h+="<tr><td>"+esc(x.payload_id)+"</td><td>"+esc(x.state)+"</td><td>"+esc(x.holder)+"</td></tr>";
  }
  h+="</table>";
  h+="<p class='note'>backend: <b>"+esc(d.backend)+"</b> &middot; payload mode: "+esc(d.payload_mode)+"</p>";
  h+="<p class='note'>refusals: "+esc(JSON.stringify(d.refusals||{}))+"</p>";
  const nr=(d.reconcile||{}).not_resumed||[];
  if(nr.length){ h+="<p class='note bad'>restart parked "+nr.length+" task(s) and resumed none: "+esc(JSON.stringify(nr))+"</p>"; }
  h+="<p class='note'>traffic reading age: "+(d.traffic_age_s==null?"none":d.traffic_age_s.toFixed(1)+"s")+"</p>";
  document.getElementById("body").innerHTML=h;
}
tick(); setInterval(tick, 2000);
</script></body></html>
"""


class _Handler(BaseHTTPRequestHandler):
    server_version = "fleet009/0.1"

    def log_message(self, *_args) -> None:  # keep the console readable
        return

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/api/snapshot":
            payload = self.server.dashboard.payload()  # type: ignore[attr-defined]
            self._send(200, json.dumps(payload).encode("utf-8"), "application/json")
            return
        if path == "/healthz":
            self._send(200, b'{"ok":true}', "application/json")
            return
        self._send(404, b'{"error":"not a read-only path"}', "application/json")

    def _refuse_write(self) -> None:
        self._send(
            405,
            json.dumps(
                {
                    "error": "this dashboard is read-only",
                    "use": "fleet_cli submit | cancel | fault",
                }
            ).encode("utf-8"),
            "application/json",
        )

    do_POST = do_PUT = do_DELETE = do_PATCH = _refuse_write  # type: ignore[assignment]


class Dashboard(Node):
    def __init__(self) -> None:
        super().__init__("fleet_dashboard")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8099)
        self.declare_parameter("poll_hz", 1.0)
        self.declare_parameter("fresh_s", FRESH_S)
        self.declare_parameter("stale_s", STALE_S)
        self.declare_parameter("resources_config", "")
        self.declare_parameter("publish_markers", True)

        host = str(self.get_parameter("host").value)
        if host not in LOOPBACK:
            raise RuntimeError(
                f"fleet_dashboard: refusing to bind {host!r}. This page is read-only and "
                "must not be reachable off the machine; it has no authentication."
            )
        self.port = int(self.get_parameter("port").value)
        self.fresh_s = float(self.get_parameter("fresh_s").value)
        self.stale_s = float(self.get_parameter("stale_s").value)

        self.tasks_cli = self.create_client(Trigger, "/fleet/tasks")
        self.traffic_cli = self.create_client(Trigger, "/fleet/status")
        self._snapshot: dict | None = None
        self._traffic: dict | None = None
        self._source_at: dict[str, float] = {}
        self._lock = threading.Lock()
        self._poll_ticks = 0
        self._dry_streak = 0
        self._first_reading_on_tick: int | None = None
        self._poll_stats: dict[str, dict[str, int]] = {}
        self._inflight: dict[str, tuple[object, float]] = {}
        self._call_budget_s = 5.0

        self.marker_pub = None
        if bool(self.get_parameter("publish_markers").value):
            self._setup_markers()

        self.create_timer(1.0 / max(0.2, float(self.get_parameter("poll_hz").value)), self._poll)
        self.httpd = ThreadingHTTPServer((host, self.port), _Handler)
        self.httpd.dashboard = self  # type: ignore[attr-defined]
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"fleet_dashboard on http://{host}:{self.port}/ (read-only, GET only: "
            f"{', '.join(READ_ONLY_PATHS)})"
        )

    # ------------------------------------------------------------------ #

    def _setup_markers(self) -> None:
        """Publish the reservable regions and the stations as RViz markers.

        Derived from the same config the gate uses, so the picture cannot drift from
        what the gate enforces -- a drawing that disagreed with the rule would be worse
        than no drawing.
        """
        from visualization_msgs.msg import Marker, MarkerArray

        self._Marker = Marker
        self._MarkerArray = MarkerArray
        self.marker_pub = self.create_publisher(MarkerArray, "fleet/markers", 1)
        path = str(self.get_parameter("resources_config").value)
        regions: list[tuple[str, float, float, float, float]] = []
        if path and Path(path).is_file():
            try:
                tcfg = load_traffic_config(path)
                for name, spec in sorted(tcfg.resources.items()):
                    for rect in spec.rects:
                        regions.append(
                            (f"{name}:{rect.name}", rect.x_min, rect.y_min, rect.x_max, rect.y_max)
                        )
            except Exception as exc:
                self.get_logger().warning(f"markers: could not read {path}: {exc}")
        self._regions = regions
        self._marker_seq = 0

    def _publish_markers(self) -> None:
        if self.marker_pub is None:
            return
        now = self.get_clock().now().to_msg()
        self._marker_seq += 1
        array = self._MarkerArray()
        for i, (name, x0, y0, x1, y1) in enumerate(getattr(self, "_regions", [])):
            m = self._Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "resources"
            m.id = i
            m.type = 4  # CUBE
            m.action = 0  # ADD
            m.pose.position.x = (x0 + x1) / 2.0
            m.pose.position.y = (y0 + y1) / 2.0
            m.pose.position.z = 0.01
            m.scale.x = abs(x1 - x0)
            m.scale.y = abs(y1 - y0)
            m.scale.z = 0.02
            m.color.a = 0.25
            m.color.r = 0.8
            m.color.g = 0.2
            m.color.b = 0.2
            m.text = name
            array.markers.append(m)
        self.marker_pub.publish(array)

    def _read_service(self, client, key: str) -> bool:
        """Advance one service read by one step. Never blocks, never raises.

        A read is a two-tick state machine -- issue here, harvest on a later tick -- and
        that shape is REQUIRED, not a style choice. rclpy gives a node's clients the node's
        default callback group, which is MutuallyExclusive: while this timer callback is
        running, the client's response callback cannot be invoked by any thread. Waiting
        for the future inside the callback therefore waits for something that cannot
        happen. Measured before this change: `did not answer within 2 s` on every read,
        for ever, while an independent client got a 2988-byte reply in under a second.

        So "poll the future instead of spinning" was not enough: waiting inside an
        exclusive-group callback is the same deadlock as spinning inside one.

        Returns True only when a parsed reading was actually stored.
        """
        stats = self._poll_stats.setdefault(
            key,
            {"not_ready": 0, "calls": 0, "pending": 0, "answered": 0, "ok": 0,
             "timeout": 0, "refused": 0, "error": 0},
        )
        inflight = self._inflight.get(key)
        if inflight is not None:
            future, issued_at = inflight
            waited = time.monotonic() - issued_at
            if not future.done():
                if waited <= self._call_budget_s:
                    stats["pending"] += 1
                    return False
                stats["timeout"] += 1
                if stats["timeout"] == 1:
                    self.get_logger().warning(
                        f"poll: {client.service_name} did not answer within "
                        f"{self._call_budget_s:.1f}s (waited {waited:.1f}s)")
                self._inflight[key] = None
                return False
            self._inflight[key] = None
            stats["answered"] += 1
            try:
                result = future.result()
            except Exception as exc:  # a failed call never takes the page down with it
                stats["error"] += 1
                self.get_logger().warning(f"poll: {client.service_name} raised {exc!r}")
                return False
            if result is None or not result.success:
                stats["refused"] += 1
                return False
            try:
                parsed = json.loads(result.message)
            except ValueError as exc:
                stats["error"] += 1
                self.get_logger().warning(
                    f"poll: {client.service_name} sent non-JSON ({exc})")
                return False
            stats["ok"] += 1
            with self._lock:
                setattr(self, key, parsed)
                self._source_at[key] = time.monotonic()
            return True

        if not client.service_is_ready():
            stats["not_ready"] += 1
            return False
        if stats["calls"] == 0:
            self.get_logger().info(f"poll: {client.service_name} is available")
        stats["calls"] += 1
        self._inflight[key] = (client.call_async(Trigger.Request()), time.monotonic())
        return False

    def _poll(self) -> None:
        """Advance both reads, and say so when they are not arriving.

        The two sources are independent: the task service owns tasks and robots, the
        coordinator owns the corridor. One being unreachable must not be reported as the
        other being quiet, and neither may be reported as a fleet that is fine.
        """
        self._poll_ticks += 1
        got = self._read_service(self.tasks_cli, "_snapshot")
        self._read_service(self.traffic_cli, "_traffic")

        if got:
            # The arrival time is recorded where the reading is stored, PER SOURCE, so a
            # frozen coordinator cannot inherit the task service's freshness.
            self._dry_streak = 0
            if self._first_reading_on_tick is None:
                self._first_reading_on_tick = self._poll_ticks
                self.get_logger().info(
                    f"poll: first reading stored on tick {self._poll_ticks}")
        else:
            self._dry_streak += 1
            # Fail loud early, then go quiet: tick 20 is the first moment "it has not
            # arrived yet" becomes "it is not arriving". The budget is 5 s, so a healthy
            # read uses 2 ticks and never gets near this.
            if self._dry_streak in (20, 120) or self._dry_streak % 300 == 0:
                self.get_logger().warning(
                    f"poll: no reading for {self._dry_streak} consecutive ticks "
                    f"(tick {self._poll_ticks}) on "
                    f"{self.tasks_cli.service_name} / {self.traffic_cli.service_name}; "
                    f"stats={json.dumps(self._poll_stats, sort_keys=True)}. The page shows "
                    "DISCONNECTED because of this, not because the fleet is quiet.")
        self._publish_markers()

    def payload(self) -> dict:
        """What the page gets. Age is computed here, never in the browser.

        A page that timed its own data would call a frozen cache "fresh" for ever,
        because the cache object never changes -- which is precisely the "still green
        after the link is gone" failure.
        """
        with self._lock:
            snapshot, traffic = self._snapshot, self._traffic
            task_at = self._source_at.get("_snapshot")
            traffic_at = self._source_at.get("_traffic")
        now = time.monotonic()
        age = None if task_at is None else (now - task_at)
        traffic_age = None if traffic_at is None else (now - traffic_at)
        # Each source is aged separately, and neither falls back to the other. Folding
        # them into one number was the original mistake: it made a stale corridor look as
        # current as a live task service, on a page that said LIVE.
        return summarise(snapshot, traffic, age, traffic_age_s=traffic_age,
                         fresh_s=self.fresh_s, stale_s=self.stale_s)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = Dashboard()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        try:
            node.httpd.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


def _free_port() -> int:  # pragma: no cover - convenience for a manual run
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


if __name__ == "__main__":
    main()
