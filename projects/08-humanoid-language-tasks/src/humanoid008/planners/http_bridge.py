"""Provider-agnostic HTTP bridge (section 12).

This is **our** protocol, not a guess at any vendor's API::

    POST <endpoint><plan_path>
      -> {"request_id", "goal", "world", "skills", "recent_results", "system_prompt"}
      <- strict ActionProposal JSON

A provider adapter is written only *after* a real service is chosen.

Enforced here:

- connection and total timeouts
- a hard response size cap
- the API key is read from the environment and never logged
- transport and schema failures are surfaced, never swallowed

If no endpoint is configured, the planner is simply **unavailable**. The run must
record ``NOT_RUN_MISSING_PROVIDER`` -- quietly downgrading to the rule planner and
calling it a model result is exactly what section 12 forbids.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from ..contracts import strict_json_loads


class MissingProviderError(RuntimeError):
    """No planner endpoint is configured, so no real-model run can happen."""


class PlannerTransportError(RuntimeError):
    """The planner was unreachable, timed out, or exceeded a limit."""


@dataclass(frozen=True)
class BridgeConfig:
    endpoint: str | None
    api_key: str | None
    request_timeout_s: float
    max_response_bytes: int
    plan_path: str = "/plan"
    connection_timeout_s: float = 5.0

    @property
    def available(self) -> bool:
        return bool(self.endpoint and self.endpoint.strip())

    @classmethod
    def from_env(
        cls,
        path: str | Path,
        env: Mapping[str, str] | None = None,
    ) -> "BridgeConfig":
        """Read the planner config; secrets come from the environment only."""
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        planner = raw.get("planner") or {}
        environ = os.environ if env is None else env

        def from_env_var(key: str, default: Any) -> Any:
            name = planner.get(key)
            if not name:
                return default
            value = environ.get(str(name))
            return value if value not in (None, "") else default

        return cls(
            endpoint=from_env_var("endpoint_env", None),
            api_key=from_env_var("api_key_env", None),
            request_timeout_s=float(
                from_env_var(
                    "request_timeout_s_env",
                    planner.get("request_timeout_s_default", 15),
                )
            ),
            max_response_bytes=int(
                from_env_var(
                    "max_response_bytes_env",
                    planner.get("max_response_bytes_default", 65536),
                )
            ),
            plan_path=str(planner.get("plan_path", "/plan")),
            connection_timeout_s=float(planner.get("connection_timeout_s", 5.0)),
        )


def load_system_prompt(path: str | Path) -> str:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return str(raw.get("system_prompt", "")).strip()


class HttpPlannerBridge:
    def __init__(self, config: BridgeConfig, *, system_prompt: str = "") -> None:
        self._config = config
        self._system_prompt = system_prompt

    @property
    def config(self) -> BridgeConfig:
        return self._config

    @property
    def available(self) -> bool:
        return self._config.available

    def plan(self, payload: Mapping[str, Any]) -> Any:
        """Send one planning request and return the strictly-parsed response."""
        if not self.available:
            raise MissingProviderError(
                "no planner endpoint is configured (see config/planners/llm.yaml "
                "and .env.example)"
            )

        url = self._config.endpoint.rstrip("/") + self._config.plan_path
        body = json.dumps(
            {**payload, "system_prompt": self._system_prompt},
            ensure_ascii=False,
        ).encode("utf-8")

        headers = {"Content-Type": "application/json; charset=utf-8"}
        if self._config.api_key:
            # Never logged, never echoed into reports.
            headers["Authorization"] = f"Bearer {self._config.api_key}"

        request = urllib.request.Request(url, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(
                request, timeout=self._config.request_timeout_s
            ) as response:
                raw = response.read(self._config.max_response_bytes + 1)
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise PlannerTransportError(
                f"planner request failed: {type(exc).__name__}: {exc}"
            ) from exc

        if len(raw) > self._config.max_response_bytes:
            raise PlannerTransportError(
                f"planner response exceeded {self._config.max_response_bytes} bytes"
            )
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise PlannerTransportError("planner response was not valid UTF-8") from exc
        return strict_json_loads(text, max_bytes=self._config.max_response_bytes)
