# 009 — Troubleshooting

Everything here is a trap that has already cost real time on this machine. Each entry says
what it *looked like*, because in every case the surface symptom pointed somewhere other
than the cause. If a symptom below matches, the cause is probably the one listed.

---

## 1. Environment decides the answer

### `ModuleNotFoundError: No module named 'em'` when running a ROS script
Looks like ROS is broken. It is the **wrong interpreter**. The project `.venv` is
deliberately ROS-free — that is what makes "the core does not depend on ROS" provable — so
anything that imports `rclpy` must run under `/usr/bin/python3`.

```bash
.venv/bin/python scripts/some_node.py     # breaks: no rclpy
/usr/bin/python3 scripts/some_node.py     # correct for ROS-touching scripts
```

### pytest dies with `No module named 'typing_extensions'`
The shell had ROS sourced. rclpy's `launch_testing` registers as a pytest plugin and pulls
in packages the venv does not have. `env -u ROS_DISTRO` is **not** enough — `PYTHONPATH`
still carries ROS paths.

**Use a new shell, or clear all of them:**

```bash
env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u ROS_VERSION -u PYTHONPATH -u LD_LIBRARY_PATH \
    bash scripts/test_core.sh
```

`scripts/test_core.sh` refuses to run when `ROS_DISTRO` is set, on purpose.

### A static check gives a different verdict in a different shell
Check-4 in `scripts/check_undefined_names.py` used to report 23 `Node` members in a clean
shell and 109 with ROS sourced, because it fell back to a short hand-written list when
`import rclpy` failed. Half of the shadowing check was off in the shell the project actually
uses, and it still printed success. It now parses rclpy's own source when rclpy cannot be
imported, and prints how it obtained the list. If a guard's answer changes with the
environment, that is a bug in the guard.

---

## 2. Cross-environment edits (Windows staging → WSL project)

### `$VAR` silently becomes empty
`wsl.exe -d Ubuntu -- bash -lc '... $VAR ...'` eats variables, command substitutions and
globs. Symptoms seen: `cp /patch_p51b.py: No such file or directory`,
`cd: null directory`, `ls: cannot access 'reports/'`, `$F: ambiguous redirect`.

**Do not inline logic.** Write a script file, copy it in, run it:

```bash
cat > /tmp/x.sh <<'EOF'
... real script ...
EOF
```

and inside the `-lc '...'` command use only **literal paths**.

### `dirname: command not found` / `cd: null directory` on every command
Harmless noise from the shell shim, not a failure of your command. Ignore when the exit
code and stdout are right.

### An edit "succeeds" but the file is unchanged
Two known causes:

1. **`cp` into a directory that does not exist fails silently enough to miss.** A missing
   `src/fleet_interfaces/action/` meant the `.action` file never arrived and the build
   reported "file does not exist". `mkdir -p` first.
2. **`replace_all` landed on some occurrences and not others.** Four occurrences of
   `self.clients` were edited by `replace_all`; only two changed. After any multi-occurrence
   edit, `grep -c` and confirm the count.

### A patcher skips an edit it should have made
An idempotency marker must be a **literal substring of what the edit inserts**. Two real
failures:

- The marker `resp.epoch = self._epoch` already existed in a *different* method, so the
  patcher concluded the edit was done and skipped the one edit that mattered. The self-check
  grepped the same string in the whole file, so it agreed.
- A marker that was a *paraphrase* of the inserted text meant a second run did not recognise
  its own work and inserted a duplicate.

`patch_p51.py` and `patch_p51b.py` now refuse to run if `marker not in replacement`, and
assert the anchor occurs exactly once. Keep that property.

### WSL `/tmp` does not survive
Scripts staged in `/tmp` vanish when WSL idles or the host restarts. Re-copy from the
Windows staging path every time; never assume `/tmp/foo.py` still exists.

---

## 3. Gazebo / Nav2 / DDS

### Two Nav2 stacks and no second robot: `Failed to activate local_costmap`, `odom frame does not exist`
Looks like a TF or costmap problem. It is the **DDS participant ceiling** (default 32). Two
Nav2 stacks are about 30 participants. Raise it in `config/cyclonedds.xml`:

```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><Interfaces><NetworkInterface name="lo"/></Interfaces></General>
    <Ports><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Ports>
  </Domain>
</CycloneDDS>
```

`<Discovery>` at that level is rejected and sets the ceiling to **0**. Pin the interface to
loopback: WSL2 exposes four interfaces and discovery is randomly partial without it.

### The robot spins its wheels and does not move; `z = -0.055`
Wheel geometry. A cylinder visual/collision has its axis on Z, and a joint `<pose>` with
`rpy 90°` rotates the child frame so the two cancel and the axis stays Z: the wheels were
flat discs. Then exact ground tangency gave ODE no contact at all. Three fixes together:
remove the joint pose, rotate the geometry, and make the collision radius slightly larger
than the nominal (0.077 vs 0.075) plus anisotropic friction
(`<mu>1.1</mu><mu2>0.15</mu2><fdir1>1 0 0</fdir1>`). Result: yaw error 53.7° → 4.0°.

### A `<physics type="ignored">` block silently voids the real-time factor
RTF reads 0 and nothing reports a problem.

### `ros2 topic list` shows a partial graph
The CLI daemon can serve a stale or partial graph. When a node "is not there", check the
process list and the launch log before believing the graph.

### The rate or latency you measured is your own probe
A probe that calls `rclpy.spin_once` in a paced loop throttles itself. `amcl_pose` "lagging
3.5–4.2 m" was the probe, not the system — retracted. Use a `MultiThreadedExecutor`, and
compare the stamp-derived rate against the delivered rate before quoting any number.

### A `/goal` is refused with `another navigator is processing`
Nav2 is momentarily busy. Treating that as a terminal refusal fails a leg that would have
worked. Retry it, and count the retry against the attempt budget so the loop still
terminates.

---

## 4. Frames

### Every robot reads as being at `(0, 0)`
Gazebo's odometry origin is the **spawn pose**, not the map origin. Comparing it against
map-frame rectangles puts every robot inside whatever rectangle contains (0,0) — with the
geofence installed, that freezes the fleet from boot. Always compose before comparing:

```python
map_x, map_y, map_yaw = compose_2d(spawn, (odom_x, odom_y, odom_yaw))
```

This bug has appeared **three times** in this project: once in the gate, once in the
adapter's `RobotState`, once in the acceptance harness. Check it first.

### A robot stopped somewhere it "should reach" refuses to release
The release node must leave enough room for the stack's **measured** arrival error. A release
node at 3.5 m left 0.20 m of slack against a measured error of 0.5051 m, so both directions
on both robots were refused. The general rule: any node whose whole purpose is to satisfy a
geometric condition needs slack ≥ the measured arrival error, not the ideal one. The
derivation is in the config comment and recomputed by `tests/test_p4_release.py`.

---

## 5. ROS node pitfalls

### `AttributeError: property 'clients' of 'TaskService' object has no setter`
`rclpy.node.Node` has read-only properties. Assigning `self.clients`, `self.publishers`,
`self.executor`, `self.handle` … raises at construction time, in a place that looks
unrelated. `check_undefined_names.py` catches this; it is wired into `build.sh`.

### `NameError` on a name that exists in a sibling method
Python locals do not cross function boundaries. `_execute` used `map_x`, a local of
`_build_state`; every completed leg raised `NameError` and returned an **empty** failure,
which reads as a dead executor rather than as a scope mistake. `check_undefined_names.py`
now does function-scope analysis and catches it.

### The node deadlocks and prints nothing
`rclpy.spin_until_future_complete` inside a callback on a `MultiThreadedExecutor` nests a
spin on an already-spinning node. Poll the future instead and `sleep`.

### `sqlite3.ProgrammingError: SQLite objects created in a thread can only be used in that same thread`
The `Ledger` opens its connection in the constructing thread; service and timer callbacks
arrive on others. The class already serialises every mutation on its own `RLock`, so the fix
is to tell sqlite that: `check_same_thread=False`.

### A `NameError` inside a thread disappears
An exception in a worker thread does not reach the caller. The P4 acceptance harness lost
its verdict to exactly this: the drive thread died, `results` stayed empty, and both cases
reported `both_completed: False` *while the robots had actually completed*. **A check that
can fail silently is indistinguishable from a robot that did not move.** Capture the
exception and put it in the result.

---

## 6. Reading the evidence

- `fail_reason` / a task's `reason` field is only trustworthy if the call site passes a
  distinct one. It was hardcoded to `CANCEL_NOT_CONFIRMED` for a while; a constant label is
  a label nobody can use.
- The world-level truth stream has **no frame names** — 2816 of 2816 `child_frame_id` values
  were empty across 176 messages. Identify robots by message index plus at least two
  independent geometric sources.
- A run whose teardown did not verify is not evidence. Check
  `teardown verified clean` in the transcript before reading any number.
- Exit code 139 (SIGSEGV) from the acceptance harness means the verdict was never written;
  it is not a task failure.
