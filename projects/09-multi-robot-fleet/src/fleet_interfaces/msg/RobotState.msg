# CONTRACTS.md section 3: the minimum an adapter must publish about itself.
#
# This is the ESTIMATED state the controller actually uses, never ground truth.
# The evaluator compares these against Gazebo's dynamic_pose stream; nothing here
# is allowed to come from that stream, or the fleet would be steering on truth.
#
# Field order follows the contract table so a reader can check them side by side.

string schema_version
string robot_id
string robot_boot_id
uint64 sequence
builtin_interfaces/Time stamp_sim

# --- where the robot believes it is --------------------------------------- #
# yaw is carried explicitly as well as in the quaternion-free form because the
# geofence needs a heading to know which way the body points, and turning on the
# spot must not read as standing still.
float64 x
float64 y
float64 yaw
float64 speed_mps
float64 angular_radps
bool localization_valid
float64 pose_age_s

# --- what it is doing ------------------------------------------------------ #
string operating_state
string task_id
uint32 assignment_revision
string leg_id
string[] capabilities

# --- energy (simulated model; see BatteryConfig) --------------------------- #
float64 battery_fraction
float64 battery_wh
bool battery_valid

# --- ownership and authority ----------------------------------------------- #
# held_resources is what the robot was GRANTED, not proof of what it occupies.
string payload_id
string[] held_resources
uint32 permit_generation

# --- layered diagnostics --------------------------------------------------- #
# nav_state and gate_reason are separate on purpose: "Nav2 is fine but the gate
# is refusing" and "Nav2 gave up" look identical from outside otherwise, and
# they need opposite responses.
string nav_state
string gate_reason

# --- the two flags that must never be inferred ----------------------------- #
# stopped_confirmed is the only thing that may be treated as stopped; a zero
# command is not a stopped robot.
bool stopped_confirmed
string fault_code
