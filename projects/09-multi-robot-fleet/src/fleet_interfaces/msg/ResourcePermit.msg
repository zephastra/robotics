# A robot's current authority over a protected crossing, as sent to that robot's
# safety gate. CONTRACTS section 7.
#
# Time is deliberately NOT an absolute wall-clock instant. The coordinator and the
# gate are different processes with independent monotonic clocks, and comparing
# them is meaningless. `valid_for_wall_s` is a duration; the receiver turns it into
# a local deadline the moment the message arrives. A refreshed grant therefore
# cannot inherit a longer life than it was given, and a replayed one cannot
# resurrect an expired permit.

std_msgs/Header header

string robot_id
string task_id
string boot_id
int32 revision
int32 generation
int32 epoch

bool granted
string permit_id

# Remaining life of this grant, in seconds on the SENDER's wall clock.
# 0.0 when granted is false.
float64 valid_for_wall_s

# Which bundle: "west_to_east" or "east_to_west". Empty when not granted, because a
# direction-less permit would authorise nothing and say nothing.
string direction
string[] resources

string reason_code
string detail
