"""Analytic SCARA kinematics; all units are metres/radians."""
import math

L1, L2 = 0.42, 0.40
TCP_Z_ZERO = 0.35
JOINT_NAMES = ['shoulder', 'elbow', 'lift', 'wrist', 'finger_left', 'finger_right']
TRAY = (-0.24, 0.0, 0.48)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def inverse(x, y, z, yaw=0.0):
    if not all(math.isfinite(v) for v in (x, y, z, yaw)):
        raise ValueError('Non-finite target')
    c = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
    lift = z - TCP_Z_ZERO
    if not -1.0-1e-9 <= c <= 1.0+1e-9 or not -1e-9 <= lift <= 0.55+1e-9:
        raise ValueError(f'Target out of arm workspace: {(x,y,z)}')
    c = min(1.0,max(-1.0,c)); lift = min(.55,max(0.0,lift))
    elbow = math.acos(c)
    shoulder = wrap(math.atan2(y,x) - math.atan2(L2*math.sin(elbow), L1+L2*c))
    wrist = wrap(yaw - shoulder - elbow)
    return [shoulder, elbow, lift, wrist]


def forward(joints):
    a, b, z, w = joints[:4]
    return (L1*math.cos(a)+L2*math.cos(a+b),
            L1*math.sin(a)+L2*math.sin(a+b), TCP_Z_ZERO+z, wrap(a+b+w))


def world_to_base(x, y, bx, by, yaw):
    dx, dy = x-bx, y-by
    return (math.cos(yaw)*dx+math.sin(yaw)*dy,
            -math.sin(yaw)*dx+math.cos(yaw)*dy)


def base_to_world(x, y, bx, by, yaw):
    return (bx+math.cos(yaw)*x-math.sin(yaw)*y,
            by+math.sin(yaw)*x+math.cos(yaw)*y)
