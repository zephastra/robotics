"""ROS-facing nodes for the 009 fleet.

This package exists as a SEPARATE package from fleet_adapter on purpose. P1's rule is
that the coordination core must run and be tested with no ROS present, and
tests/test_p1_adapter_no_ros.py enforces it by scanning source for rclpy imports. The
moment a node that talks to DDS lived in fleet_adapter, that invariant would have to
be weakened. So: the decision logic stays in fleet_adapter and is pure Python; this
package is only the thin layer that feeds it real topics and acts on its verdict.

Nothing in here contains policy. If a rule about motion safety appears in this
package rather than in fleet_adapter.safety_gate, it is in the wrong place.
"""

__all__ = ["__version__"]

__version__ = "0.1.0"
