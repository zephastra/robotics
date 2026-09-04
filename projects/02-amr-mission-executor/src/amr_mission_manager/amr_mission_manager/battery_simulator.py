"""Simple battery simulator used by the warehouse mission demo."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class BatterySimulator(Node):
    def __init__(self) -> None:
        super().__init__("battery_simulator")
        self.declare_parameter("initial_percentage", 100.0)
        self.declare_parameter("drain_per_second", 0.05)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("force_low_percentage", 10.0)

        self.initial_percentage = float(
            self.get_parameter("initial_percentage").value
        )
        self.drain_per_second = max(
            0.0, float(self.get_parameter("drain_per_second").value)
        )
        self.publish_period = max(
            0.1, float(self.get_parameter("publish_period").value)
        )
        self.force_low_percentage = max(
            0.0, min(100.0, float(self.get_parameter("force_low_percentage").value))
        )
        self.percentage = max(0.0, min(100.0, self.initial_percentage))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            Float32, "/mission/battery_percentage", qos
        )
        self.create_service(Trigger, "/mission/battery/reset", self._reset)
        self.create_service(Trigger, "/mission/battery/force_low", self._force_low)
        self.create_timer(self.publish_period, self._tick)
        self._publish()
        self.get_logger().info(
            f"Battery simulator started at {self.percentage:.1f}% "
            f"(drain {self.drain_per_second:.3f}%/s)"
        )

    def _publish(self) -> None:
        message = Float32()
        message.data = float(self.percentage)
        self.publisher.publish(message)

    def _tick(self) -> None:
        self.percentage = max(
            0.0,
            self.percentage - self.drain_per_second * self.publish_period,
        )
        self._publish()

    def _reset(self, _request: Trigger.Request, response: Trigger.Response):
        self.percentage = max(0.0, min(100.0, self.initial_percentage))
        self._publish()
        response.success = True
        response.message = f"Battery reset to {self.percentage:.1f}%"
        return response

    def _force_low(self, _request: Trigger.Request, response: Trigger.Response):
        self.percentage = self.force_low_percentage
        self._publish()
        response.success = True
        response.message = f"Battery forced to {self.percentage:.1f}%"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
