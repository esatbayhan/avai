from time import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from std_msgs.msg import Bool


class Heartbeat(Node):

    HEARTBEAT_THRESHOLD_MAX = 5

    def __init__(self):
        super().__init__("heartbeat")

        self.publisher_controls = None
        self.last_heartbeat = None
        self.heartbeat_threshold = None
        self.emergency_stop_frequency = 20

        self.check_heartbeat_timer = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscription_name_heartbeat", "heartbeat")
        self.declare_parameter("publisher_name_controls", "cmd_vel")
        self.declare_parameter("heartbeat_threshold", 1.0)
        self.declare_parameter("emergency_stop_frequency", 5)

    def init_subscription_heartbeat(self, topic_name: str) -> bool:
        self.get_logger().info("Initializing subscription heartbeat")

        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(Bool, topic_name, self.subscription_heartbeat, qos_profile=qos_profile_sensor_data)) == Subscription

    def init_publisher_controls(self, topic_name: str) -> bool:
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        self.publisher_controls = self.create_publisher(Twist, topic_name, 2)

        return True

    def init_heartbeat_threshold(self, threshold: float) -> bool:
        if type(threshold) != float:
            return False

        if not (0 < threshold < Heartbeat.HEARTBEAT_THRESHOLD_MAX):
            return False

        self.heartbeat_threshold = threshold
        return True

    def initializer(self, param: Parameter, valid_types: tuple, init_function) -> bool:
        if param.type_ not in valid_types:
            return False

        return init_function(param.value)

    def init_parameters(self, params):
        results = []

        for param in params:
            if param.name == "subscription_name_heartbeat":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscription_heartbeat))
            elif param.name == "publisher_name_controls":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher_controls))
            elif param.name == "heartbeat_threshold" and param.type_ == Parameter.Type.DOUBLE:
                self.heartbeat_threshold = param.value
                results.append(True)
            elif param.name == "emergency_stop_frequency" and param.type_ == Parameter.Type.INTEGER:
                self.emergency_stop_frequency = param.value
                results.append(True)
            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscription_heartbeat(self, message: Bool) -> None:
        if type(self.check_heartbeat_timer) != Timer:
            self.get_logger().info("Received first heartbeat")
            self.check_heartbeat_timer = self.create_timer(self.heartbeat_threshold, self.check_heartbeat)

        self.last_heartbeat = time()

        self.get_logger().info(f"Received heartbeat at {self.last_heartbeat}")

    def check_heartbeat(self) -> None:
        # Type Checking
        if type(self.last_heartbeat) != float:
            self.get_logger().warn(f"self.last_heartbeat is not float but {self.last_heartbeat}")
            return
        
        if type(self.publisher_controls) != Publisher:
            self.get_logger().warn(f"self.publisher_controls is not float but {self.publisher_controls}")
            return

        if type(self.heartbeat_threshold) != float:
            self.get_logger().warn(f"self.heartbeat_threshold is not float but {self.heartbeat_threshold}")
            return

        # Check Heartbeat
        if time() - self.last_heartbeat > self.heartbeat_threshold:
            self.destroy_timer(self.check_heartbeat_timer)
            self.create_timer(1 / self.emergency_stop_frequency, self.emergency_stop)

    def emergency_stop(self):
        self.get_logger().warn("NO HEARTBEAT, EMERGENCY STOP")
        self.publisher_controls.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    heartbeat = Heartbeat()
    rclpy.spin(heartbeat)

    heartbeat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
