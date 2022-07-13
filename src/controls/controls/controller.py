from copy import deepcopy
from math import asin, cos, radians, sqrt

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray


class Controller(Node):
    # Cone Detection
    CONE_CLASS_BLUE = 0
    CONE_CLASS_ORANGE = 1
    CONE_CLASS_YELLOW = 2
    ACCURACY_THRESHOLD = 0.4
    WIDTH_THRESHOLD = 0.04
    WIDTH_MIN, WIDTH_MAX = 0, 1

    # Laser Scan
    ANGLE_DEGREES_START = 31.1
    ANGLE_DEGREES_STOP = -31.1

    def __init__(self):
        super().__init__("controller")
        self.subscriber_queue_size = 2
        self.publisher_queue_size = 0

        self.laser_scan = None
        self.publisher_controls = None

        self.twist = Twist()
        self.orientate_counter = 0
        self.drive_counter = 0

        self.create_timer(0.2, self.publish_heartbeat)

        # Drive
        self.drive_linear_speed_start, self.drive_linear_speed_max = 0.0, 0.0
        self.drive_linear_speed_basis = 0.0
        self.drive_angular_speed_max = 0.0

        # Orientation
        self.orientation_linear_speed = 0.0
        self.orientation_angular_speed_start, self.orientation_angular_speed_max = 0.0, 0.0
        self.orientation_angular_basis = 0.0

        self.add_on_set_parameters_callback(self.init_parameters)

        self.declare_parameter("publisher_name_heartbeat", "heartbeat")
        self.declare_parameter("subscriber_laser_scan", "scan")
        self.declare_parameter("subscriber_bounding_boxes", "bounding_boxes")
        self.declare_parameter("publisher_controls", "cmd_vel")

        # Drive
        self.declare_parameter("drive_linear_speed_start", 0.2)
        self.declare_parameter("drive_linear_speed_max", 2.0)
        self.declare_parameter("drive_linear_speed_basis", 1.05)
        self.declare_parameter("drive_angular_speed_max", 1.0)

        # Orientation
        self.declare_parameter("orientation_linear_speed", 0.2)
        self.declare_parameter("orientation_angular_speed_start", 0.1)
        self.declare_parameter("orientation_angular_speed_max", 0.5)
        self.declare_parameter("orientation_angular_speed_basis", 1.1)

    def init_publisher_heartbeat(self, topic_name: str) -> bool:
        self.publisher_heartbeat = self.create_publisher(Bool, topic_name, qos_profile=qos_profile_sensor_data)
        self.create_timer(0.2, self.publish_heartbeat)

        return True

    def init_subscriber_laser_scan(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(LaserScan, topic_name, self.subscribe_laser_scan, qos_profile=qos_profile_sensor_data)) == Subscription

    def init_subscriber_bounding_boxes(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(Float64MultiArray, topic_name, self.subscribe_bounding_boxes, self.subscriber_queue_size)) == Subscription

    def init_publisher_controls(self, topic_name: str) -> bool:
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        self.publisher_controls = self.create_publisher(Twist, topic_name, self.publisher_queue_size)

        return True

    def initializer(self, param: Parameter, valid_types: tuple, init_function) -> bool:
        if param.type_ not in valid_types:
            return False

        return init_function(param.value)

    def init_parameters(self, params):
        results = []

        for param in params:
            # Publisher & Subscriber Parameters
            if param.name == "publisher_name_heartbeat":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher_heartbeat))
            elif param.name == "subscriber_laser_scan":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_laser_scan))
            elif param.name == "subscriber_bounding_boxes":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_bounding_boxes))
            elif param.name == "publisher_controls":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher_controls))

            # Drive Parameters
            elif param.name == "drive_linear_speed_start" and param.type_ == Parameter.Type.DOUBLE:
                self.drive_linear_speed_start = param.value
                results.append(True)
            elif param.name == "drive_linear_speed_max" and param.type_ == Parameter.Type.DOUBLE:
                self.drive_linear_speed_max = param.value
                results.append(True)
            elif param.name == "drive_linear_speed_basis" and param.type_ == Parameter.Type.DOUBLE:
                self.drive_linear_speed_basis = param.value
                results.append(True)
            elif param.name == "drive_angular_speed_max" and param.type_ == Parameter.Type.DOUBLE:
                self.drive_angular_speed_max = param.value
                results.append(True)

            # Orientation Parameters
            elif param.name == "orientation_linear_speed" and param.type_ == Parameter.Type.DOUBLE:
                self.orientation_linear_speed = param.value
                results.append(True)
            elif param.name == "orientation_angular_speed_start" and param.type_ == Parameter.Type.DOUBLE:
                self.orientation_angular_speed_start = param.value
                results.append(True)
            elif param.name == "orientation_angular_speed_max" and param.type_ == Parameter.Type.DOUBLE:
                self.orientation_angular_speed_max = param.value
                results.append(True)
            elif param.name == "orientation_angular_speed_basis" and param.type_ == Parameter.Type.DOUBLE:
                self.orientation_angular_speed_basis = param.value
                results.append(True)
            else:
                self.get_logger().warn(f"Could not identify parameter {param}")
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def publish_heartbeat(self) -> None:
        self.publisher_heartbeat.publish(Bool())

    def subscribe_laser_scan(self, laser_scan: LaserScan):
        self.laser_scan = laser_scan

    def subscribe_bounding_boxes(self, bounding_boxes: Float64MultiArray):
        # bounding_box: [center_x, center_y, width, height, accuracy, class, ...] Array is flattened
        
        if type(bounding_boxes) != Float64MultiArray:
            self.get_logger().info("bounding_boxes is not Float64MultiArray")
            return

        if type(self.publisher_controls) != Publisher:
            self.get_logger().info("self.publisher_controls is not Publisher")
            return

        angle_distance_blue, angle_distance_yellow = self.get_nearest_cones(bounding_boxes)

        if angle_distance_blue == None:
            self.get_logger().info("angle_distance_blue is None")
            self.orientate(True)
            return

        if angle_distance_yellow == None:
            self.get_logger().info("angle_distance_yellow is None")
            self.orientate(False)
            return

        self.drive(angle_distance_blue, angle_distance_yellow)

    def orientate(self, left: bool) -> None:
        if not left:
            left = -1

        self.twist.angular.z = left * min(self.orientation_angular_speed_start * self.orientation_angular_speed_basis**self.orientate_counter, self.orientation_angular_speed_max)
        self.twist.linear.x = self.orientation_linear_speed

        self.drive_counter = 0
        self.orientate_counter += 1 

        self.publisher_controls.publish(self.twist)

    def drive(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        angle = self.get_angle(angle_distance_blue, angle_distance_yellow)

        self.twist.linear.x = min(self.drive_linear_speed_start * self.drive_linear_speed_basis**self.drive_counter, self.drive_linear_speed_max)
        self.twist.angular.z = min(abs(angle), self.drive_angular_speed_max) * (1 if angle > 0 else -1)

        self.get_logger().info(f"Set angular speed to {self.twist.angular.z}")

        self.orientate_counter = 0
        self.drive_counter += 1

        self.publisher_controls.publish(self.twist)

    def get_angle(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        c = self.get_distance_between_cones(angle_distance_blue, angle_distance_yellow)
        s_c = self.get_median_triangle_c(angle_distance_yellow[1], angle_distance_blue[1], c)
        x_m = self.get_center_between_cones(angle_distance_blue, angle_distance_yellow)

        if s_c == 0:
            self.get_logger().info("s_c is 0")
            return 0

        angle = asin(abs(x_m) / s_c)

        if x_m < 0:
            angle = -angle

        return angle

    def get_median_triangle_c(self, a: float, b: float, c: float) -> float:
        return sqrt(2 * (a*a + b*b) - c*c) / 2

    def get_distance_between_cones(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        alpha, a = angle_distance_yellow
        beta, b = angle_distance_blue
        
        return sqrt(a*a + b*b - 2*a*b*cos(radians(alpha + beta)))

    def get_nearest_cones(self, bounding_boxes: Float64MultiArray) -> tuple:
        if type(self.laser_scan) != LaserScan:
            self.get_logger().info("self.laser_scan is not LaserScan")
            return (None, None)

        laser_scan = deepcopy(self.laser_scan)
        angle_distance_blue = []
        angle_distance_yellow = []

        for i in range(len(bounding_boxes.data) // 6):
            center_x, cy, width, h, accuracy, cone_class = bounding_boxes.data[i*6:i*6 + 6]

            if accuracy < Controller.ACCURACY_THRESHOLD:
                continue

            if width < Controller.WIDTH_THRESHOLD:
                continue

            index = self.pixel_to_index(center_x)

            if laser_scan.ranges[index] == np.inf:
                continue

            if cone_class == Controller.CONE_CLASS_BLUE:
                angle_distance = angle_distance_blue
            elif cone_class == Controller.CONE_CLASS_YELLOW:
                angle_distance = angle_distance_yellow
            else:
                continue

            angle_distance.append((index, laser_scan.ranges[index]))
            
        angle_distance_blue = min(angle_distance_blue, key=lambda angle_distance: angle_distance[1], default=None)
        angle_distance_yellow = min(angle_distance_yellow, key=lambda angle_distance: angle_distance[1], default=None)

        return (angle_distance_blue, angle_distance_yellow)

    def get_center_between_cones(self, cone_blue: tuple, cone_yellow: tuple) -> float:
        angle_blue, distance_blue = cone_blue
        angle_yellow, distance_yellow = cone_yellow

        adjacent_blue = self.get_adjacent(distance_blue, radians(90 - abs(angle_blue)))
        adjacent_yellow = self.get_adjacent(distance_yellow, radians(90 - abs(angle_yellow)))

        if angle_blue < 0:
            adjacent_blue = -adjacent_blue
        if angle_yellow < 0:
            adjacent_yellow = -adjacent_yellow

        return (adjacent_blue + adjacent_yellow) / 2

    def get_adjacent(self, hypotenuse: float, alpha: float) -> float:
        return cos(alpha) * hypotenuse

    def pixel_to_index(self, pixel: float) -> int:
        return round(self.map_range_to(
            Controller.WIDTH_MIN, Controller.WIDTH_MAX,
            Controller.ANGLE_DEGREES_START, Controller.ANGLE_DEGREES_STOP,
            pixel))

    def map_range_to(self, input_start, input_end, output_start, output_end, value):
        # https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
        return output_start + ((output_end - output_start) / (input_end - input_start)) * (value - input_start)

    def destroy_node(self) -> bool:
        for subscription in self.subscriptions:
            subscription.destroy()

        self.publisher_controls.publish(Twist())

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
