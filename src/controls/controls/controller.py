from copy import deepcopy
from math import asin, cos, isnan, radians, sin, sqrt

import rclpy
from geometry_msgs.msg import Twist
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
    ACCURACY_THRESHOLD = 0.6
    WIDTH_THRESHOLD = 0.04
    WIDTH_MIN, WIDTH_MAX = 0, 1

    # Laser Scan
    ANGLE_DEGREES_START = 31.1
    ANGLE_DEGREES_STOP = -31.1

    RELATIVE_DISTANCE_CONES_THRESHOLD = 3.0

    def __init__(self):
        super().__init__("controller")
        self.subscriber_queue_size = 2
        self.publisher_queue_size = 0

        self.laser_scan = None
        self.publisher_commandos = None

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
        self.orientation_counter_threshold = 0

        self.add_on_set_parameters_callback(self.init_parameters)

        self.declare_parameter("publisher_name_heartbeat", "heartbeat")
        self.declare_parameter("subscriber_name_laser_scan", "scan")
        self.declare_parameter("subscriber_name_bounding_boxes", "bounding_boxes")
        self.declare_parameter("publisher_name_commandos", "commandos")

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
        self.declare_parameter("orientation_counter_threshold", 30)

    def init_publisher_heartbeat(self, topic_name: str) -> bool:
        self.publisher_heartbeat = self.create_publisher(
            Bool, topic_name, qos_profile=qos_profile_sensor_data)
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

    def init_publisher_commandos(self, topic_name: str) -> bool:
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        self.publisher_commandos = self.create_publisher(
            Twist, topic_name, self.publisher_queue_size)

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
            elif param.name == "subscriber_name_laser_scan":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_laser_scan))
            elif param.name == "subscriber_name_bounding_boxes":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_bounding_boxes))
            elif param.name == "publisher_name_commandos":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher_commandos))

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
            elif param.name == "orientation_counter_threshold" and param.type_ == Parameter.Type.INTEGER:
                self.orientation_counter_threshold = param.value
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
            self.get_logger().warn(
                f"bounding_boxes is not Float64MultiArray but {type(bounding_boxes)}")
            return

        if type(self.publisher_commandos) != Publisher:
            self.get_logger().warn(
                f"self.publisher_commandos is not Publisher but {type(self.publisher_commandos)}")
            return

        if type(self.laser_scan) != LaserScan:
            self.get_logger().warn(
                f"self.laser_scan is not LaserScan but {type(self.laser_scan)}")
            return

        angle_distance_blue, angle_distance_yellow = self.get_nearest_cones(
            bounding_boxes, deepcopy(self.laser_scan))

        if angle_distance_blue == None:
            self.get_logger().info("angle_distance_blue is None")
            self.orientate(left=True)
            return

        if angle_distance_yellow == None:
            self.get_logger().info("angle_distance_yellow is None")
            self.orientate(left=False)
            return

        if angle_distance_blue[1] <= 0.0:
            self.get_logger().warn("distance to blue cone is <= 0")
            return

        if angle_distance_yellow[1] <= 0.0:
            self.get_logger().warn("distance to yellow cone is <= 0")
            return

        if angle_distance_blue[1] >= angle_distance_yellow[1] * Controller.RELATIVE_DISTANCE_CONES_THRESHOLD:
            self.get_logger().info(
                f"blue cone is to far away: blue cone {angle_distance_blue}, yellow cone: {angle_distance_yellow}")
            self.orientate(left=True)
            return

        if angle_distance_yellow[1] >= angle_distance_blue[1] * Controller.RELATIVE_DISTANCE_CONES_THRESHOLD:
            self.get_logger().info(
                f"yellow cone is to far away: yellow cone {angle_distance_blue}, blue cone: {angle_distance_yellow}")
            self.orientate(left=False)
            return

        self.drive(angle_distance_blue, angle_distance_yellow)

    def orientate(self, left: bool) -> None:
        if not left:
            left = -1

        self.twist.angular.z = left * min(self.orientation_angular_speed_start *
                                          self.orientation_angular_speed_basis**self.orientate_counter, self.orientation_angular_speed_max)
        self.twist.linear.x = self.orientation_linear_speed

        if self.orientate_counter > self.orientation_counter_threshold:
            self.twist.linear.x = 0.0

        self.drive_counter = 0
        self.orientate_counter += 1

        self.publisher_commandos.publish(self.twist)

    def drive(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        angle = self.get_angle(angle_distance_blue, angle_distance_yellow)

        self.twist.linear.x = min(self.drive_linear_speed_start *
                                  self.drive_linear_speed_basis**self.drive_counter, self.drive_linear_speed_max)
        self.twist.angular.z = min(
            abs(angle) * 0.5, self.drive_angular_speed_max) * (1 if angle > 0 else -1)

        self.get_logger().info(f"Set angular speed to {self.twist.angular.z}")

        self.orientate_counter = 0
        self.drive_counter += 1

        self.publisher_commandos.publish(self.twist)

    def get_angle(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        """Returns the angle of the turtlebot and the middlepoint between the blue and yellow cone. If any calculation goes wrong it returns 0.0.

        Args:
            angle_distance_blue (tuple): (angle, distance) of the blue cone. Angle is in radians.
            angle_distance_yellow (tuple): (angle, distance) of the yellow cone. Angle is in radians. 

        Returns:
            float: angle. Angle is in radians.
        """
        c = self.get_distance_between_cones(
            angle_distance_blue, angle_distance_yellow)
        s_c = self.get_median_triangle_c(
            angle_distance_yellow[1], angle_distance_blue[1], c)
        x_m = self.get_center_between_cones(
            angle_distance_blue, angle_distance_yellow)

        if c == 0:
            self.get_logger().warn(
                f"c is 0, angle_distance_blue is {angle_distance_blue}, angle_distance_yellow is: {angle_distance_yellow}")
            return 0.0

        if s_c == 0:
            self.get_logger().info("s_c is 0")
            return 0.0

        if x_m == 0:
            self.get_logger().info("x_m is 0")
            return 0.0

        angle = asin(abs(x_m) / s_c)

        if isnan(angle):
            return 0.0

        if x_m < 0:
            angle = -angle

        return angle

    def get_median_triangle_c(self, a: float, b: float, c: float) -> float:
        # https://de.wikipedia.org/wiki/Seitenhalbierende#Eigenschaften
        return sqrt(2 * (a**2 + b**2) - c**2) / 2

    def get_distance_between_cones(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        # https://de.wikipedia.org/wiki/Kosinussatz#Allgemeine_Formulierung
        alpha, a = angle_distance_yellow
        beta, b = angle_distance_blue

        alpha = abs(alpha)
        beta = abs(beta)

        return sqrt(a*a + b*b - 2*a*b*cos(alpha + beta))

    def get_nearest_cones(self, bounding_boxes: Float64MultiArray, laser_scan: LaserScan) -> tuple:
        """Match bounding box of a cone with data from the LiDAR and return the nearest blue and yellow cone.

        Args:
            bounding_boxes (Float64MultiArray): Flattened array of cone bounding boxes. [center_x, center_y, width, height, accuracy, class, ...]
            laser_scan (LaserScan): Current datapoints of the LiDAR sensor

        Returns:
            tuple: ((angle_blue_cone, distance_blue_cone), (angle_yellow_cone, distance_yellow_cone)). Angles are in radians. Distances are in meters.
        """
        angle_distance_blue_cones = []
        angle_distance_yellow_cones = []

        for i in range(len(bounding_boxes.data) // 6):
            center_x, cy, width, h, accuracy, cone_class = bounding_boxes.data[i*6:i*6 + 6]

            if accuracy < Controller.ACCURACY_THRESHOLD:
                continue

            if width < Controller.WIDTH_THRESHOLD:
                continue

            index = self.pixel_to_index(center_x)

            if laser_scan.ranges[index] == 0.0:
                continue

            if cone_class == Controller.CONE_CLASS_BLUE:
                angle_distance = angle_distance_blue_cones
            elif cone_class == Controller.CONE_CLASS_YELLOW:
                angle_distance = angle_distance_yellow_cones
            else:
                continue

            angle_distance.append((radians(index), laser_scan.ranges[index]))

        angle_distance_blue_nearest = min(
            angle_distance_blue_cones, key=lambda angle_distance: angle_distance[1], default=None)
        angle_distance_yellow_nearest = min(
            angle_distance_yellow_cones, key=lambda angle_distance: angle_distance[1], default=None)

        return (angle_distance_blue_nearest, angle_distance_yellow_nearest)

    def get_center_between_cones(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        angle_blue, distance_blue = angle_distance_blue
        angle_yellow, distance_yellow = angle_distance_yellow

        adjacent_blue = self.get_opposite(distance_blue, abs(angle_blue))
        adjacent_yellow = self.get_opposite(distance_yellow, abs(angle_yellow))

        if angle_blue < 0:
            adjacent_blue = -adjacent_blue
        if angle_yellow < 0:
            adjacent_yellow = -adjacent_yellow

        return (adjacent_blue + adjacent_yellow) / 2

    def get_opposite(self, hypotenuse: float, alpha: float) -> float:
        return sin(alpha) * hypotenuse

    def pixel_to_index(self, pixel: float) -> int:
        return round(self.map_range_to(
            Controller.WIDTH_MIN, Controller.WIDTH_MAX,
            Controller.ANGLE_DEGREES_START, Controller.ANGLE_DEGREES_STOP,
            pixel))

    def map_range_to(self, input_start, input_end, output_start, output_end, value):
        # https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
        return output_start + ((output_end - output_start) / (input_end - input_start)) * (value - input_start)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
