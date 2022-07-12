from copy import deepcopy

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
from math import cos, radians, asin, sqrt


class WaypointFilter(Node):
    CONE_CLASS_BLUE = 0
    CONE_CLASS_ORANGE = 1
    CONE_CLASS_YELLOW = 2
    ACCURACY_THRESHOLD = 0.4
    WIDTH_THRESHOLD = 0.04

    INCLUDE_MIN = -1
    INCLUDE_MAX = 1

    WIDTH = 1
    HEIGHT = 1

    ANGLE_DEGREES_START = 31.1
    ANGLE_DEGREES_STOP = -31.1

    LINEAR_SPEED = 1.0
    ANGULAR_SPEED = 1.0

    def __init__(self):
        super().__init__("waypoint_filter")
        self.subscriber_queue_size = 2
        self.publisher_queue_size = 0

        self.laser_scan = None
        self.publisher_controls = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_laser_scan", "scan")
        self.declare_parameter("subscriber_bounding_boxes", "bounding_boxes")
        self.declare_parameter("publisher_controls", "cmd_vel")

    def init_subscriber_laser_scan(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(LaserScan, topic_name, self.subscribe_laser_scan, self.subscriber_queue_size)) == Subscription

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

        self.publisher = self.create_publisher(Twist, topic_name, self.publisher_queue_size)

    def initializer(self, param: Parameter, valid_types: tuple, init_function) -> bool:
        if param.type_ not in valid_types:
            return False

        return init_function(param.value)

    def init_parameters(self, params):
        results = []

        for param in params:
            if param.name == "subscriber_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_laser_scan))
            elif param.name == "subscriber_bounding_boxes":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_bounding_boxes))
            elif param.name == "publisher_controls":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher_controls))
            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe_laser_scan(self, laser_scan: LaserScan):
        self.laser_scan = laser_scan

    def subscribe_bounding_boxes(self, bounding_boxes: Float64MultiArray):
        # bounding_box: [center_x, center_y, width, height, accuracy, class, ...] Array is flattened
        
        if type(bounding_boxes) != Float64MultiArray:
            return

        if type(self.publisher_controls) != Publisher:
            return

        angle_distance_blue, angle_distance_yellow = self.get_nearest_cones(bounding_boxes)

        if angle_distance_blue == None or angle_distance_yellow == None:
            return

        self.drive(angle_distance_blue, angle_distance_yellow)

    def drive(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        angle = self.get_angle(angle_distance_blue, angle_distance_yellow)

        twist = Twist()
        linear = Vector3()
        linear.x = WaypointFilter.LINEAR_SPEED
        twist.linear = linear
        angular = Vector3()
        angular.z = WaypointFilter.ANGULAR_SPEED * 1 if (angle > 0) else -1
        twist.angular = angular

        self.publisher_controls.publish(twist)
        self.create_rate(1 / abs(angle)).sleep()
        angular.z = 0
        twist.angular = angular
        self.publisher_controls.publish()

    def get_angle(self, angle_distance_blue: tuple, angle_distance_yellow: tuple) -> float:
        c = self.get_distance_between_cones(angle_distance_blue, angle_distance_yellow)
        s_c = self.get_median_triangle_c(angle_distance_yellow[1], angle_distance_blue[1], c)
        x_m = self.get_center_between_cones(angle_distance_blue, angle_distance_yellow)

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
            (None, None)

        laser_scan = deepcopy(self.laser_scan)
        angle_distance_blue = []
        angle_distance_yellow = []

        for i in range(len(bounding_boxes.data) // 6):
            center_x, cy, width, h, accuracy, cone_class = bounding_boxes.data[i*6:i*6 + 6]

            if accuracy < WaypointFilter.ACCURACY_THRESHOLD:
                continue

            if width < WaypointFilter.WIDTH_THRESHOLD:
                continue

            index = self.pixel_to_index(center_x)

            if laser_scan.ranges[index] == np.inf:
                continue

            if cone_class == WaypointFilter.CONE_CLASS_BLUE:
                angle_distance = angle_distance_blue
            elif cone_class == WaypointFilter.CONE_CLASS_YELLOW:
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
            0, WaypointFilter.WIDTH,
            WaypointFilter.ANGLE_DEGREES_START, WaypointFilter.ANGLE_DEGREES_STOP,
            pixel))

    def map_range_to(self, input_start, input_end, output_start, output_end, value):
        # https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
        return output_start + ((output_end - output_start) / (input_end - input_start)) * (value - input_start)


def main(args=None):

    rclpy.init(args=args)

    lidar_filter = WaypointFilter()
    # spin node so callback function is called
    rclpy.spin(lidar_filter)

    lidar_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
