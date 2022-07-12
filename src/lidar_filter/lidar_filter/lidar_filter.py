from copy import deepcopy

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray


class LidarFilter(Node):
    CONE_CLASS_BLUE = 0
    CONE_CLASS_ORANGE = 1
    CONE_CLASS_YELLOW = 2
    ACCURACY_THRESHOLD = 0.4
    WIDTH_THRESHOLD = 0.04

    INCLUDE_MIN = -1
    INCLUDE_MAX = 1

    def __init__(self):
        super().__init__("lidar_filter")
        self.subscriber_queue_size = 2

        self.width = 640
        self.height = 480

        self.angle_degrees_start = 31.1
        self.angle_degrees_stop = -31.1

        self.publisher_queue_size = 2

        self.cosinus_theta = None
        # thats 1 degree in radiant
        self.init_cosinus_theta(0.01745329251994, 360)
        self.pixel_x_nparr = None
        self.laser_scan = None

        self.publisher_laser_scan_yellow = self.create_publisher(
            LaserScan, "laser_scan_yellow_cones", self.publisher_queue_size)
        self.publisher_laser_scan_blue = self.create_publisher(
            LaserScan, "laser_scan_blue_cones", self.publisher_queue_size)

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_topic_name", "scan")
        self.declare_parameter("subscriber_bounding_boxes", "bounding_boxes")

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

    def init_cosinus_theta(self, angle_increment: float, number_points: int):
        if self.cosinus_theta:
            return

        half_pi = np.pi / 2

        self.cosinus_theta = np.cos(
            [half_pi - angle_increment for i in range(number_points)])

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
            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe_laser_scan(self, laser_scan: LaserScan):
        self.laser_scan = laser_scan

    def subscribe_bounding_boxes(self, bounding_boxes: Float64MultiArray):
        """"
        bounding_box: [center_x, center_y, width, height, accuracy, class, ...] Array is flattened
        """
        if type(self.laser_scan) != LaserScan:
            return

        laser_scan = deepcopy(self.laser_scan)
        laser_scan_blue = deepcopy(laser_scan)
        laser_scan_yellow = deepcopy(laser_scan)

        # clear laser scans
        for i in range(len(laser_scan.ranges)):
            laser_scan_blue.ranges[i] = np.inf
            laser_scan_yellow.ranges[i] = np.inf
            laser_scan_blue.intensities[i] = 0
            laser_scan_yellow.intensities[i] = 0

        for i in range(len(bounding_boxes.data) // 6):
            center_x, cy, width, h, accuracy, cone_class = bounding_boxes.data[i*6:i*6 + 6]

            if accuracy < LidarFilter.ACCURACY_THRESHOLD:
                continue

            if width < LidarFilter.WIDTH_THRESHOLD:
                continue

            index = self.pixel_to_index(center_x)

            if cone_class == LidarFilter.CONE_CLASS_BLUE:
                scan = laser_scan_blue
            elif cone_class == LidarFilter.CONE_CLASS_YELLOW:
                scan = laser_scan_yellow
            else:
                continue

            for shift in range(LidarFilter.INCLUDE_MIN, LidarFilter.INCLUDE_MAX + 1):
                scan.ranges[index+shift] = laser_scan.ranges[index+shift]
                scan.intensities[index +
                                 shift] = laser_scan.intensities[index+shift]

        self.publisher_laser_scan_blue.publish(laser_scan_blue)
        self.publisher_laser_scan_yellow.publish(laser_scan_yellow)

    def pixel_to_index(self, pixel: float) -> int:
        return round(self.map_range_to(
            0, 1,
            31.1, -31.1,
            pixel))

    def map_range_to(self, input_start, input_end, output_start, output_end, value):
        # https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
        return output_start + ((output_end - output_start) / (input_end - input_start)) * (value - input_start)


def main(args=None):

    rclpy.init(args=args)

    lidar_filter = LidarFilter()
    # spin node so callback function is called
    rclpy.spin(lidar_filter)

    lidar_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
