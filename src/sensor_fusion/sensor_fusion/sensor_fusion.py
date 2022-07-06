from copy import deepcopy
import re
from turtle import width
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from sensor_msgs.msg import LaserScan, CameraInfo
from std_msgs.msg import Float64MultiArray
import numpy as np
from copy import deepcopy


class SensorFusion(Node):
    CONE_CLASS_BLUE = 0
    CONE_CLASS_ORANGE = 1
    CONE_CLASS_YELLOW = 2

    def __init__(self):
        super().__init__("sensor_fusion")  # init node with name
        self.subscriber_queue_size = 2

        self.width = 640
        self.height = 480

        self.publisher_queue_size = 2
        
        self.cosinus_theta = None
        self.init_cosinus_theta(0.01745329251994, 360) # thats 1 degree in radiant
        self.pixel_x_nparr = None

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

        self.cosinus_theta = np.cos([half_pi - angle_increment for i in range(number_points)])

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

    def subscribe_bounding_boxes(self, bounding_boxes_message: Float64MultiArray):
        """"
        bounding_box: [center_x, center_y, width, height, class, accuracy, ...] Array is flattened
        """
        if type(self.laser_scan) != LaserScan:
            return

        bounding_boxes = self.unflatten_bounding_boxes(bounding_boxes_message)
        laser_scan = deepcopy(self.laser_scan)
        laser_scan_blue = deepcopy(laser_scan)
        laser_scan_yellow = deepcopy(laser_scan)
        pixel_x_nparr = self.get_pixel_x(laser_scan)

        # clear laser scans
        for i in range(len(laser_scan.ranges)):
            laser_scan_blue.ranges[i] = np.inf
            laser_scan_yellow.ranges[i] = np.inf

        # filter laser scan matching a bounding box and sort to corresponding laser_scan
        for i in range(len(pixel_x_nparr)):
            pixel_x = pixel_x_nparr[i]

            if pixel_x == np.inf:
                continue

            for x_center, ch, width, h, cone_class, a in bounding_boxes:
                shift = width // 2

                if x_center - shift < pixel_x / self.width < x_center + shift:
                    if cone_class == SensorFusion.CONE_CLASS_BLUE:
                        laser_scan_blue.ranges[i] = laser_scan.ranges[i]
                    elif cone_class == SensorFusion.CONE_CLASS_YELLOW:
                        laser_scan_yellow.ranges[i] = laser_scan.ranges[i]

                    break

        self.publisher_laser_scan_blue.publish(laser_scan_blue)
        self.publisher_laser_scan_yellow.publish(laser_scan_yellow)

    def get_pixel_nparr(self, laser_scan: LaserScan) -> np.array:
        """"
        Convert polar coordinate to pixel coordinate
        see: https://www.khanacademy.org/computing/computer-programming/programming-natural-simulations/programming-angular-movement/a/polar-coordinates
        """
        # cartesic coordinate has it origin at in the middle of the picture thats why we need to shift e. g. (0, 640) to (-320, 320)
        shift = self.width // 2
        max_x_shifted = shift
        min_x_shifted = -max_x_shifted

        return np.array([
            pixel_x + shift if max_x_shifted < pixel_x < min_x_shifted else np.inf
            for pixel_x in laser_scan.ranges * self.cosinus_theta])

    def unflatten_bounding_boxes(self, bounding_boxes: Float64MultiArray) -> np.array:
        """"
        Reshapes bounding_boxes to a 2 dimonsional array
        """

        number_bounding_box_attributes = 6 # center_x, center_y, width, height, class, accuracy
        number_bounding_boxes = len(bounding_boxes.data)

        return np.array(bounding_boxes.data).reshape(
            number_bounding_boxes // number_bounding_box_attributes, number_bounding_box_attributes)


def main(args=None):

    rclpy.init(args=args)

    sensor_fusion = SensorFusion()
    # spin node so callback function is called
    rclpy.spin(sensor_fusion)

    sensor_fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
