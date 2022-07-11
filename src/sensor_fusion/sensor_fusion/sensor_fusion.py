import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import joblib


class SensorFusion(Node):

    def __init__(self):
        super().__init__("sensor_fusion")  # init node with name
        self.subscriber_queue_size = 2

        self.safe_counter = 0
        self.scan_msgs = []
        self.odom_msgs = []

        self.width = 640
        self.height = 480

        self.publisher_queue_size = 2
        self.publisher = None


        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_topic_name", "/scan")
        self.declare_parameter("subscriber_bounding_boxes", "/bounding_boxes")
        self.declare_parameter("subscriber_odometry", "/odom")
        self.declare_parameter("publisher_topic_name", "filtered_scan")

    def init_subscriber(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(LaserScan, topic_name, self.subscribe, self.subscriber_queue_size)) == Subscription
    
    def init_subscriber_bounding_boxes(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(Float64MultiArray, topic_name, self.subscribe_bounding_boxes, self.subscriber_queue_size)) == Subscription

    def init_subscriber_odometry(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(Odometry, topic_name, self.subscribe_odometry, self.subscriber_queue_size)) == Subscription


    def init_publisher(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        self.publisher = self.create_publisher(
            LaserScan, topic_name, self.publisher_queue_size)
        return type(self.publisher) == Publisher


    def initializer(self, param: Parameter, valid_types: tuple, init_function) -> bool:
        if param.type_ not in valid_types:
            return False

        return init_function(param.value)

    def init_parameters(self, params):
        results = []

        for param in params:
            if param.name == "subscriber_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber))
            elif param.name == "subscriber_odometry":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_odometry))
            elif param.name == "subscriber_bounding_boxes":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_bounding_boxes))
            elif param.name == "publisher_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher))
            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_msgs.append(msg)
    
    def subscribe_odometry(self, msg: Odometry):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msgs.append(msg)
    

    def subscribe_bounding_boxes(self, msg: Float64MultiArray):
        
        self.get_logger().info("Received bounding boxes {}".format(self.safe_counter))
        dump_data = {
            "scan_msgs": self.scan_msgs,
            "odom_msgs": self.odom_msgs,
            "bounding_boxes": msg,
        }
        joblib.dump(dump_data, '/home/parallels/stored_joblib/msg_dump{}.joblib'.format(self.safe_counter))
        self.safe_counter += 1
    
        

    def publish(self, scan_msg: LaserScan) -> None:
        if type(self.publisher) != Publisher:
            return

        self.get_logger().info("publishded filterd scan")
        self.publisher.publish(scan_msg)



def main(args=None):

    rclpy.init(args=args)

    sensor_fusion = SensorFusion()
    # spin node so callback function is called
    rclpy.spin(sensor_fusion)

    sensor_fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
