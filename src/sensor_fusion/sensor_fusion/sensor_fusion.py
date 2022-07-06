import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from sensor_msgs.msg import LaserScan, CameraInfo
from std_msgs.msg import Float64MultiArray
# from ros2_numpy import point_cloud2
# from image_geometry import PinholeCameraModel
import joblib
import numpy as np


class SensorFusion(Node):

    def __init__(self):
        super().__init__("sensor_fusion")  # init node with name
        self.subscriber_queue_size = 2

        # self.pinhole_camera_model = None
        self.scan_msg = None

        self.width = 640
        self.height = 480

        self.publisher_queue_size = 2
        self.publisher = None

        # self.camera_info = None
        # self.subscriber_camera_info = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_topic_name", "/scan")
        self.declare_parameter("subscriber_bounding_boxes", "/bounding_boxes")
        self.declare_parameter("publisher_topic_name", "filtered_scan")
        # self.declare_parameter("camera_info_topic_name", "/camera/camera_info")

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


    # def init_subscriber_camera_info(self, topic_name: str) -> bool:
    #     # ToDo(0) topic names have naming specifications
    #     # watch here https://design.ros2.org/articles/topic_and_service_names.html
    #     # and make sure topic_name is a valid topic name
    #     if type(topic_name) != str:
    #         return False

    #     if len(topic_name) == 0:
    #         return False

    #     self.subscriber_camera_info = self.create_subscription(CameraInfo, topic_name, self.init_camera_model, self.subscriber_queue_size)
    #     return True

    # def init_camera_model(self, camera_info: CameraInfo) -> None:
    #     self.destroy_subscription(self.subscriber_camera_info)
    #     self.pinhole_camera_model = PinholeCameraModel()
    #     self.pinhole_camera_model.fromCameraInfo(camera_info)

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
            # elif param.name == "camera_info_topic_name":
            #     results.append(self.initializer(
            #         param, (Parameter.Type.STRING,), self.init_subscriber_camera_info))
            elif param.name == "publisher_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher))
            elif param.name == "subscriber_bounding_boxes":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_bounding_boxes))
            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, msg: LaserScan):
        self.scan_msg = msg
        
        # joblib.dump(msg, '/home/parallels/stored_joblib/scan_msg.joblib')

    
    def subscribe_bounding_boxes(self, msg: Float64MultiArray):
        if self.scan_msg:
            scan_msg = self.scan_msg
            bboxes  = np.array(msg.data).reshape(len(msg.data)//6, 6)

            sensor_ranges_lr = np.append(scan_msg.ranges[-31:], scan_msg.ranges[:31])

            angel_increment = scan_msg.angle_increment
            laser_pixel_intervall = 640 / (1.085595 / angel_increment)

            x1 = bboxes[:, 0] * 640
            x2 = (bboxes[:, 0] + bboxes[:, 2]) * 640
            cone_range_index = []

            for i in range(len(bboxes)):
                laser_index_1 = int(x1[i]/laser_pixel_intervall) - 6
                laser_index_2 = int(x2[i]/laser_pixel_intervall) + 6
                start_index = np.where(sensor_ranges_lr[laser_index_1:laser_index_2] != np.inf)
                cone_index = start_index[0] + laser_index_1
                for index in cone_index:
                    if index < 31: 
                        cone_range_index.append(index + 329)
                    else:
                        cone_range_index.append(index - 31)

            l = [ind for ind in range(360) if ind not in cone_range_index]
            for b in l:
                scan_msg.ranges[b] = np.inf
            
            self.publish(scan_msg)
        else:
            self.get_logger().info("scan is empty")
        

        # self.get_logger().info("Received bounding boxes")
        # joblib.dump(msg, '/home/parallels/stored_joblib/bba.joblib')
            


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
