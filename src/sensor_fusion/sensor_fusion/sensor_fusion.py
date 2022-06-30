import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from sensor_msgs.msg import PointCloud2, CameraInfo
from ros2_numpy import point_cloud2
from image_geometry import PinholeCameraModel


class SensorFusion(Node):

    def __init__(self):
        super().__init__("sensor_fusion")  # init node with name
        self.subscriber_queue_size = 2

        self.pinhole_camera_model = None

        self.width = 640
        self.height = 480

        self.camera_info = None
        self.subscriber_camera_info = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_topic_name", "scan_matched_points2")
        self.declare_parameter("camera_info_topic_name", "/camera/camera_info")

    def init_subscriber(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(PointCloud2, topic_name, self.subscribe, self.subscriber_queue_size)) == Subscription
    
    def init_subscriber_camera_info(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        self.subscriber_camera_info = self.create_subscription(CameraInfo, topic_name, self.init_camera_model, self.subscriber_queue_size)
        return True

    def init_camera_model(self, camera_info: CameraInfo) -> None:
        self.destroy_subscription(self.subscriber_camera_info)
        self.pinhole_camera_model = PinholeCameraModel()
        self.pinhole_camera_model.fromCameraInfo(camera_info)

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
            elif param.name == "camera_info_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_camera_info))
            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, msg: PointCloud2):
        if (self.pinhole_camera_model == None):
            return
            
        for x, y, z in point_cloud2.pointcloud2_to_xyz_array(msg):
            px, py = self.pinhole_camera_model.rectifyPoint(
                        self.pinhole_camera_model.project3dToPixel((x, y, z)))
            
            if 0 < px < self.width or 0 < py < self.height:
                print(px, py)


def main(args=None):

    rclpy.init(args=args)

    sensor_fusion = SensorFusion()
    # spin node so callback function is called
    rclpy.spin(sensor_fusion)

    sensor_fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
