import rclpy
from cv2 import VideoCapture
from cv_bridge import \
    CvBridge  # Package to convert between ROS and OpenCV Images
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from sensor_msgs.msg import CompressedImage


class Camera(Node):

    def __init__(self):
        super().__init__('campkg_camera')  # init node with name
        self.image_format = "jpg"
        self.publisher_queue_size = 2

        self.publisher = None
        self.video_capture = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter("topic_name", "frames_raw")
        self.declare_parameter("fps", 30)

        self.cv_bridge = CvBridge()  # object to convert ROS2 to OpenCV image

    def init_camera(self, id) -> bool:
        if type(self.video_capture) == VideoCapture:
            self.video_capture.release()

        self.video_capture = VideoCapture(id)

        return self.video_capture.isOpened()

    def init_timer(self, fps: int) -> bool:
        if type(fps) != int:
            return False

        for timer in self.timers:
            self.destroy_timer(timer)

        if fps <= 0:
            return True

        return type(self.create_timer(1 / fps, self.publish)) == Timer

    def init_publisher(self, topic_name: str) -> bool:
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        self.publisher = self.create_publisher(
            CompressedImage, topic_name, self.publisher_queue_size)

        return type(self.publisher) == Publisher

    def initializer(self, param: Parameter, valid_types: tuple, init_function) -> bool:
        if param.type_ not in valid_types:
            return False

        return init_function(param.value)

    def init_parameters(self, params):
        results = []

        for param in params:
            if param.name == "camera_id":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING, Parameter.Type.INTEGER), self.init_camera))

            elif param.name == "fps":
                results.append(self.initializer(
                    param, (Parameter.Type.INTEGER,), self.init_timer))

            elif param.name == "topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher))

            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def publish(self) -> None:
        if type(self.video_capture) != VideoCapture:
            return

        ret, frame = self.video_capture.read()

        if ret:
            self.publisher.publish(
                self.cv_bridge.cv2_to_compressed_imgmsg(frame, self.image_format))
            self.get_logger().debug('Publishing video frame')
        else:
            self.get_logger().warning("Couldn't publish video frame")


def main(args=None):

    rclpy.init(args=args)

    camera = Camera()
    rclpy.spin(camera)  # spin node so callback function is called

    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
