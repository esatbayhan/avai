import rclpy
from cv_bridge import \
    CvBridge  # Package to convert Image and CompressedImage
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import CompressedImage, Image


class Simcam(Node):

    def __init__(self):
        super().__init__('simcam')  # init node with name
        self.image_format = "jpg"
        self.publisher_queue_size = 2
        self.subscriber_queue_size = 2

        self.publisher = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter('subscriber_topic_name', "camera/image_raw")
        self.declare_parameter("publisher_topic_name", "frames_processed")

        self.cv_bridge = CvBridge()  # object to convert Image to CompressedImage

    def init_subscriber(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(Image, topic_name, self.subscribe, self.subscriber_queue_size)) == Subscription

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
            if param.name == "subscriber_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber))

            elif param.name == "publisher_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher))

            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, image: Image) -> None:
        # Image -> Numpy -> CompressedImage
        self.publish(
            self.cv_bridge.cv2_to_compressed_imgmsg(
                self.cv_bridge.imgmsg_to_cv2(image)))

    def publish(self, compressed_image: CompressedImage) -> None:
        if type(compressed_image) != CompressedImage:
            return

        self.publisher.publish(compressed_image)


def main(args=None):

    rclpy.init(args=args)

    simcam = Simcam()
    rclpy.spin(simcam)  # spin node so callback function is called

    simcam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
