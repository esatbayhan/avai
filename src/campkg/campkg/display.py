from datetime import datetime

import rclpy
from cv2 import imshow, imwrite, waitKey
from cv_bridge import \
    CvBridge  # Package to convert between ROS and OpenCV Images
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from sensor_msgs.msg import CompressedImage  # Image is the message type


class Display(Node):

    def __init__(self):
        super().__init__('campkg_display')  # init node with name
        self.imwrite_path = "/home/ubuntu/Downloads"
        self.subscriber_queue_size = 2
        self.cv_window_name = "Frame"
        self.cv_wait_ms = 10

        self.is_saving = False
        self.cv_bridge = CvBridge()

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("is_saving", self.is_saving)
        self.declare_parameter("subscriber_topic_name", "frames_processed")

    def init_subscriber(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(CompressedImage, topic_name, self.subscribe, self.subscriber_queue_size)) == Subscription

    def init_is_saving(self, is_saving: bool) -> bool:
        if type(is_saving) != bool:
            return False

        self.is_saving = is_saving

        return True

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

            elif param.name == "is_saving":
                results.append(self.initializer(
                    param, (Parameter.Type.BOOL,), self.init_is_saving))

            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, msg) -> None:
        try:
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

            imshow(self.cv_window_name, frame)
            waitKey(self.cv_wait_ms)

            if self.is_saving:
                imwrite(
                    f"{self.imwrite_path}/{datetime.now().isoformat()}.jpg", frame)

        except Exception as e:
            self.get_logger().error(f"Error occured {e}")


def main(args=None):
    rclpy.init(args=args)

    display = Display()
    rclpy.spin(display)

    display.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
