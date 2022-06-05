import rclpy
import torch
from cv2 import COLOR_BGR2RGB, COLOR_RGB2BGR, cvtColor, imshow, waitKey
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
from datetime import datetime


class Detection(Node):

    def __init__(self):
        super().__init__('perception_detection')
        self.subscriber_queue_size = 2
        self.publisher_queue_size = 2
        self.cv_window_name = "Detection"
        self.cv_wait_ms = 10
        self.is_logging = True

        self.model = None
        self.publisher = None
        self.is_displaying = False
        self.cv_bridge = CvBridge()

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_topic_name", "frames_processed")
        self.declare_parameter("publisher_topic_name", "bounding_boxes")
        self.declare_parameter("model", "/home/ubuntu/models/yolov5s_cones.pt")
        self.declare_parameter("is_displaying", self.is_displaying)

    def init_subscriber(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(CompressedImage, topic_name, self.subscribe, self.subscriber_queue_size)) == Subscription

    def init_publisher(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        self.publisher = self.create_publisher(
            Float64MultiArray, topic_name, self.publisher_queue_size)
        return type(self.publisher) == Publisher

    def init_model(self, path) -> bool:
        try:
            model = torch.hub.load(
                "ultralytics/yolov5",
                "custom",
                path=path,
                force_reload=True,
                autoshape=True)
        except:
            self.get_logger().error(f"Coulnt load model from path {path}")
            return False

        if model == None:
            self.get_logger().error(f"Coulnt load model from path {path}")
            return False

        self.model = model
        return True

    def init_is_displaying(self, is_displaying: bool) -> bool:
        if type(is_displaying) != bool:
            return False

        self.is_displaying = is_displaying
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

            elif param.name == "publisher_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher))

            elif param.name == "model":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_model))

            elif param.name == "is_displaying":
                results.append(self.initializer(
                    param, (Parameter.Type.BOOL,), self.init_is_displaying))

            else:
                results.append(False)
                break

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, msg) -> None:
        frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        frame = cvtColor(frame, COLOR_BGR2RGB)

        if self.is_logging:
            t0 = datetime.now()

        result = self.model(frame)

        if self.is_logging:
            delta = datetime.now() - t0
            self.get_logger().info(f"[DETECTION/TIMEDELTA]{delta.seconds}.{delta.microseconds}")

        if self.is_displaying:
            cv_frame = cvtColor(result.render()[0], COLOR_RGB2BGR)
            imshow(self.cv_window_name, cv_frame)
            waitKey(self.cv_wait_ms)

        self.publish(result.xywhn[0])

    def publish(self, bounding_boxes: torch.Tensor) -> None:
        if type(self.publisher) != Publisher:
            return

        points = 6
        number_objects = bounding_boxes.shape[0]

        message = Float64MultiArray()
        message.data = bounding_boxes.reshape(points*number_objects).tolist()

        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)

    detection = Detection()
    # spin node so callback function is called
    rclpy.spin(detection)

    detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
