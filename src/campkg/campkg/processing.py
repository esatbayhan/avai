import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from sensor_msgs.msg import CompressedImage  # Image is the message type


class Processing(Node):

    def __init__(self):
        super().__init__("campkg_processing")  # init node with name
        self.subscriber_queue_size = 2
        self.publisher_queue_size = 2

        self.chached_frame = None
        self.publisher = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_topic_name", "frames_raw")
        self.declare_parameter("publisher_topic_name", "frames_processed")
        self.declare_parameter("fps", 10)

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
            CompressedImage, topic_name, self.publisher_queue_size)
        return type(self.publisher) == Publisher

    def init_timer(self, fps: int) -> bool:
        if type(fps) != int:
            return False

        for timer in self.timers:
            self.destroy_timer(timer)

        if fps <= 0:
            return True

        return type(self.create_timer(1 / fps, self.publish)) == Timer

    def initializer(self, param: Parameter, valid_types: tuple, init_function) -> bool:
        if param.type_ not in valid_types:
            return False

        return init_function(param.value)

    def init_parameters(self, params):
        results = []

        for param in params:
            if param.name == "fps":
                results.append(self.initializer(
                    param, (Parameter.Type.INTEGER,), self.init_timer))

            elif param.name == "subscriber_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber))

            elif param.name == "publisher_topic_name":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_publisher))

            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, msg: CompressedImage):
        self.chached_frame = msg

    def publish(self):
        if type(self.publisher) != Publisher:
            return

        if type(self.chached_frame) != CompressedImage:
            return

        self.publisher.publish(self.chached_frame)


def main(args=None):

    rclpy.init(args=args)

    processing = Processing()
    # spin node so callback function is called
    rclpy.spin(processing)

    processing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
