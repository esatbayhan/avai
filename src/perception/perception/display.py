import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from std_msgs.msg import Float64MultiArray
from tabulate import tabulate


class Display(Node):

    def __init__(self):
        super().__init__('perception_display')
        self.subscriber_queue_size = 2

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_topic_name", "bounding_boxes")

    def init_subscriber(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(Float64MultiArray, topic_name, self.subscribe, self.subscriber_queue_size)) == Subscription

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

            else:
                results.append(False)
                break

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscribe(self, msg: Float64MultiArray) -> None:
        data = [msg.data[i*6:i*6+6] for i in range(len(msg.data) // 6)]
        print(tabulate(data))


def main(args=None):
    rclpy.init(args=args)

    display = Display()
    # spin node so callback function is called
    rclpy.spin(display)

    display.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
