import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult


class WaypointCalculator(Node):

    def __init__(self):
        super().__init__("waypoint_calculator")  # init node with name
        self.subscriber_queue_size = 2
        self.publisher_queue_size = 2

        self.laser_scan_yellow_cones = None
        self.laser_scan_blue_cones = None

        self.add_on_set_parameters_callback(self.init_parameters)
        self.declare_parameter("subscriber_yellow_cones", "laser_scan_yellow_cones")
        self.declare_parameter("subscriber_blue_cones", "laser_scan_blue_cones")

    def init_subscriber_laser_scan_yellow_cones(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(LaserScan, topic_name, self.subscriber_laser_scan_yellow_cones, self.subscriber_queue_size)) == Subscription
    
    def init_subscriber_laser_scan_blue_cones(self, topic_name: str) -> bool:
        # ToDo(0) topic names have naming specifications
        # watch here https://design.ros2.org/articles/topic_and_service_names.html
        # and make sure topic_name is a valid topic name
        if type(topic_name) != str:
            return False

        if len(topic_name) == 0:
            return False

        return type(self.create_subscription(LaserScan, topic_name, self.subscriber_laser_scan_blue_cones, self.subscriber_queue_size)) == Subscription


    def initializer(self, param: Parameter, valid_types: tuple, init_function) -> bool:
        if param.type_ not in valid_types:
            return False

        return init_function(param.value)

    def init_parameters(self, params):
        results = []

        for param in params:
            if param.name == "subscriber_laser_scan_yellow_cones":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_laser_scan_yellow_cones))
            elif param.name == "subscriber_laser_scan_blue_cones":
                results.append(self.initializer(
                    param, (Parameter.Type.STRING,), self.init_subscriber_laser_scan_blue_cones))
            else:
                results.append(False)

        successful = len(results) > 0 and all(results)
        return SetParametersResult(successful=successful)

    def subscriber_laser_scan_yellow_cones(self, laser_scan_yellow_cones: LaserScan):
        self.laser_scan_yellow_cones = laser_scan_yellow_cones
        self.calculate_waypoints()

    def subscriber_laser_scan_blue_cones(self, laser_scan_blue_cones: LaserScan):
        self.laser_scan_blue_cones = laser_scan_blue_cones

    def calculate_waypoints(self):
        self.get_logger().info("calculate_waypoints")

        navigator = BasicNavigator()

        goal_poses = []

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.3
        goal_pose.pose.position.y = 6.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.23
        goal_pose.pose.orientation.w = 0.97
        goal_poses.append(goal_pose)
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = -3.5
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.707
        goal_pose.pose.orientation.w = -0.707
        goal_poses.append(goal_pose)

        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)
 


def main(args=None):

    rclpy.init(args=args)

    lidar_filter = WaypointCalculator()
    # spin node so callback function is called
    rclpy.spin(lidar_filter)

    lidar_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()