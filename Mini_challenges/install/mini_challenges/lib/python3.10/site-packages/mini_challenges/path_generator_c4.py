import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import numpy as np

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('N', 4),
            ]
        )

        self.N = int(self.get_parameter('N').get_parameter_value().integer_value)

        # Publisher
        self.publish_path = self.create_publisher(Pose2D, 'goal', 10)

        # Subscriber
        self.create_subscription(Bool, 'ready', self.flag_callback, 10)

        # Variables
        self.angle = 0.0
        self.radius = 1.0
        self.goal = Pose2D()

        # Flags
        self.ready = True

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def flag_callback(self, msg):
        self.ready = msg.data

    def timer_callback(self):
        self.N = int(self.get_parameter('N').get_parameter_value().integer_value)

        if self.ready:
            self.angle += 2 * np.pi / self.N
            if self.angle > 2 * np.pi:
                self.angle -= 2 * np.pi
            self.goal.x = self.radius * np.cos(self.angle) + 1
            self.goal.y = self.radius * np.sin(self.angle)
            self.publish_path.publish(self.goal)
            self.get_logger().info('Goal: x = %f, y = %f' % (self.goal.x, self.goal.y))
            self.ready = False

def main(args=None):
    rclpy.init(args=args)

    path_generator = PathGenerator()

    rclpy.spin(path_generator)

    path_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()