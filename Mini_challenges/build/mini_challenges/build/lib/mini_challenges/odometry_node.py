import rclpy
from rclpy.node import Node
import rclpy.qos
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, Twist
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry_Node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('radius', 0.05), # 0.05 for Puzzlebot
                ('length', 0.19), # 0.19 for Puzzlebot
                ('Kv', 0.3),
                ('Kw', 1.0),
            ]
        )

        # Msgs
        self.robot_pose = Pose2D()
        self.robot_vel = Twist()
        self.flags = Float32()
        self.flags.data = 1.0

        # Data
        self.encoderR = 0.0
        self.encoderL = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.radius = float(self.get_parameter('radius').get_parameter_value().double_value)
        self.length = float(self.get_parameter('length').get_parameter_value().double_value)
        self.theta = 0.0
        self.velocities = np.array([[0.0], [0.0], [0.0]])
        self.m_flag = False

        # Times
        self.current_time = 0
        self.previous_time = 0

        # Desired Position
        self.xG = 0.0
        self.yG = 0.0

        self.timer_period = 0.1

        # Gains for the controller
        self.Kv = float(self.get_parameter('Kv').get_parameter_value().double_value)
        self.Kw = float(self.get_parameter('Kw').get_parameter_value().double_value)

        # Tolerance
        self.tolerance = 0.1


        # Position
        self.publish_pose = self.create_publisher(Pose2D, 'pose', 10)
        self.publish_velocity = self.create_publisher(Twist, 'ctrl_vel', 10)
        self.publish_ready = self.create_publisher(Float32, 'ready', 10)

        # Subscribers
        self.read_encoderR = self.create_subscription(Float32, 'VelocityEncR', self.encoderR_callback, qos_profile_sensor_data)
        self.read_encoderL = self.create_subscription(Float32, 'VelocityEncL', self.encoderL_callback, qos_profile_sensor_data)
        self.read_goal = self.create_subscription(Pose2D, 'goal', self.goal_callback, 10)
        self.restablish = self.create_subscription(Float32, 'restart', self.restart_callback, 10)
        self.controller_timer = self.create_timer(self.timer_period, self.controller_callback)

        self.get_logger().info('Odometry Node has been started!!')


    def encoderR_callback(self, msg):
        self.encoderR = msg.data

    def encoderL_callback(self, msg):
        self.encoderL = msg.data

    def goal_callback(self, msg):
        self.xG = msg.x
        self.yG = msg.y
        self.m_flag = True

    def restart_callback(self,msg):
        self.robot_pose.x = 0.0
        self.robot_pose.y = 0.0
        self.robot_pose.theta = 0.0
        self.get_logger().info('I restarted :D')

    def controller_callback(self):
        self.Kv = float(self.get_parameter('Kv').get_parameter_value().double_value)
        self.Kw = float(self.get_parameter('Kw').get_parameter_value().double_value)

        self.current_time = self.get_clock().now().nanoseconds
        self.dt = float((self.current_time - self.previous_time) / (10.0**9)) # in seconds
        self.previous_time = self.current_time

        self.jacobian = np.array([[(self.radius/2) * np.cos(self.robot_pose.theta), (self.radius/2) * np.cos(self.robot_pose.theta)], 
                                  [(self.radius/2) * np.sin(self.robot_pose.theta), (self.radius/2) * np.sin(self.robot_pose.theta)],
                                  [self.radius/self.length, -self.radius/self.length]])
        
        self.encoders = np.array([[self.encoderR], [self.encoderL]])

        self.velocities = self.jacobian @ self.encoders

        self.robot_pose.x += float(self.velocities[0]) * self.dt
        self.robot_pose.y += float(self.velocities[1]) * self.dt
        self.robot_pose.theta += float(self.velocities[2]) * self.dt        
        
        ed = np.sqrt((self.xG-self.robot_pose.x)**2+(self.yG-self.robot_pose.y)**2) 
        thetaG = np.arctan2((self.yG-self.robot_pose.y),(self.xG-self.robot_pose.x)) 
        e_theta = thetaG-self.robot_pose.theta 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) # Limit the angle to [-pi,pi] 

        if self.m_flag:
            if abs(e_theta) > 2.0*self.tolerance: 
                self.robot_vel.angular.z = self.Kw * e_theta
                self.robot_vel.linear.x = 0.0

            else:
                if abs(ed) > self.tolerance: 
                    self.robot_vel.linear.x = self.Kv * ed
                    self.robot_vel.angular.z = self.Kw * e_theta
                    if self.robot_vel.linear.x > 0.5:
                        self.robot_vel.linear.x = 0.5
                else:
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0

                if abs(ed) < self.tolerance:
                    self.m_flag = False
                    self.publish_ready.publish(self.flags)

        else:
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = 0.0
        
        # if abs(ed) > self.tolerance:
        #     self.robot_vel.angular.z = self.Kw * e_theta
        #     self.robot_vel.linear.x = self.Kv * ed  
        
        self.get_logger().info('Pose: x = %f, y = %f, theta = %f' % (self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta))
        self.publish_pose.publish(self.robot_pose) #publish the robot's speed  
        self.publish_velocity.publish(self.robot_vel) #publish the robot's velocityencoderR



def main(args = None):
    rclpy.init(args=args)
    o_node = Odometry()
    rclpy.spin(o_node)
    o_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
