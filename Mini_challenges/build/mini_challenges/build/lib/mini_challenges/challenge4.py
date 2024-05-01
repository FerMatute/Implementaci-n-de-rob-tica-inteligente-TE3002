import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 

class ColorTracker(Node): 
    def __init__(self): 
        super().__init__('color_tracker') 

        self.bridge = CvBridge() 
 
        self.sub = self.create_subscription(Image, 'image_raw', self.camera_callback, 10) 
        self.pub = self.create_publisher(Image, 'processed_img', 10) 
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.radius_max = 100.0 # To check the maximum radius of the blue object [pixels]

        self.Kw = 0.01 # Gain for the angular velocity
        self.Kv = 0.01 # Gain for the linear velocity

        self.robot_vel = Twist() # Message to publish the velocity

        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.1 
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('Color Tracker started!!!') 
 
    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            self.image_received_flag = True  
        except: 
            self.get_logger().info('Failed to get an image') 
 
    def timer_callback(self): 
        if self.image_received_flag: 
            # We will now use OpenCV to detect the color blue in the image
            # First we need to convert the image to HSV
            image = self.cv_img.copy()
            
            # JUST FOR THE PUZZLEBOT
            # Check if the image is taken from the real camera or the simulation
            image = cv2.flip(image, 0)

            # Convert the image to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Obtain the dimensions of image
            size = image.shape
            
            # Define the center of the image
            x_center = size[1]/2

            # Define the range of blue color in HSV
            min_blue = np.array([105, 20, 20])
            max_blue = np.array([130, 255, 255])

            # Create a mask for the blue color
            b_mask = cv2.inRange(hsv, min_blue, max_blue)
            
            # Apply the mask to the image
            b_res = cv2.bitwise_and(image, image, mask = b_mask)

            [contours, heriarchy] = cv2.findContours(b_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            center = None
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 10:
                    cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(image, center, 5, (0, 0, 255), -1)
                    
                    # Calculate the error
                    x_e = x_center - x
                    r_e = self.radius_max - radius

                    # We will send the velocity commands to the robot based on the error
                    self.robot_vel.linear.x = self.Kv * r_e
                    self.robot_vel.angular.z = self.Kw * x_e

                else:
                    radius = 0.0
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
            else:
                x = 0.0
                y = 0.0
                radius = 0.0
                x_e = 0.0
                r_e = 0.0
                self.robot_vel.linear.x = self.Kv * r_e
                self.robot_vel.angular.z = self.Kw * x_e
                
            
            self.get_logger().info(f"x: {x}, y: {y}, Radius: {radius}")
            self.get_logger().info(f"x_error: {x_e}, r_e: {r_e}")
            # Publish the image with the blue color
            self.pub.publish(self.bridge.cv2_to_imgmsg(image,'bgr8')) 
            self.vel_pub.publish(self.robot_vel)
     
def main(args=None): 
   rclpy.init(args=args) 
   clr_t = ColorTracker() 
   rclpy.spin(clr_t) 
   clr_t.destroy_node() 
   rclpy.shutdown() 
 
if __name__ == '__main__': 
   main() 