""" 
    This program publishes the radius and center of the detected ball   

    The radius will be zero if there is no detected object  

    published topics:  

        /processed_img [Image] 

    subscribed topics:  

        /robot/camera1/image_raw    [Image]  

"""  
import rclpy 
from rclpy.node import Node 
import cv2 # El open CV
import numpy as np 
from cv_bridge import CvBridge # Para pasar imagenes de ros2 a open CV
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
class ColorId(Node): 

    def __init__(self): 

        super().__init__('color_identification') 

        self.bridge = CvBridge() 
        
        # Subcribers
        self.sub = self.create_subscription(Image, 'robot/camera1/image_raw', self.camera_callback, 10) 
        self.vel_sub = self.create_subscription(Twist, 'ctrl_vel',self.getVels, 10) 

        # Publishers
        self.pub = self.create_publisher(Image, 'processed_img', 10) 
        self.cmd_vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info('Color Identificarion Node started!!!') 
        
        self.l_vel = 0.0
        self.w_vel = 0.0

        self.color_r = 0
        self.color_y = 0
        self.color_v = 0

        self.robot_vel = Twist()
    
    def getVels(self,msg):
        self.l_vel = msg.linear.x
        self.w_vel = msg.angular.z
        
    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            self.image_received_flag = True  
        except: 
            self.get_logger().info('Failed to get an image') 

    def timer_callback(self): 
        
        if self.image_received_flag: 
            
            # Create a copy of the image 
            image=self.cv_img.copy()

            image = cv2.flip(image, 0)
            
            #Once we read the image we need to change the color space to HSV 
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
            
            #Hsv limits are defined 
        
            # Rojos
            min_red = np.array([0,20,20])     # Ya detecta rojos :D
            max_red = np.array([10,255,255])
            # Verdes

            min_green = np.array([40,50,50])  # <--- Hay que tunear el verde
            max_green = np.array([80,255,255]) 
            # # Amarillos
            min_yellow = np.array([15, 20, 20]) # Ya detecta amarillos :D
            max_yellow = np.array([30, 255, 255])
            
            self.get_logger().info(f"Yellow: {min_yellow} - {max_yellow}")

            #This is the actual color detection  
            mask_r = cv2.inRange(hsv, min_red, max_red)
            mask_v = cv2.inRange(hsv, min_green, max_green)
            mask_y = cv2.inRange(hsv, min_yellow, max_yellow)
            
            # find contours in the mask and initialize the current (x, y) center of the ball  
            [cnts_r, _] = cv2.findContours(mask_r.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            [cnts_v, _] = cv2.findContours(mask_v.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            [cnts_y,  _] = cv2.findContours(mask_y.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            center = None  

            try:
                if len(cnts_r) > 0: #If the light is red  
                    # find the largest contour in the mask, then use  
                    # it to compute the minimum enclosing circle and  
                    # centroid  
                    c_r = max(cnts_r, key = cv2.contourArea)  
                    ((x, y), radius) = cv2.minEnclosingCircle(c_r)
                    if radius > 10: #10 pixels   
                        M_r = cv2.moments(c_r)  
                        center = (int(M_r["m10"] / M_r["m00"]), int(M_r["m01"] / M_r["m00"]))
                        # only proceed if the radius meets a minimum size  
                    
                        # Draw the circle and centroid on the cv_img. 
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                        cv2.circle(image, center, 5, (0, 0, 255), -1)  

                        self.color_r = 1

                    else: #If the detected object is too small 
                        radius = 0.0  #Just set the radius of the object to zero
                        self.color_r = 0
                        # Si el objeto es rojo, el robot se detendrá
                    
                if len(cnts_y) > 0: #If the light is yellow
                    # find the largest contour in the mask, then use  
                    # it to compute the minimum enclosing circle and  
                    # centroid  
                    c_y = max(cnts_y, key=cv2.contourArea)  
                    ((x, y), radius) = cv2.minEnclosingCircle(c_y)  
                    if radius > 10 and self.color_r != 1:
                        M_y = cv2.moments(c_y)  
                        center = (int(M_y["m10"] / M_y["m00"]), int(M_y["m01"] / M_y["m00"]))  
                    
                    # only proceed if the radius meets a minimum size  
                        # Draw the circle and centroid on the cv_img. 
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                        cv2.circle(image, center, 5, (0, 0, 255), -1)  
                        self.color_y = 1

                    else: #If the detected object is too small 
                        self.color_y = 0
                        radius = 0.0  #Just set the radius of the object to zero
                        

                if len(cnts_v) > 0: #If the light is green
                    # find the largest contour in the mask, then use  
                    # it to compute the minimum enclosing circle and  
                    # centroid  
                    c = max(cnts_v, key=cv2.contourArea)  
                    ((x, y), radius) = cv2.minEnclosingCircle(c)  
                    if radius > 10 and self.color_r != 1 and self.color_y != 1:
                        
                        M = cv2.moments(c)  
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  
                        # only proceed if the radius meets a minimum size  
                            # Draw the circle and centroid on the cv_img. 
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                        cv2.circle(image, center, 5, (0, 0, 255), -1)  
                        self.color_v = 1

                    else: #If the detected object is too small 
                        radius = 0.0  #Just set the radius of the object to zero
                        self.color_v = 0
                    # Si el objeto es verde, el robot seguirá la trayectoria

                    
            except:  
                # All the values will be zero if there is no object  
                self.get_logger().info("No color detected")
                x = 0.0 
                y = 0.0 
                radius = 0.0
                self.color = 0       

            if self.color_r == 1:
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
                self.get_logger().info("Stop")
            
            elif self.color_y == 1:
                self.robot_vel.linear.x = self.l_vel * 0.5
                self.robot_vel.angular.z = self.w_vel * 0.5
                self.get_logger().info("Middle speed")

            elif self.color_v == 1:
                self.robot_vel.linear.x = self.l_vel
                self.robot_vel.angular.z = self.w_vel
                self.get_logger().info("Full speed")

            self.cmd_vel_pub.publish(self.robot_vel)
            # Publish the radius and center of the detected object
            self.pub.publish(self.bridge.cv2_to_imgmsg(image,'bgr8'))

    
def main(args=None): 
    rclpy.init(args=args) 
    cl_id = ColorId() 
    rclpy.spin(cl_id) 
    cl_id.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 

    main() 