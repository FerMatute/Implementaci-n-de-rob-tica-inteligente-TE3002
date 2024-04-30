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
class CVExample(Node): 

    def __init__(self): 

        super().__init__('Semaforos') 
        self.bridge = CvBridge() 
        
        # Subcribers
        self.sub = self.create_subscription(Image, 'robot/camera1/image_raw', self.camera_callback, 10) 
        self.vel_sub = self.create_subscription(Twist, 'ctr_vel',self.getVels, 10) # <------- Cambia 'ctr_vel' por el nombre del topic que publica la velocidad
        # Publishers
        self.pub = self.create_publisher(Image, 'processed_img', 10) 
        self.cmd_vel_pub = self.create_publisher(Twist,'cmd_vel',10)
        self.image_received_flag = False #This flag is to ensure we received at least one image  
        dt = 0.1 
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('ros_color_tracker Node started') 
        
        self.kw = 0.002
        self.Kv = 0.004
        self.max_rad = 200
        self.vel = Twist()
    
    def getVels(self,msg):
        self.Xvel = msg.linear.x
        self.Wvel = msg.angular.z
        
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
            
            size= image.shape
            xcenter = size[1]/2 # Image with/2 
            
            #Once we read the image we need to change the color space to HSV 
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
            
            #Hsv limits are defined 
        
            # Rojos
            min_red = np.array([0,100,100])     # Ya detecta rojos :D
            max_red = np.array([10,255,255])
            # Verdes
            min_green = np.array([40,100,100])  # <--- Hay que tunear el verde
            max_green = np.array([100,255,255]) 
            # Amarillos
            min_yellow = np.array([20,100,100]) # Ya detecta amarillos :D
            max_yellow = np.array([40,255,255])
            
            #This is the actual color detection  
            mask_r = cv2.inRange(hsv,min_red,max_red)
            mask_v = cv2.inRange(hsv,min_green,max_green)
            mask_y = cv2.inRange(hsv,min_yellow,max_yellow)
            
            # find contours in the mask and initialize the current (x, y) center of the ball  
            cnts_r,_ = cv2.findContours(mask_r.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts_v,_ = cv2.findContours(mask_v.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts_y,_ = cv2.findContours(mask_y.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            center = None  
            try:
                if len(cnts_r) > 0: #If the light is red  
                    # find the largest contour in the mask, then use  
                    # it to compute the minimum enclosing circle and  
                    # centroid  
                    print("Red")
                    c = max(cnts_r, key=cv2.contourArea)  
                    ((x, y), radius) = cv2.minEnclosingCircle(c)  
                    M = cv2.moments(c)  
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  
                    # only proceed if the radius meets a minimum size  
                    if radius > 10: #10 pixels 
                        # Draw the circle and centroid on the cv_img. 
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                        cv2.circle(image, center, 5, (0, 0, 255), -1)  
                    else: #If the detected object is too small 
                        radius = 0.0  #Just set the radius of the object to zero
                    # Si el objeto es rojo, el robot se detendrá
                    self.vel.linear.x=0.0
                    self.vel.angular.z=0.0
                    self.cmd_vel_pub.publish(self.vel)
                    
                        
                elif len(cnts_v) > 0: #If the light is green
                    # find the largest contour in the mask, then use  
                    # it to compute the minimum enclosing circle and  
                    # centroid  
                    print("Green")
                    c = max(cnts_v, key=cv2.contourArea)  
                    ((x, y), radius) = cv2.minEnclosingCircle(c)  
                    M = cv2.moments(c)  
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  
                    # only proceed if the radius meets a minimum size  
                    if radius > 5:
                        # Draw the circle and centroid on the cv_img. 
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                        cv2.circle(image, center, 5, (0, 0, 255), -1)  
                    else: #If the detected object is too small 
                        radius = 0.0  #Just set the radius of the object to zero
                    # Si el objeto es verde, el robot seguirá la trayectoria
                    self.vel.linear.x=self.Xvel
                    self.vel.angular.z=self.Wvel
                    self.cmd_vel_pub.publish(self.vel)
                        
                elif len(cnts_y) > 0: #If the light is yellow
                    # find the largest contour in the mask, then use  
                    # it to compute the minimum enclosing circle and  
                    # centroid  
                    print("Yellow")
                    c = max(cnts_y, key=cv2.contourArea)  
                    ((x, y), radius) = cv2.minEnclosingCircle(c)  
                    M = cv2.moments(c)  
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  
                    # only proceed if the radius meets a minimum size  
                    if radius > 5:
                        # Draw the circle and centroid on the cv_img. 
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                        cv2.circle(image, center, 5, (0, 0, 255), -1)  
                    else: #If the detected object is too small 
                        radius = 0.0  #Just set the radius of the object to zero
                    
                    # Si el objeto es amarillo, el robot se moverá hacia el objeto 
                    # y se detendrá a una distancia de 200 pixeles  
                    d =  xcenter - x
                    r = self.max_rad - radius 
                    w= self.kw*d
                    v= self.Kv*r
                    self.vel.linear.x=v
                    self.vel.angular.z=w
                    self.cmd_vel_pub.publish(self.vel)
                        
                else:  
                    # All the values will be zero if there is no object  
                    x = 0.0 
                    y = 0.0 
                    radius=0.0       
                # print("x: ", x) 
                # print("y: ", y) 
                # print("radius: ", radius) 
            
                # Publish the radius and center of the detected object
                
                self.pub.publish(self.bridge.cv2_to_imgmsg(image,'bgr8')) 
            except:
                print("Error in the detection")
        
def main(args=None): 
    rclpy.init(args=args) 
    cv_e = CVExample() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 

    main() 