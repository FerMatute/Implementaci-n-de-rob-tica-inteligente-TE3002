from djitellopy import Tello
import cv2

# Frame source: 0-> from webcam 1-> from drone
frame_source = 0
in_speed = 50

# Initializing camera stream
if frame_source == 0: # Webcam
	capture = cv2.VideoCapture(0)
elif frame_source == 1: # Drone
	drone = Tello()
	drone.connect()
	drone.streamoff()
	drone.streamon()
	drone.left_right_velocity = 0
	drone.foward_bakcward_velocity = 0
	drone.up_down_velocity = 0
	drone.yaw_velocity = 0
	drone.speed = 50
	drone.fly = False
	
    # Create a slider window
	cv2.namedWindow("Speedbar")
	cv2.resizeWindow("Speedbar", (100, 50))
	
    # Create the callback function
	def callback():
		if frame_source == 1:
			drone.speed = cv2.getTrackbarPos("Trackbar", "Speedbar")

    # Create a speed trackbar
	# 0 = Min val
    # 100 = Max val
	# callback = function that calls each time it changes speed
	cv2.createTrackbar("Trackbar", "Speedbar", 0, 100, callback)
	cv2.setTrackbarPos("Trackbar", "Speedbar", in_speed)

def main():
	print("main program running now")
	# Ciclo principal
	while True:
		# Obtaining a new frame
		if frame_source == 0:
			ret, img = capture.read()
		elif frame_source == 1:
			frame_read = drone.get_frame_read()
			img = frame_read.frame
			img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

		# Rotating the image
		img = cv2.flip(img, 1)

		# Resizing the image --- cv2.resize('ImageName',(x_dimension,y_dimension))
		img = cv2.resize(img, (500, 500))

		# Writing the battery level in the image.... cv2.putText(ImageName, text, location, font, scale, color, thickness)
		if frame_source == 1:
			cv2.putText(img, 'Battery:  ' + str(drone.get_battery()), (0, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 3)

		# Showing the image in a window
		cv2.imshow("Image", img)

		# Keyboard monitor
		key = cv2.waitKey(1) & 0xFF

		# close the windows and break the program if 'q' is pressed
		if key == 113:
			cv2.destroyAllWindows()
			if frame_source == 1:
				drone.land()
				drone.streamoff()
				drone.end()
			break
		# if source_frame == 1:
		# Take off if spacebar is pressed
		elif key == 32:
			drone.fly = True
			if drone.fly == True:
				drone.fly = False
				drone.takeoff()
			
        # Lands if DEL is pressed
		elif key == 127:
			drone.land()
		
        # Move foward when 'w' is pressed
		elif key == 119:
			drone.left_right_velocity = 0
			drone.foward_bakcward_velocity = drone.speed
			drone.up_down_velocity = 0
			drone.yaw_velocity = 0
			
        # Move backward when 's' is pressed
		elif key == 115:
			drone.left_right_velocity = 0
			drone.foward_bakcward_velocity = -(drone.speed)
			drone.up_down_velocity = 0
			drone.yaw_velocity = 0
			
        # Move left when 'a' is pressed
		elif key == 97:
			drone.left_right_velocity = -(drone.speed)
			drone.foward_bakcward_velocity = 0
			drone.up_down_velocity = 0
			drone.yaw_velocity = 0
			
        # Move right when 'd' is pressed
		elif key == 100:
			drone.left_right_velocity = drone.speed
			drone.foward_bakcward_velocity = 0
			drone.up_down_velocity = 0
			drone.yaw_velocity = 0
			
        # Move up when 'u' is pressed
		elif key == 117:
			drone.left_right_velocity = 0
			drone.foward_bakcward_velocity = 0
			drone.up_down_velocity = drone.speed
			drone.yaw_velocity = 0
			
        # Move down when 'i' is pressed
		elif key == 105:
			drone.left_right_velocity = 0
			drone.foward_bakcward_velocity = 0
			drone.up_down_velocity = -(drone.speed)
			drone.yaw_velocity = 0
			
        # Move yaw more when 'y' is pressed
		elif key == 121:
			drone.left_right_velocity = 0
			drone.foward_bakcward_velocity = 0
			drone.up_down_velocity = 0
			drone.yaw_velocity = drone.speed
			
        # Move yaw less when 't' is pressed
		elif key == 116:
			drone.left_right_velocity = 0
			drone.foward_bakcward_velocity = 0
			drone.up_down_velocity = 0
			drone.yaw_velocity = -(drone.speed)
		else:
            #If nothing is pressed
			drone.left_right_velocity = 0
			drone.foward_bakcward_velocity = 0
			drone.up_down_velocity = 0
			drone.yaw_velocity = -(drone.speed)
			
		drone.send_rc_control(drone.left_right_velocity, drone.forward_backward_velocity, drone.up_down_velocity, drone.yaw_velocity)


try:
	main()

except KeyboardInterrupt:
	print('KeyboardInterrupt exception is caught')
	cv2.destroyAllWindows()
	if frame_source == 1:
		if drone.fly == True:
			drone.land()
	drone.streamoff()
	drone.end()

else:
	print('No exceptions are caught')
