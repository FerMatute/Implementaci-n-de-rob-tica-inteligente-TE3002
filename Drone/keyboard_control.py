from djitellopy import Tello
import cv2

INITIAL_SPEED = 70 # 0-100%
INITIAL_VALID_HEIGHT = 150 # 0-300 cm

# Frame source: 0-> from webcam 1-> from drone
frame_source = 1

# Initializing camera stream
if frame_source == 0:
	capture = cv2.VideoCapture(0)
elif frame_source == 1:
	# Drone variables
	rc_velocities = [0, 0, 0, 0]
	valid_height = INITIAL_VALID_HEIGHT
	speed = INITIAL_SPEED
	counter = 1000
	going_down = False
	fast_changing_input = False
	cmd_sent = False
	delay_waited = False
	# Connecting to the drone
	drone = Tello()
	drone.connect()
	drone.streamoff()
	drone.streamon()

# Callback functions to update the speed
def speed_callback(val):
	global speed
	speed = cv2.getTrackbarPos("Speed", "Trackbars")

# Callback functions to update the valid height
def height_callback(val):
	global valid_height
	valid_height = cv2.getTrackbarPos("Height", "Trackbars")

def main():
	global frame_source
	print("main program running now")
	# Create window for the sliders if drone is the frame source
	if frame_source == 1:
		global speed, valid_height, drone, counter, fast_changing_input, rc_velocities, cmd_sent, delay_waited
		# Create window for sliders
		cv2.namedWindow("Trackbars")
		cv2.resizeWindow("Trackbars", (500, 90))

		# Create trackbars for the speed and height
		cv2.createTrackbar("Speed", "Trackbars", 0, 100, speed_callback)
		cv2.setTrackbarPos("Speed", "Trackbars", speed)
		cv2.createTrackbar("Height", "Trackbars", 50, 300, height_callback)
		cv2.setTrackbarPos("Height", "Trackbars", valid_height)

	# Main Loop
	prev_key = chr(255)
	while True:
		# Obtaining a new frame
		if frame_source == 0:
			_, img = capture.read()
		elif frame_source == 1:
			frame_read = drone.get_frame_read()
			img = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)

		# Resizing the chosen image to 500x500
		img = cv2.resize(img, (500, 500))

		# Keyboard monitor with debouncer for fast response
		key = chr(cv2.waitKey(1) & 0xFF)
		if ord(key) != 255:
			cmd_sent = True
			counter = 0
		else:
			counter += 1
		
		if cmd_sent:
			if counter > 10 and counter < 40:
				delay_waited = True
			elif counter > 40:
				cmd_sent = False
				delay_waited = False
			if delay_waited:
				if ord(key) != 255:
					fast_changing_input = True

			key = prev_key if ord(key) == 255 else key
		
		if fast_changing_input:
			if counter > 5:
				fast_changing_input = False
				cmd_sent = False
				delay_waited = False

		# Things to do if the frame source is the drone
		if frame_source == 1:
			# Displaying the battery, speed, height and the battery status on the image
			cv2.putText(img, f'Battery: {drone.get_battery()}', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (185,79,249), 3)

			cv2.putText(img, f'Speed: {speed}', (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

			if drone.get_battery() < 25 and drone.get_battery() >= 15:
				cv2.putText(img, 'Low Battery', (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
			elif drone.get_battery() < 15:
				cv2.putText(img, 'Critical Battery', (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 3)
			else:
				cv2.putText(img, 'Normal Battery', (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (185,79,249), 3)

			cv2.putText(img, f'Height: {drone.get_height()}', (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (185,79,249), 3)

			# Ends the drone object when q is pressed or battery is lower than 12%
			if key == 'q' :
				drone.send_rc_control(0,0,0,0)
				drone.land()
				drone.streamoff()
				drone.end()
			# If the drone is not flying and t is pressed, the drone will take off
			elif key == 't' and not drone.is_flying:
				if drone.get_battery() < 25:
					cv2.putText(img, 'Battery LOW, cannot takeoff', (0, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 3)
				else:
					drone.send_rc_control(0,0,0,0)
					drone.takeoff()
			# If the drone is flying and l is pressed, the drone will land
			elif key == 'l' and drone.is_flying:
				drone.send_rc_control(0,0,0,0)
				drone.land()
			# If the drone is flying and the keys w,a,s,d,z,x,.,, are pressed, the drone will move
			elif key in 'wasdzx,.' and drone.is_flying:
				# Dictionary to decode the key pressed into the drone's velocity
				decoder_f_b = {'w': speed,
								's': -speed}
				decoder_l_r = {'a': -speed,
								'd': speed}
				decoder_u_d = {'z': speed, 
								'x': -speed}
				decoder_yaw = {',': -speed, 
								'.': speed}
				# Assigning the velocities to movement directions
				forward_backward_velocity = decoder_f_b[key] if key in decoder_f_b else 0
				left_right_velocity = decoder_l_r[key] if key in decoder_l_r else 0
				up_down_velocity = decoder_u_d[key] if key in decoder_u_d else 0
				yaw_velocity = decoder_yaw[key] if key in decoder_yaw else 0
				rc_velocities = [left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity]

			if drone.get_height() > valid_height:
				rc_velocities[2] = -speed

			# Sending corresponding drone cmmand once to avoid network bottleneck
			if key != prev_key:
				if ord(key) == 255:
					rc_velocities = [0, 0, 0, 0]
				drone.send_rc_control(*rc_velocities)

		# Showing the image in a window
		cv2.imshow("Image", img)
		
		# Breaking the loop if 'q' is pressed
		if key == 'q':
			cv2.destroyAllWindows()
			break

		prev_key = key

# Try-except block to catch KeyboardInterrupt exception and safely land the drone if the frame source is the drone
try:
	main()
except KeyboardInterrupt:
	print('KeyboardInterrupt exception is caught')
	cv2.destroyAllWindows()
	if frame_source == 1:
		drone.land()
		drone.streamoff()
		drone.end()
else:
	print('No exceptions are caught')
