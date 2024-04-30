from djitellopy import Tello
import cv2

# Frame source: 0-> from webcam 1-> from drone
frame_source = 0

# Initializing camera stream
if frame_source == 0: # Webcam
	capture = cv2.VideoCapture(0)
elif frame_source == 1: # Drone
	drone = Tello()
	drone.connect()
	drone.streamoff()
	drone.streamon()


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
