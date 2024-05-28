from djitellopy import Tello
import cv2
import numpy as np
import mediapipe as mp

# Initializing MediaPipe Hands and drawing utilities
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Frame source: 0-> from webcam 1-> from drone
frame_source = 1

if frame_source == 1:
    cv2.namedWindow("Trackbars")

INITIAL_SPEED = 50 # 0-100%
INITIAL_VALID_HEIGHT = 300 # 0-300 cm


KP_YAW = 0.5
KP_PROXIMITY = 0.001
KP_HEIGHT = 0.9



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
	capture = cv2.VideoCapture(0)
	

# Callback functions to update the speed
def speed_callback(val):
	global speed
	speed = cv2.getTrackbarPos("Speed", "Trackbars")

# Callback functions to update the valid height
def height_callback(val):
	global valid_height
	valid_height = cv2.getTrackbarPos("Height", "Trackbars")


# Image size
h = 500
w = 500


# thumb up function
def is_thumb_up(landmarks):
    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP].y
    thumb_ip = landmarks[mp_hands.HandLandmark.THUMB_IP].y
    if frame_source == 1:
        drone.send_rc_control(0,0,0,speed)
    return thumb_tip < thumb_ip

def is_index_finger_pointing(hand_landmarks):
    tolerance = .5
    if frame_source== 0:
        if hand_landmarks[8].x > hand_landmarks[5].x + tolerance:
            return "Right"
        elif hand_landmarks[8].x < hand_landmarks[5].x - tolerance:
            return "Left"
        elif hand_landmarks[8].y < hand_landmarks[5].y - tolerance:
            return "Up"
        elif hand_landmarks[8].y > hand_landmarks[5].y + tolerance:
            return "Down"
        else:
            return "No Movement"
    elif frame_source==1 :
        if hand_landmarks[8].x > hand_landmarks[5].x + tolerance:
            drone.send_rc_control(speed, 0, 0, 0)
            return "Right"
        elif hand_landmarks[8].x < hand_landmarks[5].x - tolerance:
            drone.send_rc_control(-speed, 0, 0, 0)
            return "Left"
        elif hand_landmarks[8].y < hand_landmarks[5].y - tolerance:
            drone.send_rc_control(0, speed, 0, 0)
            return "Up"
        elif hand_landmarks[8].y > hand_landmarks[5].y + tolerance:
            drone.send_rc_control(0, -speed, 0, 0)
            return "Down"
        drone.send_rc_control(0,0,0,0)
        return "No Movement"
        


def main():
    global frame_source
    
    if frame_source == 1:
        global speed, valid_height, drone, counter, fast_changing_input, rc_velocities, cmd_sent, delay_waited# Create trackbars for the speed and height
        
        cv2.createTrackbar("Speed", "Trackbars", 0, 100, speed_callback)
        cv2.setTrackbarPos("Speed", "Trackbars", INITIAL_SPEED)
        
        cv2.createTrackbar("Height", "Trackbars", 50, 300, height_callback)
        cv2.setTrackbarPos("Height", "Trackbars", INITIAL_VALID_HEIGHT)
    
    with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.25,
            min_tracking_confidence=0.5) as hands:
        
        print("main program running now")
        
        prev_key = chr(255)
        # Main cycle
        while True:
            # Obtaining a new frame
            if frame_source == 0:
                _, img2 = capture.read()
            elif frame_source == 1:
                frame_read = drone.get_frame_read()
                img = frame_read.frame
                # Going from RGB to BGR color workspace
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                _,img2 = capture.read()

            # Rotating the image
                img = cv2.flip(img, 1)

                # Resizing the image --- cv2.resize('ImageName',(x_dimension,y_dimension))
                img = cv2.resize(img, (500, 500))

            # hand detection
            img2 = cv2.flip(img2, 1)
            results = hands.process(img2)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # drwing landmarks and connections
                    mp_drawing.draw_landmarks(img2, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    cv2.putText(img2, "Index Finger pointing: " + is_index_finger_pointing(hand_landmarks.landmark), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                                    cv2.LINE_AA)
                cv2.putText(img, f'Battery: {drone.get_battery()}', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (185,79,249), 3)
                cv2.putText(img, f'Speed: {speed}', (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                if drone.get_battery() < 25 and drone.get_battery() >= 15:
                    cv2.putText(img, 'Low Battery', (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
                elif drone.get_battery() < 15:
                    cv2.putText(img, 'Critical Battery', (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 3)
                else:
                    cv2.putText(img, 'Normal Battery', (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (185,79,249), 3)
                    cv2.putText(img, f'Height: {drone.get_height()}', (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (185,79,249), 3)
			
            # Showing the image in a window
            cv2.imshow("Hand Gesture Recognition", img2)
            cv2.waitKey(1) 
            if frame_source == 1:
                # Keyboard monitor
                key = chr(cv2.waitKey(1) & 0xFF)
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
                cv2.imshow("Trackbars", img)
                
                # Breaking the loop if 'q' is pressed
                if key == 'q':
                    cv2.destroyAllWindows()
                    break

                prev_key = key

         


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
