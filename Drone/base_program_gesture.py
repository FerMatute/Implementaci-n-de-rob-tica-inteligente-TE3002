from djitellopy import Tello
import cv2
import numpy as np
import mediapipe as mp

# Initializing MediaPipe Hands and drawing utilities
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Frame source: 0-> from webcam 1-> from drone
frame_source = 0

# Initializing camera stream
if frame_source == 0:
    capture = cv2.VideoCapture(0)
elif frame_source == 1:
    drone = Tello()
    drone.connect()
    drone.streamoff()
    drone.streamon()

# Image size
h = 500
w = 500


# thumb up function
def is_thumb_up(landmarks):
    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP].y
    thumb_ip = landmarks[mp_hands.HandLandmark.THUMB_IP].y
    return thumb_tip < thumb_ip


def main():
    with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:
        print("main program running now")
        # Main cycle
        while True:
            # Obtaining a new frame
            if frame_source == 0:
                ret, img = capture.read()
            elif frame_source == 1:
                frame_read = drone.get_frame_read()
                img = frame_read.frame
                # Going from RGB to BGR color workspace
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # Rotating the image
            img = cv2.flip(img, 1)

            # Resizing the image --- cv2.resize('ImageName',(x_dimension,y_dimension))
            img = cv2.resize(img, (500, 500))

            # hand detection
            results = hands.process(img)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # drwing landmarks and connections
                    mp_drawing.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                    # verify if the thumb is up
                    if is_thumb_up(hand_landmarks.landmark):
                        cv2.putText(img, "Thumb Up", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                                    cv2.LINE_AA)
                    else:
                        cv2.putText(img, "Thumb Not Up", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                                    cv2.LINE_AA)


            # Writing the battery level in the image cv2.putText(ImageName, text, location, font, scale, color, thickness)
            if frame_source == 1:
                cv2.putText(img, 'Battery:  ' + str(drone.get_battery()), (0, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0),
                            3)

            # Showing the image in a window
            cv2.imshow("Hand Gesture Recognition", img)

            # Keyboard monitor
            key = cv2.waitKey(1) & 0xFF

            # close the windows and break the program if 'q' is pressed
            if key == 113:
                cv2.destroyAllWindows()
                if frame_source == 1:
                    # drone.land()
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
