

import cv2
import numpy as np
import serial


def main():

    # Establish a serial connection with the Arduino
    arduino = serial.Serial('/dev/ttyACM0', 9600)

    # Create a window to display the camera feed
    cv2.namedWindow("Blob Detection")

    # Capture video from the default camera
    cap = cv2.VideoCapture(0)

    # Set up SimpleBlobDetector parameters
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Area
    params.filterByArea = True
    params.minArea = 150

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.7

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a SimpleBlobDetector object
    detector = cv2.SimpleBlobDetector_create(params)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect blobs
        keypoints = detector.detect(gray_frame)

         # Print the coordinates of detected blobs
        for keypoint in keypoints:
            x, y = keypoint.pt
            arduino.write(f"{x},{y}\n".encode())
            print(f"Detected blob at coordinates (x, y): ({x}, {y})")

        # Draw detected blobs as red circles
        img_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
                                               cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show the frame with detected blobs
        cv2.imshow("Blob Detection", img_with_keypoints)

        # Exit if the user presses the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


