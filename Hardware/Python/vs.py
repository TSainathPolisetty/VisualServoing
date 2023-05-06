# import cv2
# import numpy as np
# import serial

# def main():
#     # Establish a serial connection with the Arduino
#     arduino = serial.Serial('/dev/ttyACM0', 9600)

#     # Load the YOLOv4-tiny model
#     net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4.cfg")
#     layer_names = net.getLayerNames()
#     output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

#     # Load the COCO dataset class names
#     with open("coco.names", "r") as f:
#         classes = [line.strip() for line in f.readlines()]

#     # Initialize the camera
#     cap = cv2.VideoCapture(0)

#     while True:
#         # Read a frame from the camera
#         ret, frame = cap.read()

#         # Detect objects using YOLOv4-tiny
#         blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), (0, 0, 0), True, crop=False)
#         net.setInput(blob)
#         layer_outputs = net.forward(output_layers)

#         # Process the detected objects
#         for output in layer_outputs:
#             for detection in output:
#                 scores = detection[5:]
#                 class_id = np.argmax(scores)
#                 confidence = scores[class_id]

#                 if confidence > 0.5:
#                     # Get the object's center coordinates
#                     center_x = int(detection[0] * frame.shape[1])
#                     center_y = int(detection[1] * frame.shape[0])

#                     # Send the coordinates to the Arduino
#                     try:
#                         arduino.write(f"{center_x},{center_y}\n".encode())
#                     except serial.SerialException as e:
#                         print(f"Failed to write to Arduino: {e}")

#         # Display the frame
#         cv2.imshow("Object Detection", frame)

#         # Exit if the user presses the 'q' key
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     # Release the camera and close all windows
#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()




import cv2
import numpy as np
import serial

def main():
    # Establish a serial connection with the Arduino
    arduino = serial.Serial('/dev/ttyACM0', 9600)
    # Replace 'COM_PORT' with the appropriate port name for your Arduino, e.g., 'COM3' on Windows or '/dev/ttyACM0' on Linux

    # Load the YOLOv4-tiny model
    net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4.cfg")
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

    # Load the COCO dataset class names
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    # Initialize the camera
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # Detect objects using YOLOv4-tiny
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        layer_outputs = net.forward(output_layers)

        # Process the detected objects
        class_ids = []
        confidences = []
        boxes = []

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.3:
                    # Get the object's center coordinates and dimensions
                    center_x = int(detection[0] * frame.shape[1])
                    center_y = int(detection[1] * frame.shape[0])
                    w = int(detection[2] * frame.shape[1])
                    h = int(detection[3] * frame.shape[0])

                    # Calculate the bounding box coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Perform non-maximum suppression
        # Perform non-maximum suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.4)

        for i in np.array(indices).flatten():
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            color = (0, 255, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, f"{label} {int(confidence * 100)}%", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Send the coordinates to the Arduino
            try:
                arduino.write(f"{center_x},{center_y}\n".encode())
            except serial.SerialException as e:
                print(f"Failed to write to Arduino: {e}")



        # Display the frame
        cv2.imshow("Object Detection", frame)

        # Exit if the user presses the 'q
         
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
