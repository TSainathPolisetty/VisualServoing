#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import Point


class ObjectDetector:
    def __init__(self):
        # Initialize ROS node and subscribers/publishers
        rospy.init_node('object_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.image_callback)
        self.object_pub = rospy.Publisher('/object_location', Point, queue_size=10)
        self.trackerr_pub = rospy.Publisher('/Tracking_error', Point, queue_size=10)
        self.image_pub = rospy.Publisher('/object_detector/image_with_box', Image, queue_size=10)
        
        # Define color range for object detection (in HSV color space)
        self.color_lower = np.array([0, 0, 100])
        self.color_upper = np.array([10, 255, 255])
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
    def image_callback(self, msg):
        print("got image")
        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Threshold image to isolate object based on color
        mask = cv2.inRange(hsv_image, self.color_lower, self.color_upper)
        red_mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
        red_mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
                
        # Find contours in the mask
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find largest contour and its centroid
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            x, y, w, h = cv2.boundingRect(largest_contour);
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = int(x + w/2)
            center_y = int(y + h/2)
            cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
            track_err = Point()
            track_err.x = 400 - center_x
            track_err.y = 400 - center_y
            track_err.z = 0
            self.trackerr_pub.publish(track_err)

            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # Publish object location as a ROS Point message
                object_loc = Point()
                object_loc.x = cx
                object_loc.y = cy
                object_loc.z = 0
                cv2.circle(image, (cx, cy), 5, (225, 0, 0), -1)
                self.object_pub.publish(object_loc)
                print(object_loc)
        else:
            print("No contours")
        try:
            # Convert the annotated image back to ROS format and publish it
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
            print("Image published with box")
        except CvBridgeError as e:
            rospy.logerr(e)
            
            
if __name__ == '__main__':
    try:
        ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
