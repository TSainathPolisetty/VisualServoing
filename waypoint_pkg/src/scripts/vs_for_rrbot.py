#!/usr/bin/env python

import rospy
import cv2
import numpy as np
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class rrbot_vs_control:
    def __init__(self):
        # Initialize ROS node and subscribers/publishers
        rospy.init_node('rrbot_vs_control', anonymous=True)

        self.joint1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)

        self.rrstate_sub = rospy.Subscriber('/rrbot/joint_states', JointState, self.joint_states_callback)
        self.obj_sub = rospy.Subscriber('/object_location', Point, self.obj_callback)
        
        self.joint1_state = 0;
        self.joint2_state = 0;
        self.got_joints = 0;
                
        self.object_loc = Point();
        self.got_obj = 0;

    def joint_states_callback(self,msg):
        self.joint1_state = msg.position[0];
        self.joint2_state = msg.position[1];
        #print("Joint states sub " ,self.joint2_state, " ", self.joint2_state)
        self.got_joints = 1;
        self.vscontrol();

    def obj_callback(self,msg):
        self.object_loc = msg;
        #object_loc.x = msg.x;
        #object_loc.y = msg.y;
        #object_loc.z = msg.z;    
        print("got on=bj center " ,self.object_loc.x, " ", self.object_loc.y)
        self.got_obj = 1;
        self.vscontrol();
        

    def vscontrol(self):
        # PID
        #errx = 0 - self.joint1_state;
        #erry = 0 - self.joint2_state;                
        #rate1 = 0.1;
        #rate2 = 0.1;
        #self.joint1_pub.publish(Float64(  (self.joint1_state + rate1*errx)  ))
        #self.joint2_pub.publish(  Float64(  self.joint2_state + rate2*erry  )  )
        #print("got on=bj center " ,self.object_loc.x, " ", self.object_loc.y)
        #print("W " ,errx, " ", erry)
        #print("Joint states sub " ,self.joint1_state, " ", self.joint2_state)
        #print("Publishing joints" ,self.joint1_state - rate1*errx, " ", self.joint2_state + rate2*erry)

        # Visual Servoing
        errx = 400 - self.object_loc.x;
        erry = 400 - self.object_loc.y;                
        rate1 = 0.0001;
        rate2 = 0.001;
        self.joint1_pub.publish(Float64(  (self.joint1_state + rate1*errx)  ))
        self.joint2_pub.publish(  Float64(  self.joint2_state - rate2*erry  )  )
        print("got on=bj center " ,self.object_loc.x, " ", self.object_loc.y)
        print("W " ,errx, " ", erry)
        print("Joint states sub " ,self.joint1_state, " ", self.joint2_state)
        print("Publishing joints" ,self.joint1_state - rate1*errx, " ", self.joint2_state + rate2*erry)
        
        



if __name__ == '__main__':
    try:
        rrbot = rrbot_vs_control();
        #rrbot.vscontrol()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
