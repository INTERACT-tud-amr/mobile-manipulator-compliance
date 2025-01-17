import rospy
import time
import pickle
import sys
import os
import numpy as np
from user_interface_msg.msg import Record, Ufdbk, Record
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Pose
import copy

class TrackerCompliant():
    def __init__(self, robot_name):
        # -- parameters --#
        self.robot_name = robot_name
        self.Kd = np.eye(3) #* 40
        self.Kd[0, 0] = 40
        self.Kd[1, 1] = 40
        self.Kd[2, 2] = 10
        self.Dd = np.eye(3) * 3
        self.Ko = np.eye(3) * 1.
        self.desired_pose_offset = [0., 0., 0.1]
        # self.Kd = np.eye(3) * 0.000000001
        # self.Dd = np.eye(3) * 0.000000001
        self.end = False
        self.TARGET_RESET = False
        self.target = None
        self.stiffness_enabled = False
        self.mode = "Unknown"
        
        # -- subscribers -- 
        rospy.Subscriber('%s/bluetooth_teleop/joy' % robot_name, Joy, self._callback_joystick, queue_size=10)
        rospy.Subscriber('%s/compliant/feedback' % robot_name, Ufdbk, self._callback_feedback_mode, queue_size=10)
        rospy.Subscriber('%s/compliant/record' % robot_name, Record, self._callback_record, queue_size=10)

        # -- publishers --
        self.pub_mode = rospy.Publisher("%s/compliant/make_compliant" % robot_name, Bool, queue_size=1)
        self.pub_desired_pose = rospy.Publisher("%s/compliant/desired_pose" % robot_name, Pose, queue_size=1)
        self.pub_stiffness = rospy.Publisher("%s/compliant/set_stiffness" % robot_name, Float32MultiArray, queue_size=1)
        
        
    def _callback_feedback_mode(self, data):
        self.mode = data.mode

    def _callback_joystick(self, data):
        if data.buttons[0] and self.end == False: #cross button
            self.end = True 
            print("stop compliant controller")
            
    def _callback_record(self, msg):
        self.target = msg.relative_target
            
    def publish_stiffness(self):
        stiffness_msg = Float32MultiArray()
        diag_stiffness_list = [self.Kd[i, i] for i in range(len(self.Kd))] + [self.Dd[i, i] for i in range(len(self.Dd))] + [self.Ko[i, i] for i in range(len(self.Ko))]
        print("diag_stiffness_list: ", diag_stiffness_list)
        stiffness_msg.data = diag_stiffness_list
        self.pub_stiffness.publish(stiffness_msg)
        
    def publish_desired_pose(self):
        desired_pose = [self.target[i] + self.desired_pose_offset[i] for i in range(3)]
        desired_pose_msg = Pose()
        desired_pose_msg.position.x = desired_pose[0]
        desired_pose_msg.position.y = desired_pose[1]
        desired_pose_msg.position.z = desired_pose[2]
        self.pub_desired_pose.publish(desired_pose_msg)
        
    def load_recording(self, file_name_recording):
        file_name = file_name_recording + ".pk"
        folder_path = "demonstrations"
        pickle_file_path = os.path.join(folder_path, file_name)
        with open(pickle_file_path, 'rb') as file:
            self.data_recording = pickle.load(file)
            self.q_d_list = self.data_recording["q"]
            self.pose_base = self.data_recording["base_pose"]
            
    def run(self):
        # publish pose goal
        # -- publish stiffness --
        if self.stiffness_enabled == False:
            self.publish_stiffness()
            self.stiffness_enabled = True
        
        # check if compliant mode is activated, otherwise active:
        if self.mode != "LLC_task":
            print("Making the robot compliant, do not press any keys!!")
            mode = Bool()
            mode.data = True
            self.pub_mode.publish(mode)
            time.sleep(15)
            print("You can press keys now!!!")
            
        if self.mode == "LLC_task" and self.TARGET_RESET == False and self.target != None:
            self.publish_desired_pose()
            self.TARGET_RESET = True
        
        # if "X" button pressed on joystick, the compliant mode is deactivated:
        if self.end:
            mode = Bool()
            mode.data = False
            self.pub_mode.publish(mode)
            #exit
            time.sleep(2)
            exit()
        
if __name__ == '__main__':
    rospy.init_node('tracker_compliant')
    state_recorder = TrackerCompliant(sys.argv[1])
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            state_recorder.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    
    