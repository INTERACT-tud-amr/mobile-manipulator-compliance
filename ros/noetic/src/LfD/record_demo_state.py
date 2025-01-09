import rospy
import time
import pickle
import sys
import os
from user_interface_msg.msg import Record, Ufdbk
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

"""
A script to record the state (joint state + end-effector pose) of the robot.
Use the buttons on the dinova controller to start and stop a recording:
- triangle button: start recording
- cross button: stop recording
"""

class StateRecorder:
    def __init__(self, robot_name, save_id):
        self.save_id = save_id
        self.start = False
        self.end = False
        self.mode = "Unknown"
        self.q, self.q_dot, self.time_prev = None, None, None
        self.x_pos, self.x_orient = None, None
        self.q_history, self.q_dot_history, self.time_history= [], [], []
        self.x_pos_history, self.x_quat_history, self.base_pos_history, self.base_quat_history = [], [], [], []
        self.relative_target_history, self.absolute_target_history = [], []
        self.joystick_data = None
        
        #Create ROS subscriber
        rospy.Subscriber('%s/compliant/record' % robot_name, Record, self._callback_state, queue_size=10)
        rospy.Subscriber('%s/compliant/feedback' % robot_name, Ufdbk, self._callback_feedback_mode, queue_size=10)
        rospy.Subscriber('%s/bluetooth_teleop/joy' % robot_name, Joy, self._callback_joystick, queue_size=10)
        self.pub_mode = rospy.Publisher("%s/compliant/make_compliant" % robot_name, Bool, queue_size=1)
        
    def _callback_state(self, data):
        self.q = data.pos_q
        self.q_dot = data.vel_q
        self.x_pos = data.pos_x
        self.x_quat = data.quat_x
        self.base_pos = data.pos_b
        self.base_quat = data.quat_b
        self.time = data.time
        self.relative_target = data.relative_target
        self.absolute_target = data.absolute_target
        
    def _callback_feedback_mode(self, data):
        self.mode = data.mode
    
    def _callback_joystick(self, data):
        if data.buttons[3] and self.start == False: #triangle button
            self.start = True
            print("recording started")
        if data.buttons[0] and self.end == False: #cross button
            self.end = True 
            print("recording ended")
    
    def _append_state(self):
        self.q_history.append(self.q)
        self.q_dot_history.append(self.q_dot)
        self.x_pos_history.append(self.x_pos)
        self.x_quat_history.append(self.x_quat)
        self.base_pos_history.append(self.base_pos)
        self.base_quat_history.append(self.base_quat)
        self.time_history.append(self.time)
        self.relative_target_history.append(self.relative_target)
        self.absolute_target_history.append(self.absolute_target)
        
    def _save_trajectory(self):
        #Create dictionary
        trajectory = {"q": self.q_history,
                      "q_dot": self.q_dot_history,
                      "x_pos": self.x_pos_history,
                      "x_quat": self.x_quat_history,
                      "base_pos": self.base_pos_history,
                      "base_quat": self.base_quat_history,
                      "time": self.time_history,
                      "relative_target": self.relative_target_history,
                      "absolute_target": self.absolute_target_history}
        #Save dictionary
        file_name = "recording_demonstration_" + self.save_id + ".pk"
        folder_path = "demonstrations"
        file_path = os.path.join(folder_path, file_name)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Folder '{folder_path}' created.")
        with open(file_path, 'wb') as file:
            pickle.dump(trajectory, file)
            
    def run(self):
        # check if compliant mode is activated, otherwise active:
        if self.mode != "LLC_task":
            print("Making the robot compliant, do not press any keys!!")
            mode = Bool()
            mode.data = True
            self.pub_mode.publish(mode)
            time.sleep(15)
            print("You can press keys now!!!")
        
        if self.start:
            #Append state to trajectory
            self._append_state()
            
        if self.end:
            mode = Bool()
            mode.data = False
            self.pub_mode.publish(mode)
            #Save trajectory and exit
            self._save_trajectory()
            exit()
        
    
if __name__ == '__main__':
    rospy.init_node('state_recorder')
    state_recorder = StateRecorder(sys.argv[1], save_id=sys.argv[2])
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            state_recorder.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass


    