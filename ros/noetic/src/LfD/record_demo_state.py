import rospy
import time
import pickle
import sys
from pynput.keyboard import Listener, KeyCode
from user_interface_msg.msg import Record

"""
A script to record the state (joint state + end-effector pose) of the robot.
This script has to be executed from a laptop, as it requires a keyboard press.
"""

class StateRecorder:
    def __init__(self, robot_name, save_id):
        self.save_id = save_id
        self.start = False
        self.end = False
        self.q, self.q_dot, self.time_prev = None, None, None
        self.x_pos, self.x_orient = None, None
        self.q_history, self.q_dot_history, self.delta_t_history = [], [], []
        self.x_pos_history, self.x_orient_history = [], []
        
        #Start keyboard listener
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
        
        #Create ROS subscriber
        # rospy.Subscriber('/joint_states', JointState, self._callback_joint_states, queue_size=10)
        rospy.Subscriber('/record', Record, self._callback_state, queue_size=10)
        
    def _callback_joint_states(self, data):
        self.q = data.pos_q
        self.x_pos = data.pos_x
        # self.q_dot = data.velocity
        
    # def _callback_ee_pose(self, data):
    #     self.x_pos = data.pos_x
    #     self.x_orient = data.pose.orientation
        
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('s'):
            self.start = True
            print('Recording started.')
        elif key == KeyCode.from_char('e'):
            self.end = True
            print('Recording ended.')

    def _get_delta_t(self):
        # Initialize previous time
        if self.time_prev is None:
            self.time_prev = time.perf_counter()

        # Get delta t
        delta_t = time.perf_counter() - self.time_prev

        # Update previous time
        self.time_prev = time.perf_counter()

        return delta_t
    
    def _append_state(self):
        self.q_history.append(self.q)
        self.q_dot_history.append(self.q_dot)
        self.x_pos_history.append(self.x_pos)
        self.x_orient_history.append(self.x_orient)
        self.delta_t_history.append(self._get_delta_t())
        
    def _save_trajectory(self):
        #Create dictionary
        trajectory = {"pos_q": self.q_history,
                      "q_dot": self.q_dot_history,
                      "delta_t": self.delta_t_history}
        #Save dictionary
        filename = "joint_state_" + self.save_id + ".pk"
        with open(filename, 'wb') as file:
            pickle.dump(trajectory, file)
            
    def run(self):
        if self.start:
            #Append state to trajectory
            self._append_state()
            
        if self.end:
            #Save trajectory and exit
            self.save_trajectory()
            exit()
        
    
if __name__ == '__main__':
    rospy.init_node('state_recorder')
    state_recorder = StateRecorder(ROBOT, save_id=sys.argv[2])
    rate = rospy.Rate(100)
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            state_recorder.run()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass


    