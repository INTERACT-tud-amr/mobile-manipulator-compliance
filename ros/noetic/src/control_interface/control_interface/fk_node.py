import rospkg
import rospy
from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
import numpy as np
from sensor_msgs.msg import JointState
from typing import Union, Dict, List
import copy
from geometry_msgs.msg import PoseStamped
from compliant_control.control.state import rotMatrix_to_quaternion
from std_msgs.msg import Float64

class FKNode():
    def __init__(self):
        robot_name = "dingo2"
        rospy.init_node("fk_node")
        self._rate = rospy.Rate(100)
        
        # ---- variables from yaml file ---- #
        self.robot_name = robot_name
        self.lidar = rospy.get_param('lidar', False)
        self._q = [0]*9
        self._z_compensation = 0.0
        
        # ---------------------------------------- #
        rospack = rospkg.RosPack()
        if self.lidar:
            agent_name = "dinova_lidar"
        else:
            agent_name = "dinova"
        self.symbolic_fk()
        
        # --- subscriber ---#
        rospy.Subscriber("dinova/omni_states_vicon", JointState, self._joint_states_cb, queue_size=10)
        rospy.Subscriber("dinova/z_compensation", Float64, self._z_compensation_cb, queue_size=1)
        
        # --- publisher ---#
        self.pub_current_pose = rospy.Publisher("compliant/fk/current_pose", PoseStamped, queue_size=1)
        
    def _joint_states_cb(self, msg):
        self._q = np.array(msg.position)[0:9]
        
    def _z_compensation_cb(self, msg):
        self._z_compensation = msg.data
        
    def publish_pose(self, ee_position, ee_orientation=None):
        list_position = list(ee_position)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = list_position[0]
        pose.pose.position.y = list_position[1]
        pose.pose.position.z = list_position[2]
        if ee_orientation is not None:  
            list_orientation = list(ee_orientation)
            pose.pose.orientation.x = list_orientation[0]
            pose.pose.orientation.y = list_orientation[1]
            pose.pose.orientation.z = list_orientation[2]
            pose.pose.orientation.w = list_orientation[3]
        self.pub_current_pose.publish(pose)
        
    def symbolic_fk(self) -> GenericURDFFk:
        urdf = rospy.get_param("dinova_fk_description")
        self.forward_kinematics = GenericURDFFk(
            urdf,
            root_link="base_link",
            end_links=["arm_tool_frame"],
        )
        
    def fk_numerical(self):
        ee_T = self.forward_kinematics.numpy(q=self._q,
                                            parent_link = "base_link",
                                            child_link = "arm_tool_frame",
                                            position_only=False)
        ee_position = ee_T[:3, 3]
        ee_position[2] = ee_position[2] + self._z_compensation
        ee_quaternion = rotMatrix_to_quaternion(ee_T[:3, :3])
        return ee_position, ee_quaternion
    
    def run(self):
        while not rospy.is_shutdown():
            """
            if 'mug1' in self._object_poses:
                self._logger.log(str(self._object_poses['mug1']))
            """
            ee_position, ee_orientation = self.fk_numerical()
            self.publish_pose(ee_position, ee_orientation)
            self._rate.sleep()
    
if __name__ == "__main__":
    node = FKNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass