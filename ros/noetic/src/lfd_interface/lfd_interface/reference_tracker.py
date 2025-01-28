import numpy as np

class ReferenceTracker:
    """
    Simple reference tracker to track a set of waypoints
    """
    def __init__(self, tolerance = 0.1, tolerance_gripper=0.08, ub = 1.0, lb = 0.1):
        self.waypoint_list = []
        self.tolerance = tolerance
        self.tolerance_gripper = tolerance_gripper
        self.GOAL_CLOSE = False

        self.ub = ub
        self.lb = lb
        self.current_distance_to_goal = 10.0

    def euclidian_distance(self, pos_0, pos_1):
        return np.linalg.norm(pos_0 - pos_1)

    def update_reference(self, waypoint_list:list):
        self.waypoint_list = waypoint_list

    def update_local_goal(self, current_pos: np.ndarray, waypoint_list=None) -> (list, list):
        if waypoint_list is None:
            waypoint_list = self.waypoint_list

        if self.euclidian_distance(np.array(current_pos), np.array(waypoint_list[-1])) < self.tolerance:
            return waypoint_list[-1]
        else:
            for i in range(len(waypoint_list)):
                if self.euclidian_distance(np.array(current_pos), np.array(waypoint_list[i])) > self.tolerance:
                    self.waypoint_list = waypoint_list[i:]
                    return waypoint_list[0]
        print("warning: no waypoints on the path are closer than the tolerance")
        return waypoint_list[0]
    
    def main(self):
        waypoint_list = [[1, 2, 3], 
                         [1, 2, 4],
                         [1, 2, 5]] #list of positions (x, y, z)
        self.update_reference(waypoint_list)
        current_pos = [1, 2, 2]
        for t in range(100):
            local_goal = self.update_local_goal(current_pos=current_pos)
            current_pos[2] = current_pos[2] + 0.05
            
    def main_joints(self):
        waypoint_list = [[1, 2, 3, 0, 0, 0], 
                         [1, 2, 4, 0, 0, 0],
                         [1, 2, 5, 0, 0, 0]] #list of positions (x, y, z)
        self.update_reference(waypoint_list)
        current_pos = [1, 2, 2, 0, 0, 0]
        for t in range(100):
            local_goal = self.update_local_goal(current_pos=current_pos)
            current_pos[2] = current_pos[2] + 0.05
            
if __name__ == "__main__":
    reference_tracker = ReferenceTracker()
    reference_tracker.main()
    reference_tracker.main_joints()