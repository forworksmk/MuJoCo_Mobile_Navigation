import numpy as np

class PathTracker_2D:
    def __init__(self,path,target_distance,goal_distance,body_name,mj_env):
        self.path = path
        self.path_idx = 0
        self.target_distance = target_distance
        self.goal_distance = goal_distance
        self.body_name = body_name

        self.mj_env = mj_env
 
    def set_new_path(self,path,target_distance,goal_distance):
        self.path = path
        self.path_idx = 0
        self.target_distance = target_distance
        self.goal_distance = goal_distance

    def is_reached_target(self):
        current_pos = self.mj_env.get_p_body(self.body_name)
        target_pos = self.path[self.path_idx]
        distance = np.linalg.norm(target_pos[:2] - current_pos[:2])

        if distance < self.target_distance:
            return True
        else:
            return False
        
    def is_reached_goal(self):
        current_pos = self.mj_env.get_p_body(self.body_name)
        target_pos = self.path[self.path_idx]
        distance = np.linalg.norm(target_pos[:2] - current_pos[:2])

        if distance < self.goal_distance:
            return True
        else:
            return False
        
    def visualize_path(self):
        for _idx, each_point in enumerate(self.path):
            if _idx < self.path_idx:
                continue
            # target point
            if len(self.path) -1 > _idx:
                self.mj_env.plot_sphere([each_point[0],each_point[1],0.05],0.08,rgba=[0,1,0,1],label='')
            # goal point
            else:
                self.mj_env.plot_sphere([each_point[0],each_point[1],0.05],0.08,rgba=[1,0,0,1],label='')
        
    def get_target_pos(self):
        # target pos
        if self.path_idx < len(self.path) - 1:
            if self.is_reached_target():
                self.path_idx += 1
            return self.path[self.path_idx]
        # goal pos
        else:
            if self.is_reached_goal():
                return None
            else:
                return self.path[self.path_idx]