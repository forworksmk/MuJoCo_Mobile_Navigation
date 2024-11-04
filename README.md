# Path Tracker for Mobile Navigation
Path Tracker for Mobile Navigation is a general-purpose path tracker based on 2D map. It automatically control the path of the mobile navigation when the robot reached to the target/goal point.

## Terms
#### Target Point(Pos) : Next destination(position) of the mobile robot
#### Goal Point(Pos) : Final destination(position) of the mobile robot
[Note] Path should be like this: [ #1 Target Point, #2 Target Point, .... , Goal Point(Final Target Point) ]


# Setup
```
pip install mujoco
```
Also, use custom mujoco class and its own function to get specific body's position and visualize points.
(You need to replace these functions based on your mujoco class)

# Python usage
### Initialize
```
path_tracker = PathTracker_2D(path=robot_path,target_distance=0.1,goal_distance=0.05,body_name="base_kobuki",mj_env=mujoco_env)
```
Initialize path tracker class with the upper function.
It needs 5 inputs:
- path : robot's path for mobile navigation
- target_distance : threshold distance for detecting the robot reached to the target point
- goal_distance : threshold distance for detecting the robot reached to the goal point (final target point)
- body_name : body name of the robot which is written on the xml
- mj_env : mujoco environment class which has these functions, 'mj_env.get_p_body(self.body_name)' to get body position based on body name and 'mj_env.plot_sphere(p,rgba)' to draw sphere on specific position.


### Get target position
```
path_tracker.get_target_pos()

if target_pos is None:
    break
else:
    target_pos = target_pos[:2]
```
Get current target position. Whenever this function is called, check if the robot reach to the target position or the goal position. If it reached to the target position, it returns the next target position. If it reached to the goal position, it returns 'None'.



# Example
### mobile_navigation.ipynb
part of the mobile_navigation.ipynb code.
```
...

# path
robot_path = [[-1,-2,0],[-1,0,0],[1,0,0],[1,2,0]]
mujoco_env.set_p_base_body(body_name='base_kobuki',p=np.array([-1,-2,0]))
mujoco_env.forward()
path_tracker = PathTracker_2D(robot_path,0.1,0.05,"base_kobuki",mujoco_env)

...

while mujoco_env.is_viewer_alive():
    current_pos = mujoco_env.get_p_body("base_kobuki")[:2].copy()
    current_yaw = r2rpy(mujoco_env.get_R_body("base_kobuki"))[2]
    target_pos = path_tracker.get_target_pos()
    
    # returns None when it reached to the goal position
    if target_pos is None:
        break
    else:
        target_pos = target_pos[:2]
        ctrl_wheel = nav.compute_ctrl_wheel(mujoco_env.get_sim_time(),
                                        current_pos,
                                        target_pos,
                                        current_yaw,
                                        None,
                                        )
        path_tracker.visualize_path()
        mujoco_env.step(ctrl=ctrl_wheel, ctrl_idxs= ctrl_wheel_idxs)
        mujoco_env.render()

mujoco_env.close_viewer()

```