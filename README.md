### 运行

---
运行步骤：
```shell
xhost +local:root
catkin clean -y
catkin build
source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
roslaunch hero_chassis_controller run_task_4.launch 
```