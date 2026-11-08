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

在hero.urdf.xacro文件下面加入下面的文字
```xml
  <!-- Gazebo 仿真控制插件（必须放在最外层） -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>

```