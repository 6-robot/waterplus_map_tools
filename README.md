# MapTools工具

## 使用步骤

1. 安装ROS(indigo/kinetic/melodic/noetic)
2. 配置好开发环境. [配置方法](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. 获取源码:
```
cd ~/catkin_ws/src/
git clone https://github.com/6-robot/waterplus_map_tools.git
```
4. 安装依赖项(根据ROS版本选择其中一个):
```
~/catkin_ws/src/waterplus_map_tools/scripts/install_for_indigo.sh
```
```
~/catkin_ws/src/waterplus_map_tools/scripts/install_for_kinetic.sh
```
```
~/catkin_ws/src/waterplus_map_tools/scripts/install_for_melodic.sh
```
```
~/catkin_ws/src/waterplus_map_tools/scripts/install_for_noetic.sh
```
5. 编译
```
cd ~/catkin_ws
catkin_make
```

## 平台介绍
MapTools工具是[北京六部工坊科技有限公司](http://www.6-robot.com)为旗下WP系列机器人快速设置地图航点所设计的辅助工具,具有操作简单,效果直观的优点。目前支持启智ROS,启智AI,启智MANI,启程3,启程4和启明1等型号的机器人.
![Nav pic](./media/wpb_home_nav.png)

## 操作方法

### 1. 打开地图
启智ROS:
```
roslaunch waterplus_map_tools add_waypoint.launch
```
启智AI:
```
roslaunch waterplus_map_tools add_waypoint_ai.launch
```
启智MANI:
```
roslaunch waterplus_map_tools add_waypoint_mani.launch
```
启程3:
```
roslaunch waterplus_map_tools add_waypoint_wpv3.launch
```
启明1:
```
roslaunch waterplus_map_tools add_waypoint_wpr1.launch
```
![1 pic](./media/map.png)

### 2. 设置航点
在Rviz工具栏点击"Add Waypoint"按钮可在地图上设置航点。
![2 pic](./media/toolbar.png)
![3 pic](./media/add_waypoint.png)
![4 pic](./media/waypoint.png)
![MapTools pic](./media/map_tools.png)

### 3. 保存航点
航点设置完毕后,使用如下指令保存航点:
```
rosrun waterplus_map_tools wp_saver
```

### 4. 航点遍历
航点设置完毕后,可以使用如下指令让机器人将设置的航点逐个遍历:
```
rosrun waterplus_map_tools wp_nav_test
```
