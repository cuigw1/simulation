**kejia_mapping**  
**gmapping相关的配置文件和参数文件**  
-------------------------
-------------------------

**建图**：使用ros的slam_gmappin建图  
1)使用Gazebo中的移动机器人创建一个rosbag  
&nbsp;&nbsp;i.  运行 `roslaunch kejia_sim map.launch`  
&nbsp;&nbsp;ii. 运行 rosrun teleop_tools mouse_teleop.py(可选，py脚本用于移动机器人)  
&nbsp;&nbsp;iii.`rosbag record -o (bag name) /scan /tf` (记录需要的传感器信息，可>以看情况加减)  
&nbsp;&nbsp;iv. 在Gazebo中移动小车扫描整个地图，结束后ctrl+c退出  
2)使用跑出来的rosbag建图  
&nbsp;&nbsp;i. `rosmake gmapping`  
&nbsp;&nbsp;ii. `roscore`  
&nbsp;&nbsp;iii.`rosparam set use_sim_time true`  
&nbsp;&nbsp;iv. `rosrun gmapping slam_gmapping scan:=scan`  
&nbsp;&nbsp;v. `rosbag play --clock (之前跑出来的bag)`  
&nbsp;&nbsp;iv. 等上一步把bag数据读完 `rosrun map_server map_saver -f (map name)`  
&nbsp;&nbsp;iiv.生成地图文件 .pgm文件和.yaml文件  


文件结构：  
------------
launch
  ----gmapping.launch&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;----gmapping相关参数  
final_map.yaml&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;----地图文件  
final_map.pgm&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;----地图文件  
CMakeList.txt  
package.xml  

