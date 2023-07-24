source ~/Jaka_ROS/jaka_ros_driver_cpp/devel/setup.bash      放入.bashrc中

-------------------新.静态抓取----------------------
终端1：开启roscore
	roscore
终端2：开启AG-95夹爪
	roslaunch dh_hand_driver dh_hand_controller.launch
终端3：开启jakazu3连接（先连接jaka的局域网）
	roslaunch jaka_ros_driver start.launch
终端4：开启海康相机
	roslaunch hk_camera hk_camera.launch

开始抓取（需要进入工作空间）
	cd ~/Jaka_ROS/jaka_ros_driver_cpp
	rosrun jaka_ros_driver static_grap_move_num
```

如只需要视觉检测，可对image_transfer中的图片image.jpg进行修改，或者直接改动地址
