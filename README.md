# find_camera_pose

1. Save the image you want to search as "query.png" in the project folder.
2. Make sure to create "frames", "images" and "results" folders if they do not exist.
3. Run ```roscore```
4. Run ```rosrun lsd_slam_core live_slam /image:=/ardrone/image_raw _calib:=/home/tapir/catkin_ws/src/bebop_lsd_slam/lsd_slam_core/calib/bebop_calibration.cfg```
5. Run ```rosbag play testfile.bag ```
6. Run ``` rosrun find_camera_pose find_camera_pose```



Note:
testfile.bag is the bagfile recorded in the lab, feel free to adjust steps 4 and 5 according to your data.
