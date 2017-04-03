# find_camera_pose

1. Save the image you want to search as "rawQuery.png" in the project folder.
2. Make sure to create "frames", "images", "results" and "KalmanFilter" folders if they do not exist.
3. Run ```roscore```
4. Run ```rosrun joy joy_node```
5. Connect your laptop to quadcopter WiFi
6. Run ```roslaunch bebop_converter start.launch```
7. Run ```roslaunch bebop_converter tum_ardrone.launch```
8. Run ```rosrun lsd_slam_viewer viewer```
10. Run ```rosrun find_camera_pose find_camera_pose```
11. Explore the environment for a period of time you set.
12. After exploration drone lands, calculates goal location and navigate there automatically.

Youtube Movie:
https://www.youtube.com/watch?v=BLY3kgeZrZg
