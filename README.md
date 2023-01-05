# Drone_object_detection
Drone object detection and avoidance

* Clone the repository in your workspace along with this repo
```
git clone https://github.com/RAFALAMAO/hector_quadrotor_noetic
```

## Package
This package contains two launch files

**1.test.launch**
This launches the hector drone in an empty world.

**2. bring_hector_drone.launch**
This launches the drone in a city world environment.

There are three scripts for the control of the drone.


_Obstacle avoidance_

Used the sonar sensors of the hector drone to detect obstacles in its path , dividing the scan into three regions and moving around obstacles.
## to simulate headwinds a constant force is also applied on the drone.





https://user-images.githubusercontent.com/87862080/210871987-de8fda08-e51e-4920-9f61-2129495116cd.mp4








_camera.py_
This script opens up realtime object detection by the camera feed.



https://user-images.githubusercontent.com/87862080/210872009-9569d7d2-0f92-4ae7-ba7c-020e36779504.mp4



