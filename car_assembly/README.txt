1. Install the necessary ROS packages for the project
sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control
sudo apt-get install ros-galactic-controller-manager
sudo apt-get install ros-galactic-xacro

2. Install the necessary python packages
numpy
matplotlib

3. Install the package into your workspace

4. Build the workspace

5. For the competition scenario make the following code changes and run the following commands
Uncomment lines 25 and 28 in spawn_robot_ros2.launch.py
Make sure lines 26 and 29 are commented
Rebuild workspace and source your setup.bash file
ros2 launch car_assembly competition.launch.py
ros2 run car_assembly teleop_control.py 

6. For proportional control scenario make the following code changes and run the following commands
Uncomment line 26 in spawn_robot_ros2.launch.py
Make sure line 25 is commented
In proportional_controller.py line 21, set the real_time_factor variable to the average value that your gazebo usually has for Real Time Factor.
This will help account for the simulation slowing down to below real time.
Rebuild the workspace source your setup.bash file
ros2 launch car_assembly gazebo.launch.py
ros2 run car_assembly proportional_controller.py
