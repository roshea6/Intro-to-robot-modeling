# Instructions

1. Install the necessary ROS packages for the project
sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control
sudo apt-get install ros-galactic-controller-manager
sudo apt-get install ros-galactic-xacro

2. Install the necessary python packages
numpy
matplotlib
opencv-python
sympy
pupil-apriltags

3. Install the package into your workspace

4. Clone the following the repo and follow the instructions in their README to copy the apriltag models into the local gazebo models

5. Build the workspace

6. Plug in a compatible controller

7. Execute the following commands to launch the required world and nodes
ros2 run joy joy_node js:=jsx where x is your input device number
ros2 launch aircraft_inspection_robot gazebo.launch.py 
ros2 run aircraft_inspection_robot joystick_teleop.py 
ros2 run aircraft_inspection_robot task_detection.py 

8. You may need to click the check boxes for the task and depth images to get them to appear in the RVIZ display

9. Use the following button breakdown to control the system


LB: 
End effector translation deadman switch

LB + Left vertical axis:    
End effector base frame Z axis translation velocity

LB + Left horizontal axis:  
End effector base frame Y axis  translation velocity

LB + Right vertical axis:   
End effector base frame X axis  translation velocity

RT: 
End effector rotation deadman switch

RT + Left vertical axis:    
End effector local frame Z axis rotation velocity

RT + Left horizontal axis:  
End effector local frame Y axis rotation velocity

RT + Right vertical axis:   
End effector local frame X axis rotation velocity

RB: 
Mobile base deadman switch

RB + Left vertical axis:    
Mobile base x direction velocity

RB + Right horizontal axis: 
Mobile base z axis rotation velocity

