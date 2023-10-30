#square_following

Project Description:
In this project, we developed a robot control system where a Turtlebot robot follows a straight line by observing a square using its onboard sensor. The aim was to control the robot in such a way that it follows a line perpendicular to a square in its field of vision. We implemented the control logic using the ROS (Robot Operating System) framework and Python.

Core Features:
State Machine Approach: The control logic uses a state machine with three main states: SEARCHING, FOLLOWING, and STOPPED.

Square Detection: Using OpenCV's contour detection, we detect squares in the robot's camera feed.

Dynamic Behavior: If the square goes out of the field of vision, the robot switches to the SEARCHING state where it tries to find the square again.

Adaptive Control: The robot adapts its direction based on the position of the square relative to the image center, aiming to follow a line perpendicular to the square.

How to Launch Simulation:

roslaunch square_following simulation.launch

Check dependencies

Follow "sim_setup_guide"

Python 2.7.17
18.04
Melodic Full Desktop
