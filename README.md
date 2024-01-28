# Visual-Based Adpative Interaction System 

This is a source code of Visual-Based Adaptive Interaction System (VAIS) using visual information and a fast neural learning mechanism called Modified Input Correlation-Based Learning (MICO) to improve a service delivery on a mobile service robot (MOVO).

---

# Requirement

1. Ubuntu 16.04 with ROS Kinectic
2. ROS Alvar

---

# Framework

This framework is divided into four sub-components:

1. Visual Perception: ROS Alvar package was utilized to obtain a pose-estimation of the object with the attached AR tag (QR marker provided by the delivers).
2. Signal Generator: A conversion of object's pose-estimation to the input signals using normalization of the Euclidean Distance between reference point and current position.
3. MICO: A fast modified neural network that adapt the robot behavior (speed) based on the relationship between the input signals.
4. Output Conversion: The conversion of neural output to the robot's speed output.

# Structure
This source code is based on ROS-Python containing:

1. launch: This launch folder contains files which is used to execute ROS package via ```roslaunch``` command.
2. script: This script folder contains necessary files that called methods from src folder which executed in roslaunch.
3. msg: This is a folder which stored a custom message file.
4. src: This folder contain a main source code. In this folder, the source code contains 6 sub-folders:

- ICO: This folder is where the learning modules are stored. It contains signal generator, mICO learning, and data management to store/load the learning weight. The output of this module is a neural output (o_neural) which is used in an output method.
- demo: A demo source code which performed a combination of both translational and rotational motions in a controlled environment.
- menu: This is a main menu where the user can choose to operate the script with a default paramters or enter the manual values for the learning mechanism.
- output: This output folder receives the neural output from the mICO module, and convert it into the robot's output speed (o_speed). Noted that this module is specifically to a MOVO module. In case of the other robot, user need to rewrite this code to operate on that robot.
- posture: This is only used for a MOVO to adjust it into a specific posture.
- test: A test folder which copied some script from MOVO package to test some MOVO specific components.


# Execution

1. Start a camera node by ```roslaunch vais camera.launch```.
2. Start an alvar node by ```roslaunch vais alvar.launch```.
3. Start a VAIS node by ```roslaunch vais vais.launch```.
4. Follow the instruction on ```ICO_menu.py``` command line.

# Customization

To customize the learning framework, user can adjust these components under the ```vais/src/menu/ICO_menu.py``` in a default_value method:

1. Distance threshold: User can manually set exemption threshold (e_object), predictive threshold (p_object), and reflexive treshold (r_object) in centimeter unit.
2. Learning rate: Learning rate of the MICO.
3. State: This parameter can be set as a robot's movement. 'Linear' (Linear translation movement) or 'Angular' (Angular rotation movement).
4. Alvar tag ID: The number of the ar_id tag that use to track the object.
5. Goal: This is where the robot stop its movement (i.e., finishing line). goal_x is a linear translation movement (in meters) and goal_z is an angular rotation movement (in degrees).

However, if the user has the other robot, user need to change the direction of the topics described below:

1. User can create a new camera launch file and edit the ```alvar.launch``` file on "cam_image_topic", "cam_info_topic", "output_frame" to the robot you are working on.

2. Create or edit the ```vais/src/output/base_output.py``` to subscribe and publish these topics according to your robot.

# Note
In this source code, the folder ICO is the modified Input Correlation-Based Learning (mICO) corresponding to the paper.
