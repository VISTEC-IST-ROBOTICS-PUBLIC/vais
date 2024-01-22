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
