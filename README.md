# ROSpecter
Project created for the robotics subject IMT-342, with the purpose of making an autonomous robotic vehicle using the ROS2.

Objective: To implement a system in ROS that allows the mobile robot to navigate a marker-based circuit autonomously based on visual marker detection.
<br>
*********************************************************************************
<br>
Complete the proposed circuit based on markers,
<br>
a. The circuit will be defined with 5 markers and each of the markers will be defined with ID 0 to 4.<br>
will be defined with ID 0 through 4.<br>
b. Each marker reading shall be taken at 20 cm distance.<br>
c. Each marker shall have one instruction defined:<br>
i. ID 0 -> turn 90 degrees left<br>
ii. ID 1 -> turn 90 degrees right<br>
iii. ID 2 -> turn 180 degrees<br>
iv. ID 3 -> turn 90 degrees left<br>
v. ID 4 -> turn 180 degrees and Terminate<br>
d. The layout of the markers on the circuit will be defined on the day of the<br>
the review with the possibility of testing prior to the final evaluation.<br>
<br>
*********************************************************************************
<br>
STEPS TO FOLLOW:<br>
<br>
STEP1)<br>
Execute the camera node<br>
STEP2)<br>
Execute Aruci_detector.py to view aruco codes on the frame<br>
STEP3)<br>
Execute Aruco controlo to track the aruco ID<br>
STEP4)<br>
upload IMU_motores.iono to ESP32<br>

