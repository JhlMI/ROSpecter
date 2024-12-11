# ROSpecter
Project created for the robotics subject IMT-342, with the purpose of making an autonomous robotic vehicle using the ROS2.

Objective: To implement a system in ROS that allows the mobile robot to navigate a marker-based circuit autonomously based on visual marker detection.
*********************************************************************************
Complete the proposed circuit based on markers,
a. The circuit will be defined with 5 markers and each of the markers will be defined with ID 0 to 4.
will be defined with ID 0 through 4.
b. Each marker reading shall be taken at 20 cm distance.
c. Each marker shall have one instruction defined:
i. ID 0 -> turn 90 degrees left
ii. ID 1 -> turn 90 degrees right
iii. ID 2 -> turn 180 degrees
iv. ID 3 -> turn 90 degrees left
v. ID 4 -> turn 180 degrees and Terminate
d. The layout of the markers on the circuit will be defined on the day of the
the review with the possibility of testing prior to the final evaluation.
*********************************************************************************
STEPS TO FOLLOW:

STEP1)
Execute the camera node
STEP2)
Execute <aruco_detector.py>
STEP3)
Execute <Aruco_control.py>
STEP4)
upload <IMU_motors.ino> to ESP32

