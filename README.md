**Multirotor PID Control System**
This project implements a multirotor PID control system in Python. The primary focus is on controlling the yaw, roll, and pitch of a multirotor vehicle by using Proportional-Integral-Derivative (PID) controllers. The system simulates the physics of the multirotor, including torque and angular velocity, to adjust motor throttle inputs in real time and maintain stable flight.

Key Components:
PID Controllers: Separate PID loops for yaw, roll, and pitch are integrated. Each controller computes motor throttle adjustments based on the target and current orientation.

Motor Throttles: The motor throttles are adjusted based on the control output of the PID controllers to balance the forces and torques on the multirotor.

Physics Simulation: The code includes physics calculations for angular velocity, torque, and motor forces. These allow the system to simulate how the multirotor would behave in response to motor adjustments.

Visualization: A visualization of the multirotor and its orientation is provided using vPython, allowing real-time feedback on the system's behavior during simulation.

How it Works:
Input Targets: The system receives target yaw, roll, and pitch angles.
PID Calculation: The PID controllers compare the current orientation to the targets and calculate corrections.
Motor Throttle Adjustments: The calculated corrections are applied to the motors, adjusting their throttle to achieve the desired orientation.
Simulation Feedback: The system simulates the resulting motion, updating the orientation and feeding back into the PID loop.
Installation:
To run this project, you will need:

Python 3.x
vPython for visualization
