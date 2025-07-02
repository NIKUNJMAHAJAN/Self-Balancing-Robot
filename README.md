# Self-Balancing-Robot
This project involves the design and development of a self-balancing two-wheeled robot that operates on the inverted pendulum principle, where continuous real-time feedback and control are used to maintain balance.

# Core Concept
The robot uses the MPU-6050 sensor, which combines a 3-axis gyroscope and a 3-axis accelerometer, to measure the pitch angle (tilt) of the robot. However, both the accelerometer and gyroscope have limitations: the accelerometer is noisy, while the gyroscope drifts over time. To overcome this, a complementary filter is applied to fuse both readings, providing a reliable and smooth estimate of the pitch angle.

# Control Mechanism
To maintain balance, the robot uses a PID (Proportional-Integral-Derivative) controller, which continuously calculates the error between the desired setpoint (perfectly upright) and the actual pitch angle. The PID controller processes this error and generates an output signal that is used to control the NEMA 17 stepper motors via DRV8825 drivers. These motors rotate the wheels forward or backward to counteract any detected tilt, bringing the robot back to equilibrium.

# Motion and Maneuvering
Forward/Backward Motion: By intentionally shifting the setpoint angle slightly, the robot begins to "fall" in the desired direction. The PID controller interprets this as an error and causes the motors to move accordingly, resulting in forward or backward movement.

# Turning: The robot turns by creating a speed differential between the two wheels. For example, speeding up the right wheel while slowing down the left causes a left turn, and vice versa.

# Additional Features
Servo-Controlled Top Plate: A servo motor is used to keep a top-mounted parallel plate level with the ground. It uses the same pitch data to apply a proportional correction, ensuring stable alignment.


# Structural Analysis: The robot frame was analyzed using Shear Force Diagrams (SFD) and Bending Moment Diagrams (BMD) to ensure structural integrity. Calculations were made to confirm that material stress levels remain within safe limits under static and dynamic loads.

# Conclusion
This self-balancing robot project successfully combines sensor fusion, control systems, mechatronics, and mechanical design. It demonstrates how feedback control and real-time sensor data can be used to stabilize an inherently unstable system and form the foundation for advanced robotic platforms.
