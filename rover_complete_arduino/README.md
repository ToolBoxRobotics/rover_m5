Six-Wheel Rover — Arduino sketches + ROS bridge

This repository contains two Arduino sketches intended to run on Arduino Mega (drivetrain/sensors)
and Arduino Mega (arm steppers). They integrate with ROS via rosserial (rosserial_arduino) and
provide:
  - DRI0002 motor control via PWM + DIR pins
  - Encoder ISRs for 6 wheels (A/B quadrature) providing counts
  - PCA9685 steering servos
  - TCA9548A multiplexer to access MPU6050, AHT20, and INA219 (optional)
  - IMU (MPU6050), AHT20 (temp/humidity), INA219 (battery) readouts
  - Subscriptions to wheel/steer command topics (Float64)
  - Publishing of /joint_states, /imu/data, /aht20/temperature, /aht20/humidity, /battery, /encoders

Place these sketches in Arduino IDE as-is and install the required libraries listed at the top
of each .ino file. Connect the Arduino(s) to your SBC and run rosserial_python to bridge.
