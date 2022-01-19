# AAU Robotics BSc, P3: Operating a telerobotic arm using EMG and IMU signals

## Objectives
+ Learn how to manipulate a robotic arm
+ Develop a closed-loop controller
+ Make a mathematical model of the robot arm by using kinematics and dynamics
+ Learn how to interpret electromyography (EMG) and inertial measurement unit (IMU) signals
+ Become familiar with low-level programming and how to implement a software protocol
+ Learn C++ and how to run it on hardware devices

## Project description
This project aims to have a user manipulate a robotic arm with a MyoBand, by using Bluetooth communication. The user attempts to move the end-effector, and based on that information joint angles are computed with the help of inverse kinematics. The type of control mode is torque control which is achieved by mathematical modeling and sending translated PWM values to the motors. This type of controller allows the arm to respond to external forces, hold itself against gravity and pick up slightly heavy objects. This type of remote robot control is useful in harsh environments, where people cannot operate. 

**Used tools:** Maple, the Robotics Toolbox by Peter Corke in MATLAB, C++, the MyoBand SDK

**Resources:** Control Systems Engineering by Norman S. Nise, Introduction to Robotics: Mechanics and Control by John J. Craig. 

**Used hardware:** Crust Crawler manipulator with Dynamixel motors, Arduino Mega, RS485 Breakout board. 

**Folder description:**
+ `DynamixelProtocol2` folder contains the library for interfacing an Arduino with a Dynamixel motor using the Dynamixel Communication Protocol 2.0.
+ `MyoBand` folder contains the application used to interface an Arduino with a MyoBand.
+ `Kinematics` folder contains the mathematical model in regards to the mechanics of the robot arm without taking into account the forces which cause its motion. Forward and inverse kinematics are included. 
+ `Dynamics` folder contains the mathematical model of the arm concerned with its motion under the action of external forces.
+ `ControlSystems` folder contains the steps taken for deriving a closed-loop controller using the combined mathematical models from kinematics and dynamics. It covers how to arrive from a non-linear system to a linear one, transform it in the frequency domain and use the learned techniques to find the PD gains (Kp and Kd). The controller is developed given a set of system requirements such as desired overshoot, settling time, and steady-state error. 

## How to use
Most of the above-mentioned techniques are combined and implemented in a sigle file, called `controlTheArm.ino`. That has to run together with the MyoBand application, meaning that the Arduino file runs on the microcontroller and the `hello-myo-VisualStudio2012` project runs in the Visual Studio IDE. 

It is necessary to have the Terminal in Visual Studio closed in order to be able to upload new code to the Arduino.

It is necessary to have all the mentioned above hardware, the Arduino IDE and Visual Studio installed, and the *DynamixelProtocol2* library added to the *libraries* folder of your Arduino program.  
