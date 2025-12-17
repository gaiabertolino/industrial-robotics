# Industrial Robotics: Kinematics and Trajectory Planning

#### Keywords

`Industrial Robotics`, `Manipulator Kinematics`, `Forward Kinematics`, `Inverse Kinematics`, `Differential Kinematics`, `Trajectory Planning`, `Jacobian`, `Denavit-Hartenberg`, `MATLAB`, `Robotics Toolbox`.

This project focuses on the **modeling, analysis, and simulation of an industrial robotic manipulator**, with particular emphasis on kinematic modeling and trajectory planning. The goal is to design and analyze a serial robot by implementing **forward kinematics**, **inverse kinematics**, and **differential kinematics**, and by generating smooth trajectories for the end effector.

The robot is modeled using the **Denavit–Hartenberg convention** and simulated in **MATLAB**, exploiting numerical methods based on the **Jacobian matrix** for motion control. Both **triangular** and **circular trajectories** are planned, ensuring smooth velocity profiles and continuous motion.

This project was developed as part of the coursework exam for the *Robotics* course during the academic year **2021/2022**.

---

### Key Features

* **Serial Manipulator Modeling**: Robot structure defined through Denavit–Hartenberg parameters
* **Kinematic Analysis**: Forward, inverse, and differential kinematics implementation
* **Trajectory Planning**: Smooth triangular and circular end-effector trajectories
* **Jacobian-Based Control**: Numerical solution of inverse kinematics and velocity computation
* **MATLAB Simulation**: Visualization of motion, joint variables, and velocity profiles

---

### Project Structure

* **Robot.m**
  Defines the robotic manipulator using the *Robotics Toolbox* by Peter Corke.
  Includes link definitions, joint types, and Denavit–Hartenberg parameters through `Link` and `SerialLink`

* **CinematicaDiretta.m**
  Implements **forward kinematics**, computing the end-effector pose by multiplying homogeneous transformation matrices

* **CinematicaInversa.m**
  Solves the **inverse kinematics problem** numerically, determining joint variables required to reach a desired end-effector position

* **jacobiano.m**
  Computes the **Jacobian matrix**, relating joint velocities to Cartesian velocities for differential kinematics

* **errore_3link.m**
  Defines the error function used in the numerical solution of inverse kinematics for the three-link manipulator

* **poly3.m**
  Implements third-degree polynomial interpolation for **trajectory generation**, ensuring zero initial and final velocity

* **poly3d.m**
  Extension of the cubic polynomial method used to compute **trajectory derivatives**, enabling smooth velocity profiles

---

### Simulation and Analysis

* Animated visualization of the end-effector motion in 3D space
* Joint position and velocity plots over time
* Analysis of trajectory smoothness and stopping conditions at key points
