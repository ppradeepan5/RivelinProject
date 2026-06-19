# Robotic Kinematics & Trajectory Planning  
### *C++ Implementation of a Modular Robotics Motion Planning Pipeline*

This project implements a complete **robotics motion planning system** developed as part of a technical challenge.  
It includes **forward kinematics**, **joint‑space validation**, **self‑collision detection**, and **time‑optimal trajectory generation** for a 5‑DoF articulated robot with mixed joint types (universal, revolute, prismatic).

The system is written in **modern C++**, with a clean modular architecture designed for clarity, extensibility, and real‑time execution.

---

## System Capabilities

### **1. Forward Kinematics**
Computes the **3D end‑effector position** in the base frame using the robot’s joint configuration.  
Implements a full kinematic chain including:

- Universal joint (U1, U2)  
- Revolute joints (R1, R2)  
- Prismatic joint (P1)  
- Proximal, distal, and holder link lengths  

---

### **2. Joint Limit Validation**
Ensures all joint commands respect:

- Position limits  
- Maximum velocity  
- Maximum acceleration  

Validation is performed for both static configurations and full trajectories.

---

### **3. Self‑Collision Detection**
Detects collisions between:

- Proximal link  
- Distal link  
- Holder link  
- Ground plane (z = 0)  

Links are treated as zero‑thickness line segments for geometric intersection testing.

---

### **4. Time‑Optimal Trajectory Planning**
Generates the **fastest possible joint‑space trajectory** between two configurations such that:

- All joints start and stop simultaneously  
- Velocity and acceleration limits are respected  
- Sampling occurs at **150 Hz** for real‑time streaming  
- Collisions along the path are detected and reported  

Uses **trapezoidal velocity profiles** for each joint.

---

## Architecture Overview

**robot**
ForwardKinematics.cpp     
Collision.cpp             
Validation.cpp            
TrajectoryPlanner.cpp   


**src**
main.cpp                


The system is intentionally modular — each component can be tested, extended, or replaced independently.

---

## Test Case Handling

The implementation supports all required test cases, including:

- **Reachable configurations**  
- **Unreachable joint states**  
- **Self-collision scenarios**  
- **Ground collisions**  
- **Bonus edge cases**  

Each test produces clear diagnostic output describing:

- Reachability  
- Collision type  
- Trajectory feasibility  
- End-effector position  

Tests are also fully automated.

<img width="200" height="297" alt="image" src="https://github.com/user-attachments/assets/846f10cd-3c76-474c-9e5a-40649e9763ac" />

<img width="320" height="445" alt="image" src="https://github.com/user-attachments/assets/204dc873-c98b-40cd-b3b4-ae3df24c56de" />










---

## Build & Run

```bash
mkdir build
cd build
cmake ..
make
./RivelinProject



