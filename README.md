# Robotic Kinematics and Trajectory Planning

C++ Implementation of a robotic arm motion planning system that performs forward kinematics, joint limit validation, collision detection and trajectory generation between joint configurations.

## This project implements a modular robotics software pipeline consisting of:
- Forward kinematics for computing end-effector position
- Joint limit validation
- Self-collision detection
- Trajectory planning with trapezoidal velocity profiles
- 150Hz trajectory sampling for simulated control execution

The system evaluates whether a target configuration is reachable and warns if collisions occur along the planned path.

## Structure:
### robot/
- ForwardKinematics.cpp
- Collision.cpp
- Validation.cpp
- TrajectoryPlanner.cpp

### src/
- main.cpp

## Build:
'''bash

mkdir build

cd build

cmake ..

make
