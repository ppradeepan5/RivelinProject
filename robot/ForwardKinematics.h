#pragma once
#include "JointState.h"

// the Position struct represents a 3D Cartesian co-ordinate in the base reference frame of the robot.
struct Position {
    double x;
    double y;
    double z;
};


// forwardKinematics function computes the Cartesian position of the lipstick tip when given a set of joint values.
// Input: JointState  Output: Position (relative to base origin)
Position forwardKinematics(const JointState& joints);
