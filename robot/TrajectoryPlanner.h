#pragma once

#include "JointState.h"
#include <vector>


//plans trajectory between the two specified joint states
//trajectory is represented as a vector of JointState objects
std::vector<JointState> planTrajectory(
    const JointState& origin,
    const JointState& target
);

