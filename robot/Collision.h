#pragma once
#include "JointState.h"

bool collision(const JointState& joints);

//checks whether given joint configuration results in a robot self-collision or violates any geometric constraints
// (e.g. if the arm folds backwords or dips below base)