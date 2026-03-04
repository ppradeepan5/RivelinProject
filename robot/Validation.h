#pragma once
#include "JointState.h"

bool isWithinLimits(const JointState& j);

//checks whether all joints fall within their allowed physical limits