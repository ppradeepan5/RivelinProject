#include "TrajectoryPlanner.h"
#include "Validation.h"

#include "Collision.h"
#include <iostream>


std::vector<JointState> planTrajectory(
    const JointState& origin,
    const JointState& target)
{
    std::vector<JointState> trajectory;


    //validates both inputs to ensure all oints are within their allowed physical limits
    if (!isWithinLimits(origin) || !isWithinLimits(target)) {
        std::cout << "Unreachable\n";
        return trajectory;
    }

    trajectory.push_back(origin); //adds origin config to trajectory
    
    trajectory.push_back(target); //adds target config to trajectory

    return trajectory; //returns generated trajectory.

}