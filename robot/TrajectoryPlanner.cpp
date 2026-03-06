#include "TrajectoryPlanner.h"
#include "Validation.h"
#include "Collision.h"
#include "Constants.h"

#include <iostream>
#include <cmath>
#include <algorithm>

using namespace Constants;


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
    
    //calc the distance difference between target and origin
    double diffU1 = target.U1 - origin.U1;
    double diffU2 = target.U2 - origin.U2;
    double diffR1 = target.R1 - origin.R1;
    double diffR2 = target.R2 - origin.R2;
    double diffP1 = target.P1 - origin.P1;

    //calc travel time for each joint using the max speed (time = dist/speed)
    //abs used since time must be positive
    double timeU1 = std::abs(diffU1) / 30.0;
    double timeU2 = std::abs(diffU2) / 30.0;
    double timeR1 = std::abs(diffR1) / 45.0;
    double timeR2 = std::abs(diffR2) / 45.0;
    double timeP1 = std::abs(diffP1) / 10.0;

    //used to find the maximum time taken
    double maxTime = std::max({ timeU1, timeU2, timeR1, timeR2, timeP1});

    //controller runs at 150Hz meaning 150 updates per sec. Ceil used to round up so we include the final target state
    int steps = std::ceil(maxTime * 150);

    std::cout << "Trajectory time: " << maxTime << " seconds\n";

    //flag used to record if any state along trajectory causes collision
    bool collisionDetected = false;

    //loop through each timestep of trajectory
    for(int i = 0; i <= steps; i++) {

        //used to interpolate between origin and target joint values
        double t = (double)i / steps;

        JointState state;


        //linearly interpolate each joint value between origin and target
        state.U1 = origin.U1 + t * diffU1;
        state.U2 = origin.U2 + t * diffU2;
        state.R1 = origin.R1 + t * diffR1;
        state.R2 = origin.R2 + t * diffR2;
        state.P1 = origin.P1 + t * diffP1;

        if(collision(state)) { //if interpolated joint state causes a colliison, records it but contines generating trajectory
            collisionDetected = true;
        }

        //store compted joint state into trajectory vector, each element represents one timestamp of the robot motion
        trajectory.push_back(state);
    }

    //warn user at least one point in trajectory caused a collision
    if(collisionDetected) {
        std::cout << "Collision detected\n"; 
    }
    
    return trajectory; //returns generated trajectory.

}