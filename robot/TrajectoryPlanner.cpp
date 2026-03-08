#include "TrajectoryPlanner.h"
#include "Validation.h"
#include "Collision.h"
#include "Constants.h"

#include <iostream>
#include <cmath>
#include <algorithm>

using namespace Constants;

//switched to a trapezoidal velocity profile for a more realistic trajectory plan
double moveTime(double dist, double vmax, double acc) {
    dist = std::abs(dist); //dist moved must always be postiive
    double t_acc = vmax / acc; //time needed to move from 0 to max v using given max acceleraton
    double d_acc = 0.5 * acc * t_acc * t_acc; //dist travelled through acceleration phase
    double d_total_acc = 2 * d_acc; //total dist covererd (acceleration + deacceleration)

    if (dist > d_total_acc) { //case1: distance is large enough to reach max v
        double d_const = dist - d_total_acc;
        double t_const = d_const / vmax;

        return 2 * t_acc + t_const;
    }

    else { //case2: distance is too short to reach max velocity so robot follows a triangular velocity profile instead
        return 2 * std::sqrt(dist / acc); 
    }
}


std::vector<JointState> planTrajectory(
    const JointState& origin,
    const JointState& target)
{
    std::vector<JointState> trajectory;


    if (!isWithinLimits(origin) || !isWithinLimits(target)) {
        return trajectory;
    }
    
    //calc the distance difference between target and origin
    double diffU1 = target.U1 - origin.U1;
    double diffU2 = target.U2 - origin.U2;
    double diffR1 = target.R1 - origin.R1;
    double diffR2 = target.R2 - origin.R2;
    double diffP1 = target.P1 - origin.P1;

    //calc travel time for each joint using trapezoid velocity motion.
    double timeU1 = moveTime(diffU1, 30.0, 120.0); 
    double timeU2 = moveTime(diffU2, 30.0, 120.0);
    double timeR1 = moveTime(diffR1, 45.0, 180.0);
    double timeR2 = moveTime(diffR2, 45.0, 180.0);
    double timeP1 = moveTime(diffP1, 10.0, 50.0);


    //all joint must start and finish at same time so overall trajectory duration is determined by slowest moving joint
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
        std::cout << "Collision detected at one or more points in trajectory\n"; 
    }
    
    return trajectory; //returns generated trajectory.

}