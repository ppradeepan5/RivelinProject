#include <iostream>
#include "JointState.h"
#include "ForwardKinematics.h"
#include "Validation.h"
#include "Collision.h"
#include "TrajectoryPlanner.h"


int main(){
    JointState joints;

    //used to verify forward kinematics computation
    joints.U1 = 45.0;
    joints.U2 = -90.0;
    joints.R1 = -90.0;
    joints.R2 = -90.0;
    joints.P1 = 60.0;

    // check if joint values are within allowed limits
    if(!isWithinLimits(joints)){
        std::cout << "||Unreachable||\n";
       // return 0;
    }

    //checks if configuration causes a collision
    if(collision(joints)){
        std::cout << "||Collision||\n";
      //  return 0;
    }
    

    //computes Cartesian position of the robot end effector
    Position pos = forwardKinematics(joints);

    //prints final reachable position
    std::cout << "||Reachable||" << "\n";
    std::cout << "X: " << pos.x << "mm" << "\n";
    std::cout << "Y: " << pos.y << "mm" << "\n";
    std::cout << "Z: " << pos.z << "mm" << "\n";



    JointState origin = {0, 0, 0 , 0, 0};
    JointState target = {45, -90, -90, -90, 50};

    auto trajectory = planTrajectory(origin, target);

    std::cout << "Trajectory size: " << trajectory.size() << "\n";

    return 0;


}