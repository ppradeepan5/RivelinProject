#include <iostream>
#include "JointState.h"
#include "ForwardKinematics.h"
#include "Validation.h"
#include "Collision.h"


int main(){
    JointState joints;

    //used to verify forward kinematics computation
    joints.U1 = -45.0;
    joints.U2 = 40.0;
    joints.R1 = 140.0;
    joints.R2 = 0.0;
    joints.P1 = 50.0;

    // check if joint values are within allowed limits
    if(!isWithinLimits(joints)){
        std::cout << "||Unreachable||\n";
        return 0;
    }

    //checks if configuration causes a collision
    if(collision(joints)){
        std::cout << "||Collision||\n";
        return 0;
    }
    

    //computes Cartesian position of the robot end effector
    Position pos = forwardKinematics(joints);

    //prints final reachable position
    std::cout << "||Reachable||" << "\n";
    std::cout << "X: " << pos.x << "mm" << "\n";
    std::cout << "Y: " << pos.y << "mm" << "\n";
    std::cout << "Z: " << pos.z << "mm" << "\n";

    return 0;

}