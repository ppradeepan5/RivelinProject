#include <iostream>
#include "JointState.h"
#include "ForwardKinematics.h"
#include "Validation.h"

int main(){
    JointState joints;

    //used to verify forward kinematics computation
    joints.U1 = 0.0;
    joints.U2 = 0.0;
    joints.R1 = 0.0;
    joints.R2 = 0.0;
    joints.P1 = 0.0;

    if(!isWithinLimits(joints)){
        std::cout << "||Unreachable||\n";
        return 0;
    }

    Position pos = forwardKinematics(joints);

    //output print
    std::cout << "||Reachable||" << "\n";
    std::cout << "X: " << pos.x << "mm" << "\n";
    std::cout << "Y: " << pos.y << "mm" << "\n";
    std::cout << "Z: " << pos.z << "mm" << "\n";

    return 0;

}