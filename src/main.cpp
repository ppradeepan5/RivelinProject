#include <iostream>
#include "JointState.h"
#include "ForwardKinematics.h"

int main(){
    JointState joints;

    //used to verify forward kinematics computation
    joints.U1 = 45.0;
    joints.U2 = -90.0;
    joints.R1 = -90.0;
    joints.R2 = -90.0;
    joints.P1 = 60.0;

    Position pos = forwardKinematics(joints);

    //output print
    std::cout << "X: " << pos.x << "mm" << "\n";
    std::cout << "Y: " << pos.y << "mm" << "\n";
    std::cout << "Z: " << pos.z << "mm" << "\n";

    return 0;

}