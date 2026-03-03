#pragma once

struct JointState
{
    double U1;
    double U2;
    double R1;
    double R2;
    double P1;
};

/*
- This data structure represents the 5 joints of the robot.
- U1, U2, R1, R2 are expressed in degrees
- P1 is expressed in mm

- this struct acts as a data container that can be passed between the two modules - forward kinematics and trajectory planning.

*/