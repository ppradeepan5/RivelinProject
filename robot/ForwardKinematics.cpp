#include "ForwardKinematics.h"
#include "Constants.h"
#include <cmath>

using namespace Constants;

constexpr double pi = 3.14159;

//converting degrees to radians
double radconv(double degrees){
    return degrees * pi / 180.0;
}

/*Computes cartesian end position from joint configuration
 Treated as a chain (Base -> Proximal -> Distal -> Holder -> Prismatic Extension)
*/
Position forwardKinematics(const JointState& j){
    double u1 = radconv(j.U1);
    double u2 = radconv(j.U2);
    double r1 = radconv(j.R1);
    double r2 = radconv(j.R2);

    double x = 0.0;
    double y = 0.0;
    double z = BASE_LINK; //Z starts at base_link to account for permanent vert base height

//Strategy used: Compute link projection in XZ plane then rotate around Z using base yaw (u1)

    //Proximal Link

    double ang1 = u2; //First pitch rotation (u2)

    double proxx = PROXIMAL_LINK * std::cos(ang1);
    double proxz = PROXIMAL_LINK * std::sin(ang1);

    x += proxx * std::cos(u1);
    y += proxx * std::sin(u1);
    z += proxz;

    //Distal Link
    double ang2 = u2 + r1; //pitch accumulates.

    double distx = DISTAL_LINK * std::cos(ang2);
    double distz = DISTAL_LINK * std::sin(ang2);

    x += distx * std::cos(u1);
    y += distx * std::sin(u1);
    z += distz;


    //Holder Link
    double ang3 = u2 + r1 + r2; //additioanl pitch added

    double holdx = HOLDER_LINK * std::cos(ang3);
    double holdz = HOLDER_LINK * std::sin(ang3);

    x += holdx * std::cos(u1);
    y += holdx * std::sin(u1);
    z += holdz;

    //Prismatic Extension
    double linkx = j.P1 * std::cos(ang3); //uses same cumulative pitch as the one used in holder
    double linkz = j.P1 * std::sin(ang3);

    x += linkx * std::cos(u1);
    y += linkx * std::sin(u1);
    z += linkz;

    //return final end position
    return {x, y, z};

}
