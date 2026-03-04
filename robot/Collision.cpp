#include "Collision.h"
#include "Constants.h"
#include <cmath>

using namespace Constants;


bool collision(const JointState& j){
    double u2 = radconv(j.U2); //converts degrees to radians
    double r1 = radconv(j.R1);
    double r2 = radconv(j.R2);

    //cumulative pitch angles for each link in the kinematics chain (ang1 = proximal link, ang2 = distal link, ang3 = holder link)
    double ang1 = u2;
    double ang2 = u2 + r1;
    double ang3 = u2 + r1 + r2;

    //proximal endpoint
    double x1 = PROXIMAL_LINK * cos(ang1);
    double z1 = BASE_LINK + PROXIMAL_LINK * sin(ang1);

    //distal endpoint
    double x2 = x1 + DISTAL_LINK * cos(ang2);
    double z2 = z1 + DISTAL_LINK * sin(ang2);
    
    //holder endpoint
    double x3 = x2 + HOLDER_LINK * cos(ang3);
    double z3 = z2 + HOLDER_LINK * sin(ang3);


    // collision condition 1: if holder link goes below base height, arm would interesect with robot base
    if (z3 < BASE_LINK) {
        return true;
    }

    //collision condition 2: detects self collision if distal link folds back past proximal link
    if (ang2 < ang1) {
        return true;
    }

    //no collision occured
    return false;

}
