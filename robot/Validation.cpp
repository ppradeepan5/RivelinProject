#include "Validation.h"
#include "Constants.h"

//Validates that each joint value lines within the allowed range - values defined in Constants header fiel.
bool isWithinLimits(const JointState& j){
    using namespace Constants;
    
    if (j.U1 < U1_MIN || j.U1 > U1_MAX) return false;
    if (j.U2 < U2_MIN || j.U2 > U2_MAX) return false;
    if (j.R1 < R1_MIN || j.R1 > R1_MAX) return false;
    if (j.R2 < R2_MIN || j.R2 > R2_MAX) return false;
    if (j.P1 < P1_MIN || j.P1 > P1_MAX) return false;

    return true;
}

