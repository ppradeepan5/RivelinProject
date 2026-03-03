#pragma once

//contains all physical dimensions and joint limits of the robot.
namespace Constants {
    constexpr double BASE_LINK = 300.0;
    constexpr double PROXIMAL_LINK = 400.0;
    constexpr double DISTAL_LINK = 350.0;
    constexpr double HOLDER_LINK = 50.0;

    constexpr double U1_MIN = -90.0;
    constexpr double U1_MAX = 90.0;

    constexpr double U2_MIN = -90.0;
    constexpr double U2_MAX = 90.0;

    constexpr double R1_MIN = -150.0;
    constexpr double R1_MAX = 150.0;

    constexpr double R2_MIN = -150.0;
    constexpr double R2_MAX = 150.0;

    constexpr double P1_MIN = 0.0;
    constexpr double P1_MAX = 50.0;


}