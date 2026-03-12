#pragma once

#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/distance.hpp"
#include "pros/adi.hpp"
#include "Eigen/Geometry"
#include <memory>
#include <utility>
#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"

namespace subsystems {

namespace intake {
    enum class GoalType {
        LOW_GOAL,
        MEDIUM_GOAL,
        MEDIUM_GOAL_SLOW,
        MEDIUM_GOAL_SLOWISH,
        HOLD_BALLS,
        LONG_GOAL,
        LONG_GOAL_SLOW,
        NONE
    };

    void run(GoalType goal);
    void iterate(GoalType goal);
    void stop();
}


namespace matchload {
    void extend();
    void retract();
    void toggle();
}

namespace wing {
    void extend();
    void retract();
    void toggle();
}

namespace midGoalDescore {
    void extend();
    void retract();
    void toggle();
}



namespace localization {
    enum class Wall {
        LEFT_X,
        RIGHT_X,
        TOP_Y,
        BOTTOM_Y
    };

    void leftDistanceReset(lemlib::Chassis& chassis, Wall wall);
}
};
