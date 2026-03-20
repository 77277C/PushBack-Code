#include "subsystems.hpp"


pros::MotorGroup lowerIntakeMotor({1}, pros::MotorGears::rpm_600);
pros::Motor scoringupperIntakeMotor(-18, pros::MotorGears::rpm_600);

pros::adi::Pneumatics matchloadPiston('H',false);
pros::adi::Pneumatics holdBallsPiston('F', false, false);
pros::adi::Pneumatics wingPiston('G', false, false);
pros::adi::Pneumatics midGoalDescorePiston('E', false, false);

std::shared_ptr<pros::Task> intakingTaskPtr = nullptr;

void subsystems::intake::run(GoalType goalType) {
    if (intakingTaskPtr) {
        stop();
    }

    intakingTaskPtr = std::make_shared<pros::Task>([goalType]() {
        while (true) {
            iterate(goalType);
            pros::delay(10);
        }
    });
}

void subsystems::intake::iterate(GoalType goalType) {
    static bool antiJam = false;
    static std::uint32_t startJam = 0;
    static GoalType previousMode = GoalType::NONE;
    static std::uint32_t changedModeAt = 0;

    if (goalType != previousMode) {
        changedModeAt = pros::millis();
    }
    previousMode = goalType;

    if (pros::millis() - startJam > 100) {
        antiJam = false;
    }
    if (antiJam) {
        lowerIntakeMotor.move(-127);
        scoringIntakeMotor.move(-127);
        return;
    }

    switch (goalType) {
        case GoalType::NONE:
            holdBallsPiston.retract();
            lowerIntakeMotor.brake();
            scoringIntakeMotor.brake();
            return;
        case GoalType::LOW_GOAL:
            holdBallsPiston.retract();
            lowerIntakeMotor.move(-127);
            scoringIntakeMotor.move(-127);
            return;
        case GoalType::MEDIUM_GOAL:
            holdBallsPiston.retract();
            lowerIntakeMotor.move(127);
            scoringIntakeMotor.move(-127);
            break;
        case GoalType::HOLD_BALLS:
            holdBallsPiston.retract();
            lowerIntakeMotor.move(127);
            scoringupperIntakeMotor.move(127); //-60 to -20
            break;
        case GoalType::LONG_GOAL:
            holdBallsPiston.extend();
            lowerIntakeMotor.move(127);
            scoringupperIntakeMotor.move(127);
            break;
    }

    if (pros::millis() - changedModeAt > 250 && (lowerIntakeMotor.get_efficiency() < 0.1 || scoringupperIntakeMotor.get_efficiency() < 0.1)) {
        antiJam = true;
        startJam = pros::millis();
    }
}

void subsystems::intake::stop() {
    if (intakingTaskPtr) {
        intakingTaskPtr->remove();
        intakingTaskPtr = nullptr;
    }

    iterate(GoalType::NONE);
}

void subsystems::matchload::extend() {
    matchloadPiston.extend();
}

void subsystems::matchload::retract() {
    matchloadPiston.retract();
}

void subsystems::matchload::toggle() {
    matchloadPiston.toggle();
}

bool subsystems::matchload::is_extended() {
    return matchloadPiston.is_extended();
}


void subsystems::wing::extend() {
    wingPiston.extend();
}

void subsystems::wing::retract() {
    wingPiston.retract();
}

void subsystems::wing::toggle() {
    wingPiston.toggle();
}

void subsystems::midGoalDescore::extend() {
    midGoalDescorePiston.extend();
}

void subsystems::midGoalDescore::retract() {
    midGoalDescorePiston.retract();
}

void subsystems::midGoalDescore::toggle() {
    midGoalDescorePiston.toggle();
}

bool subsystems::midGoalDescore::is_extended() {
    return midGoalDescorePiston.is_extended();
}

void subsystems::localization::leftDistanceReset(lemlib::Chassis& chassis, Wall wall) {
    static std::vector<pros::Distance> distanceSensors = pros::Distance::get_all_devices();
    if (distanceSensors.empty()) return;

    static pros::Distance leftDistanceSensor = distanceSensors[0];
    static double xOffset = 5;
    static double yOffset = 1;

    double distance = leftDistanceSensor.get() / 25.4;
    if (distance == 9999) return;

    double effectiveDistance = distance + xOffset;

    lemlib::Pose pose = chassis.getPose();
    double heading = lemlib::degToRad(pose.theta);

    switch (wall) {
    case Wall::LEFT_X:
            effectiveDistance = effectiveDistance * std::cos(heading) + yOffset * std::sin(heading);
            chassis.setPose(-72 + effectiveDistance, pose.y, pose.theta);
            break;
        case Wall::RIGHT_X:
            effectiveDistance = effectiveDistance * std::cos(heading - M_PI) + yOffset * std::sin(heading - M_PI);
            chassis.setPose(72 - effectiveDistance, pose.y, pose.theta);
            break;
        case Wall::TOP_Y:
            effectiveDistance = effectiveDistance * std::cos(heading - M_PI_2) + yOffset * std::sin(heading - M_PI_2);
            chassis.setPose(pose.x, 72 - effectiveDistance, pose.theta);
            break;
        case Wall::BOTTOM_Y:
            effectiveDistance = effectiveDistance * std::cos(heading - 3*M_PI_2) + yOffset * std::sin(heading - 3*M_PI_2);
            chassis.setPose(pose.x, -72 + effectiveDistance, pose.theta);
            break;
    }
}

