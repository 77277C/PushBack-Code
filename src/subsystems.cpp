#include "subsystems.hpp"


pros::MotorGroup lowerIntakeMotor({-19, 20});
pros::Motor scoringIntakeMotor(-16, pros::MotorGears::rpm_200);
pros::Optical intakeOpticalSensor(14);

pros::adi::Pneumatics matchloadPiston('H',false);
pros::adi::Pneumatics holdBallsPiston('F', false);
pros::adi::Pneumatics wingPiston('G', false);
pros::adi::Pneumatics midGoalDescorePiston('E', false);

std::shared_ptr<pros::Task> intakingTaskPtr = nullptr;
subsystems::intake::AllianceColor currentAllianceColor = subsystems::intake::AllianceColor::DISABLED;

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
    intakeOpticalSensor.set_integration_time(10);
    intakeOpticalSensor.set_led_pwm(100);

    static bool isSorting = false;
    static int32_t startedSorting = pros::millis();

    static bool antiJam = false;
    static std::uint32_t startJam = 0;
    static GoalType previousMode = GoalType::NONE;
    static std::uint32_t midGoalStart = 0;

    if (previousMode != GoalType::MEDIUM_GOAL && goalType == GoalType::MEDIUM_GOAL) {
        midGoalStart = pros::millis();
    }
    previousMode = goalType;

    switch (goalType) {
        case GoalType::NONE:
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
            if (pros::millis() - midGoalStart < 100) {
                lowerIntakeMotor.move(-127);
            } else {
                lowerIntakeMotor.move(127);
            }
            scoringIntakeMotor.move(-127);
            break;
        case GoalType::HOLD_BALLS:
            holdBallsPiston.retract();
            lowerIntakeMotor.move(127);
            scoringIntakeMotor.move(127); //-60 to -20
            break;
        case GoalType::LONG_GOAL_END:
            holdBallsPiston.extend();
            lowerIntakeMotor.brake();
            scoringIntakeMotor.move(127);
        break;
        case GoalType::LONG_GOAL:
            holdBallsPiston.extend();
            lowerIntakeMotor.move(127);
            double hue = intakeOpticalSensor.get_hue();
            if (
                    intakeOpticalSensor.get_proximity() > 200 &&
                    ((currentAllianceColor == AllianceColor::RED && hue > 180 && hue < 260) ||
                    (currentAllianceColor == AllianceColor::BLUE && (hue > 330 || hue < 30)))
                ) {
                isSorting = true;
                startedSorting = pros::millis();
            }

            if (pros::millis() - startedSorting > 0) {
                isSorting = false;
            }

            if (isSorting) {
                scoringIntakeMotor.move(-127);
            } else {
                scoringIntakeMotor.move(127);
            }
            break;
    }
}

void subsystems::intake::stop() {
    if (intakingTaskPtr) {
        intakingTaskPtr->remove();
        intakingTaskPtr = nullptr;
    }

    iterate(GoalType::NONE);
}

std::string subsystems::intake::getAllianceColorAsString() {
    if (currentAllianceColor == AllianceColor::RED) {
        return "R";
    }
    if (currentAllianceColor == AllianceColor::BLUE) {
        return "B";
    }
    if (currentAllianceColor == AllianceColor::DISABLED) {
        return "D";
    }
}

void subsystems::intake::setAllianceColor(AllianceColor color) {
    currentAllianceColor = color;
}

void subsystems::intake::toggleAllianceColor() {
    if (currentAllianceColor == AllianceColor::RED || currentAllianceColor == AllianceColor::DISABLED) {
        setAllianceColor(AllianceColor::BLUE);
    } else {
        setAllianceColor(AllianceColor::RED);
    }
}

void subsystems::intake::disableColorSort() {
    setAllianceColor(AllianceColor::DISABLED);
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

