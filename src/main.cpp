#include "main.h"
#include "autons.hpp"


pros::MotorGroup leftMotors({-7, -5, -4}, pros::MotorCartridge::blue);
pros::MotorGroup rightMotors({8, 9, 10}, pros::MotorCartridge::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5,
    lemlib::Omniwheel::NEW_325, 450, 8);

pros::Imu imu(12);

pros::Rotation horizontalTrackingWheelRotation(-17);
lemlib::TrackingWheel verticalTrackingWheel(&horizontalTrackingWheelRotation, 2.0, 0);


lemlib::OdomSensors sensors(&verticalTrackingWheel,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu // inertial sensor
);


// lateral PID controller
lemlib::ControllerSettings linearSettings(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularSettings(1.65, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              2, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
/*
Distance frontDistanceSensor(11, {0, 0, 0});
Distance rightDistanceSensor(12, {0, 0, -M_PI_2});
Distance leftDistanceSensor(13, {0, 0, M_PI_2});
*/
std::vector<Distance*> pfSensors = {};


localization::UpgradedChassis<PARTICLES> chassis(drivetrain, linearSettings, angularSettings,
    sensors, pfSensors);


rd::Selector selector({
    {"13 AWP", thirteen_awp},
    {"14 AWP", fourteen_awp},
    {"Right 9 w/ wing", right_9_w_wing},
    {"Right 9 w/o wing", right_9_no_wing},
    {"Left 4+3 w/ wing", left_43_w_wing},
    {"Left 4+3 w/o wing", left_43_no_wing},
    {"Left 7", seven_ball_left},
    {"Right 7", seven_ball_right},
    {"Left ML Rush (awp align)", left_ml_rush},
    {"Right ML Rush (awp align)", right_ml_rush},

    //{"RightElim", rightElim},
    //{"Tune", tunePid},
    {"Wing balls", wing_balls},
    {"Skills 96", skills_96},
    {"Skills 98", skills_98},
    {"Skills 98 Start", skills98Start},
    {"Skills 100 End", skills100End},

    
    //"Skills 98", skills_98},
    {"SkillsOne", skillsOne}, 
    //{"SkillsTwo", skillsTwo}, 
    {"SkillsThree", skillsThree}, 
    {"SkillsFour", skillsFour}, 
    {"SkillsFive", skillsFive}
});

rd::Console console;

pros::Controller controller(pros::E_CONTROLLER_MASTER);


/**
 * @brief Initializes the robot's motors and sensors.
 *
 * This function sets motor brake modes and encoder units during initialization.
 */
void initialize() {
    printf("Starting logging");
    chassis.calibrate();
    
    
    pros::Task{[&]() {
        while (true) {     
            lemlib::Pose pose = chassis.getPose(false, false);
            controller.print(0, 0, "%s %.1f %.1f %.1f",
            subsystems::intake::getAllianceColorAsString().c_str(), pose.x, pose.y, pose.theta);
            
            for (double temp : leftMotors.get_temperature_all()) {
                console.printf("%.0f ", temp);
            }
            for (double temp : rightMotors.get_temperature_all()) {
                console.printf("%.0f ", temp);
            }
            for (double temp : lowerIntakeMotor.get_temperature_all()) {
                console.printf("%.0f ", temp);
            }

            console.printf("%.0f", scoringIntakeMotor.get_temperature());
            console.println("");
            //controller.print(0, 0, "%.2f", lowerIntakeMotor.get_efficiency());
            pros::delay(100);
        }
    }};
    
}


/**
 * @brief Runs the operator control loop.
 *
 * This function continuously reads controller inputs, processes them,
 * and applies movement commands to the robot's motors.
 */
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    subsystems::intake::stop();
    while (true) {
        // drivetrain
        int32_t leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        auto [throttle, turn] = driveCurvePilon({leftY, rightX});
        chassis.tank(throttle + turn, throttle - turn, true);

        subsystems::intake::GoalType goal = subsystems::intake::GoalType::NONE;
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            goal = subsystems::intake::GoalType::LONG_GOAL;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            goal = subsystems::intake::GoalType::MEDIUM_GOAL;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            goal = subsystems::intake::GoalType::HOLD_BALLS;
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            goal = subsystems::intake::GoalType::LOW_GOAL;
        }
        subsystems::intake::iterate(goal);

        // intake colorsort
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            subsystems::intake::toggleAllianceColor();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            subsystems::intake::disableColorSort();
        }


        // pneumatics
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            subsystems::matchload::toggle();
            if (subsystems::matchload::is_extended()) {
                subsystems::midGoalDescore::retract();
            }
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            subsystems::wing::toggle();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            subsystems::midGoalDescore::toggle();
            if (subsystems::midGoalDescore::is_extended()) {
                subsystems::matchload::retract();
            }
        }
        
        // autos
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            autonomous();
            //right_9_first_part();
            //right_9_w_wing();
            //left_43_w_wing();
            //left_ml_rush();
            //left_43_w_wing();
            //thirteen_awp();
            //awp_part_two();
            // skills_1();
            //left_9_wing();
            //left_9_hold();
            //solo_awp();
            //skillsStart();
            //rightElim();
            //skillsTwo();
            //skillsFour();
            //skillsFive();
            // fourteen_awp();
            subsystems::intake::stop();
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        }

        pros::delay(10);
    }
}

/**
 * @brief Runs when the robot is disabled in competition mode.
 */
void disabled() {
        subsystems::matchload::retract();

}

/**

 * @brief Runs before autonomous mode in competition settings.
 *
 * This function is useful for setting up autonomous routines, such as
 * selecting different strategies using an LCD screen.
 */
void competition_initialize() {}

/**
 * @brief Runs the autonomous routine.
 *
 * This function is executed when the robot enters autonomous mode in competition.
 */
void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    selector.run_auton();
    //itCouldWork();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}
