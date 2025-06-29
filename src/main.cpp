#include "main.h"


pros::MotorGroup leftMotors({1, 2, 3}, pros::MotorCartridge::blue);
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorCartridge::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.5,
    lemlib::Omniwheel::NEW_325, 450, 8);

pros::Imu imu(7);
pros::Rotation verticalTrackingWheelRotation(8);
lemlib::TrackingWheel verticalTrackingWheel(&verticalTrackingWheelRotation, 2, 0);

lemlib::OdomSensors sensors(&verticalTrackingWheel,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu // inertial sensor
);


// lateral PID controller
lemlib::ControllerSettings linearSettings(10, // proportional gain (kP)
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
lemlib::ControllerSettings angularSettings(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


Distance frontDistanceSensor(9, {0, 0, 0});
Distance rightDistanceSensor(10, {0, 0, -M_PI_2});
Distance leftDistanceSensor(11, {0, 0, M_PI_2});
std::vector pfSensors = {&frontDistanceSensor, &rightDistanceSensor, &leftDistanceSensor};


ParticleFilterChassis<1000> chassis(drivetrain, linearSettings, angularSettings,
    sensors, pfSensors);


/**
 * @brief Initializes the robot's motors and sensors.
 *
 * This function sets motor brake modes and encoder units during initialization.
 */
void initialize() {
}


/**
 * @brief Runs the operator control loop.
 *
 * This function continuously reads controller inputs, processes them,
 * and applies movement commands to the robot's motors.
 */
void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true) {
        int32_t leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        auto [throttle, turn] = driveCurve({leftY, rightX});
        chassis.tank(throttle + turn, throttle - turn, true);

        pros::delay(10);
    }
}

/**
 * @brief Runs when the robot is disabled in competition mode.
 */
void disabled() {}

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
void autonomous() {}
