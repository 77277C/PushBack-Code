#include "autons.hpp"

void example() {/*
    // long goal
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    pros::delay(1000);
    subsystems::intake::stop();

    // medium goal
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
    pros::delay(1000);
    subsystems::intake::stop();

    // low goal
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(1000);
    subsystems::intake::stop();
    // open stopes scoring
    subsystems::hood::open();
    subsystems::hood::close();

    subsystems::matchload::extend();
    subsystems::matchload::retract();
    */
}

void fourteen_awp(){
    // 14 balls, take two preloads, matchload 3, score long, 3 + 3 in middle, score mid, matchload 3, score long
    // Tested: works
    // side of park zone, front to out

    subsystems::wing::extend();
    chassis.setPose(-45.5, 0, 0);
    chassis.moveToPoint(-45.5, 6, 1000, {. minSpeed = 30, .earlyExitRange = 1});
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    chassis.moveToPoint(-45, -46.5, 2500, {.forwards = false, .maxSpeed = 60, .minSpeed = 5, .earlyExitRange = 1});
    chassis.waitUntil(5);
    chassis.cancelMotion();


    chassis.moveToPoint(-45, -43.5, 2500, {.forwards = false, .maxSpeed = 100, .minSpeed = 5, .earlyExitRange = 1});
    subsystems::matchload::extend();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.turnToPoint(-62, -47.5, 2500, {.maxSpeed = 85, .minSpeed = 5,.earlyExitRange = 2});


    // shake in the matchloaer
    chassis.moveToPoint(-64, -47.5, 600, {.maxSpeed = 55, .minSpeed = 45, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.tank(40,40);
    pros::delay(330);
    chassis.tank(0,0);


    chassis.moveToPoint(-30, -47.75, 1900, {.forwards = false, .maxSpeed = 95, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntil(15); //12 to 15
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.cancelMotion();

    subsystems::matchload::retract();
    
    chassis.tank(-100, -100);

    pros::delay(1750); // 1700 to 1750

    chassis.setPose(-31, -41.5, chassis.getPose().theta);
    pros::delay(50);
    //chassis.swingToPoint(-28, -15 , lemlib::DriveSide::RIGHT, 800, {.maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1});
    chassis.moveToPoint(-33, -39, 400, {.minSpeed = 75, .earlyExitRange = 1});
    chassis.swingToPoint(-28, -15 , lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1});

    //chassis.turnToPoint(-28, -15, 1000, {.maxSpeed = 95, .minSpeed = 50, .earlyExitRange = 3});
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-27, -15, 1200, {.maxSpeed = 70, .minSpeed = 55, .earlyExitRange = 1});
    chassis.waitUntil(10);
    //subsystems::matchload::extend();
    
    chassis.moveToPoint(-27.5, 12, 1200, {.maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 1});
    chassis.waitUntil(10);
    chassis.cancelMotion();
    //subsystems::matchload::retract();
    
    chassis.moveToPoint(-26.5, 12, 1200, {.maxSpeed = 95, .minSpeed = 50, .earlyExitRange = 1});

    
    chassis.moveToPoint(-27, 26, 1200, {.maxSpeed = 75, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntil(4);
    subsystems::matchload::extend();
    
    chassis.moveToPose(-13, 12, 315, 1500, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 4});
    chassis.waitUntil(11);

    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(50);


    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);

    chassis.waitUntilDone();
    pros::delay(600);
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    subsystems::matchload::retract();

    pros::delay(50);
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    chassis.moveToPoint(-43, 48.5, 2000,  {.maxSpeed = 105, .minSpeed = 25, .earlyExitRange = 1});
    chassis.turnToPoint(-72, 52, 800, {.maxSpeed = 75, .minSpeed = 40, .earlyExitRange = 6});
    subsystems::matchload::extend();

    chassis.moveToPoint(-71.75, 52, 1000, {.maxSpeed = 55, .minSpeed = 40, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.tank(50,50);
    pros::delay(350);
    chassis.tank(0,0);  
    
    chassis.moveToPoint(-10, 54, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, .earlyExitRange = 1});
    chassis.waitUntil(15);
    chassis.cancelMotion();

    chassis.moveToPoint(-10, 54, 200, {.forwards = false, .maxSpeed = 70, .minSpeed = 25, .earlyExitRange = 1}); //52.35 to 53
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.waitUntilDone();
    chassis.tank(-127, -127);

    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    pros::delay(3000);
}

void thirteen_awp() {
    // 13 balls, take one preloads, matchload 3, score long, 3 + 3 in middle, score mid, matchload 3, score long
    // Tested: works
    // location: normal, but forward a bit
    subsystems::wing::extend();
    chassis.setPose(-45.5, -17.5, 180);
    chassis.moveToPoint(-45, -46.8, 2500, {.maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 1});
    subsystems::matchload::extend();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.turnToPoint(-62, -49, 2500, {.maxSpeed = 85, .minSpeed = 5,.earlyExitRange = 2});


    // shake in the matchloaer
    chassis.moveToPoint(-64, -48.5, 600, {.maxSpeed = 55, .minSpeed = 55, .earlyExitRange = 1}); //-67.6 to -67.2 and -50 to -49.5
    chassis.waitUntilDone();
    chassis.tank(40,40);
    pros::delay(350);
    chassis.tank(0,0);


    chassis.moveToPoint(-33, -48.5, 1900, {.forwards = false, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntil(15);
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(125);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.cancelMotion();

    subsystems::matchload::retract();


    
    chassis.tank(-100, -100);

    pros::delay(1200);

    chassis.setPose(-31, -41.5, chassis.getPose().theta);
    pros::delay(50);
    //chassis.swingToPoint(-23, -27 , lemlib::DriveSide::LEFT, 800, {.maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1});
    chassis.moveToPoint(-39, -39, 1000);
    chassis.turnToPoint(-28, -15, 1000, {.maxSpeed = 95, .minSpeed = 50, .earlyExitRange = 3});
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-28, -15, 1200, {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 1});
    chassis.waitUntil(10);
    subsystems::matchload::extend();
    
    chassis.moveToPoint(-27, 12, 1200, {.maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 1});
    chassis.waitUntil(10);
    chassis.cancelMotion();
    subsystems::matchload::retract();
    
    chassis.moveToPoint(-26.5, 12, 1200, {.maxSpeed = 95, .minSpeed = 50, .earlyExitRange = 1});

    
    chassis.moveToPoint(-27, 26, 1200, {.maxSpeed = 55, .minSpeed = 30, .earlyExitRange = 1});
    chassis.waitUntil(4);
    subsystems::matchload::extend();
    
    chassis.moveToPose(-13, 12, 315, 1500, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 4});
    chassis.waitUntil(10.5);
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(100);

    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);

    chassis.waitUntilDone();
    pros::delay(600);
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    subsystems::matchload::retract();

    pros::delay(50);
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);


    
    
    chassis.moveToPoint(-43, 47, 2000,  {.maxSpeed = 95, .minSpeed = 20, .earlyExitRange = 1});
    chassis.turnToPoint(-70, 49.55, 800, {.maxSpeed = 75, .minSpeed = 40, .earlyExitRange = 6});
    subsystems::matchload::extend();

    chassis.moveToPoint(-71.5, 50, 1000, {.maxSpeed = 55, .minSpeed = 40, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.tank(-7, -7);
    pros::delay(100);
    chassis.tank(50,50);
    pros::delay(325);
    chassis.tank(-25,-25);
    pros::delay(50);
    chassis.tank(0,0);  
    
    chassis.moveToPoint(-10, 54, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, .earlyExitRange = 1});
    chassis.waitUntil(14);
    chassis.cancelMotion();
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);

    chassis.moveToPoint(-10, 54, 200, {.forwards = false, .maxSpeed = 70, .minSpeed = 25, .earlyExitRange = 1}); //52.35 to 53
    chassis.waitUntilDone();
    chassis.tank(60, 60);
    pros::delay(50);
    chassis.tank(-127, -127);

    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    pros::delay(3000);
    
}
/*
    void useless_left() {
        chassis.setPose(-42, 12.25 ,90 );
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
        chassis.moveToPoint(-17, 19, 4000, {.maxSpeed = 85, .minSpeed = 60});
        chassis.waitUntil(15);
        subsystems::matchload::extend();
        chassis.moveToPose(-1.25, 3.25, 315, 2250, {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
        chassis.waitUntil(13);
        subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);

        pros::delay(100);
        subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
        chassis.waitUntilDone();
        pros::delay(1500);    
        subsystems::matchload::retract();

        
        // subsystems::intake::stop();
        // chassis.moveToPoint(-13.5, 16, 2250, {.maxSpeed = 70, .minSpeed = 30});
        // subsystems::matchload::retract();

        // chassis.waitUntil(10);

        // subsystems::midGoalDescore::extend();
        // chassis.waitUntilDone();
        // pros::delay(300);
        // chassis.moveToPoint(-6, 8.5,1550, {.forwards = false, .maxSpeed = 70, .minSpeed = 30});


        
        chassis.moveToPoint(-35, 41, 2000,  {.maxSpeed = 80, .minSpeed = 40});
        //subsystems::midGoalDescore::retract();

        chassis.waitUntil(15);
        subsystems::matchload::extend();




        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
        chassis.turnToPoint(-71, 43.5, 1100, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 3});

        chassis.moveToPoint(-75.5, 43.5, 1200, {.maxSpeed = 60, .minSpeed = 45, .earlyExitRange = 1});
        chassis.waitUntilDone();
        chassis.tank(-10, -10);
        pros::delay(100);
        chassis.tank(50, 50);
        pros::delay(100);



        chassis.moveToPoint(-10, 43.25, 500, {.forwards = false, .maxSpeed = 75, .minSpeed = 60, .earlyExitRange = 1});
        chassis.waitUntil(15);
        chassis.cancelMotion();
        chassis.moveToPoint(-10, 43.25, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 55, .earlyExitRange = 1});
        chassis.waitUntil(11);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
        subsystems::matchload::retract();

        chassis.waitUntilDone();
        chassis.tank(-80, -80);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
        pros::delay(600);
        chassis.tank(0,0);
        pros::delay(200);
        tunePid();

        // chassis.setPose(-15, -42, 270);
        // pros::delay(100);
        

        // chassis.moveToPoint(-30, -50.55, 1300, {.maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 2});
        // chassis.moveToPose(-20, -51.55, 270, 3500, {.forwards = false, .maxSpeed = 75, .minSpeed = 30, .earlyExitRange = 1});

        
        // chassis.moveToPoint(-5, -51.55, 4000, {.forwards = false, .maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 1});
        // subsystems::wing::retract();
        // subsystems::matchload::retract();
        // chassis.waitUntilDone();
        // chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1000);
        // chassis.tank(30,0);
        // pros::delay(10000000);

    }
*/

void left_ml_rush() {
    // 4 balls, one preload + 3 matchloader, score long goal, long goal wing
    // Tested: works
    // location: normal 13 ball awp but on opp side
    
    subsystems::wing::extend();
    chassis.setPose(-45.5, 17.5, 0);
    chassis.moveToPoint(-45, 46.8, 2500, {.maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 1});
    subsystems::matchload::extend();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.turnToPoint(-62, 49, 2500, {.maxSpeed = 85, .minSpeed = 5,.earlyExitRange = 2});


    // shake in the matchloaer
    chassis.moveToPoint(-64, 48.5, 600, {.maxSpeed = 55, .minSpeed = 55, .earlyExitRange = 1}); //-67.6 to -67.2 and -50 to -49.5
    chassis.waitUntilDone();
    chassis.tank(40,40);
    pros::delay(350);
    chassis.tank(0,0);


    chassis.moveToPoint(-33, 48.5, 1900, {.forwards = false, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntil(15);
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(125);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.cancelMotion();

    subsystems::matchload::retract();


    
    chassis.tank(-100, -100);

    pros::delay(1200);

    wing_balls();
}

void right_ml_rush() {
    // 4 balls, one preload + 3 matchloader, score long goal, long goal wing
    // Tested: works
    // location: normal 13 ball awp(but forward a bit)

    subsystems::wing::extend();
    chassis.setPose(-45.5, -17.5, 180);
    chassis.moveToPoint(-45, -46.8, 2500, {.maxSpeed = 90, .minSpeed = 5, .earlyExitRange = 1});
    subsystems::matchload::extend();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.turnToPoint(-62, -49, 2500, {.maxSpeed = 85, .minSpeed = 5,.earlyExitRange = 2});


    // shake in the matchloaer
    chassis.moveToPoint(-64, -48.5, 600, {.maxSpeed = 55, .minSpeed = 55, .earlyExitRange = 1}); //-67.6 to -67.2 and -50 to -49.5
    chassis.waitUntilDone();
    chassis.tank(40,40);
    pros::delay(350);
    chassis.tank(0,0);


    chassis.moveToPoint(-33, -48.5, 1900, {.forwards = false, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntil(15);
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(125);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.cancelMotion();

    subsystems::matchload::retract();


    
    chassis.tank(-100, -100);

    pros::delay(1200);

    wing_balls();
}

void left_7_w_wing() {
    // 7 balls, preload + 3 mid, score in long goal, 3 matchloader, score mid, descore mid, long goal wing
    // Tested: works
    // location: normal left

    chassis.setPose(-42, 12.25 ,90 );
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-17, 19, 4000, {.maxSpeed = 85, .minSpeed = 60});
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    chassis.waitUntilDone();
    //chassis.moveToPose(-40, 43, 135, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.moveToPoint(-33, 42, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});

    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.moveToPoint(-16, 42, 1800, {.forwards = false, .maxSpeed = 70}); //41.5 to 42 and 3000 to 1700
    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    chassis.tank(60, 60);
    pros::delay(60);
    chassis.tank(-127, -127);
    pros::delay(1300);
    chassis.setPose(-25, 48, chassis.getPose().theta);
    subsystems::intake::stop();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-73.5, 47, 1100, {.maxSpeed = 53, .minSpeed = 42, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40);
    pros::delay(500); // 600 to 450
    chassis.tank(0, 0);
    chassis.moveToPoint(-50, 47, 250, {.forwards = false});
    chassis.moveToPose(-7.5, 9, 315, 2500, {.forwards = false});
    chassis.waitUntil(47); // 50 to 40
    subsystems::matchload::retract();
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);

    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
    pros::delay(1000); // 1500 to 1000
    subsystems::intake::stop();

    chassis.moveToPoint(-15.5, 17.5, 1000, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntil(5);
    subsystems::midGoalDescore::extend();
    chassis.waitUntilDone();


    chassis.moveToPose(-6.25, 8.25, 315, 3000, {.forwards = false, .maxSpeed = 70, .minSpeed = 65});
    chassis.waitUntilDone();
    pros::delay(200);

    chassis.moveToPoint(-29, 35.75, 2000); //35.25 to 35.5
    chassis.waitUntilDone();

    subsystems::midGoalDescore::retract();

    chassis.turnToHeading(89, 1000, {.minSpeed = 30, .earlyExitRange = 2}); //91 to 90
    chassis.waitUntilDone();

    chassis.moveToPoint(-7, 37.25, 5000);
    chassis.waitUntilDone();
    chassis.tank(30, -20);
    pros::delay(100000);
}

void left_9_wing() {
    // 9 balls, preload + 3 mid + 2 under long, score in long goal, 3 matchloader, score mid, descore mid, long goal wing
    // Tested: works
    // location: normal left

    chassis.setPose(-42, 12.25 ,90 );
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-17, 19, 4000, {.maxSpeed = 85, .minSpeed = 60});
    chassis.waitUntil(15);
    subsystems::matchload::extend();

    chassis.waitUntilDone();
    subsystems::matchload::retract();
    chassis.moveToPoint(-1.5, 39, 2000, {.maxSpeed = 60, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    chassis.waitUntilDone();
    pros::delay(400);

    chassis.moveToPoint(-10, 10, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.waitUntil(10);
    chassis.cancelMotion();
    chassis.tank(5,5);
    pros::delay(100);
    chassis.tank(0,0);

    //chassis.moveToPoint(-22, 25, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});


    //chassis.moveToPose(-40, 43, 135, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.moveToPoint(-38, 45, 4000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1}); //46.5 to 47
    chassis.waitUntil(20);
    chassis.cancelMotion();
    chassis.moveToPoint(-38, 47, 1000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1}); //46.5 to 47

    chassis.waitUntilDone();
    //chassis.turnToHeading(270, 1000, {.minSpeed = 70, .earlyExitRange = 3});
    //chassis.waitUntilDone();
    chassis.moveToPoint(-16, 47.6, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(25);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.waitUntilDone();

    //chassis.tank(60, 60);
    //pros::delay(60);
    chassis.tank(-127, -127);
    pros::delay(1350);
    chassis.setPose(-25, 48, chassis.getPose().theta);
    subsystems::intake::stop();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-73.5, 47, 1100, {.maxSpeed = 53, .minSpeed = 42, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40);
    pros::delay(500); // 600 to 450
    chassis.tank(0, 0);
    chassis.moveToPoint(-50, 47, 300, {.forwards = false});
    chassis.moveToPose(-7.5, 9, 315, 2500, {.forwards = false});
    chassis.waitUntil(50); // 50 to 40
    subsystems::matchload::retract();
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);

    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
    pros::delay(1250); // 1500 to 1000
    subsystems::intake::stop();

    
    chassis.moveToPoint(-15.5, 17.5, 1000, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntil(5);
    subsystems::midGoalDescore::extend();


    chassis.moveToPose(-7.5, 8.25, 315, 3000, {.forwards = false, .maxSpeed = 70, .minSpeed = 65}); //-6.25 to -7.5
    chassis.waitUntilDone();
    
    chassis.moveToPoint(-29, 35.75, 2000); //35.25 to 35.5
    chassis.waitUntilDone();

    subsystems::midGoalDescore::retract();

    chassis.turnToHeading(88, 1000, {.minSpeed = 30, .earlyExitRange = 2}); //91 to 90
    chassis.waitUntilDone();

    chassis.moveToPoint(-7, 37.25, 5000);
    chassis.waitUntilDone();
    chassis.tank(30, -20);
    pros::delay(100000);
}

void left_9_no_wing() {
    // 9 balls, preload + 3 mid + 2 under long, score in long goal, 3 matchloader, score mid, wait descore mid
    // Tested: 
    // location: normal left


    chassis.setPose(-42, 12.25 ,90 );
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-17, 19, 4000, {.maxSpeed = 85, .minSpeed = 60});
    chassis.waitUntil(15);
    subsystems::matchload::extend();

    chassis.waitUntilDone();
    subsystems::matchload::retract();
    chassis.moveToPoint(-1.5, 39, 2000, {.maxSpeed = 60, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    chassis.waitUntilDone();
    pros::delay(400);

    chassis.moveToPoint(-10, 10, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.waitUntil(10);
    chassis.cancelMotion();
    chassis.tank(5,5);
    pros::delay(100);
    chassis.tank(0,0);

    //chassis.moveToPoint(-22, 25, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});


    //chassis.moveToPose(-40, 43, 135, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.moveToPoint(-38, 44, 4000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1}); //46.5 to 47
    chassis.waitUntil(20);
    chassis.cancelMotion();
    chassis.moveToPoint(-38, 47.5, 1000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1}); //46.5 to 47

    chassis.waitUntilDone();
    //chassis.turnToHeading(270, 1000, {.minSpeed = 70, .earlyExitRange = 3});
    //chassis.waitUntilDone();
    chassis.moveToPoint(-16, 47.6, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(25);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.waitUntilDone();

    //chassis.tank(60, 60);
    //pros::delay(60);
    chassis.tank(-127, -127);
    pros::delay(1350);
    chassis.setPose(-25, 48, chassis.getPose().theta);
    subsystems::intake::stop();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-73.5, 47, 1100, {.maxSpeed = 53, .minSpeed = 42, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40);
    pros::delay(450); // 600 to 450
    chassis.tank(0, 0);
    chassis.moveToPoint(-50, 47, 300, {.forwards = false});
    chassis.moveToPose(-7.5, 9, 315, 2500, {.forwards = false});
    chassis.waitUntil(50); // 50 to 40
    subsystems::matchload::retract();
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);

    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
    pros::delay(1250); // 1500 to 1000
    subsystems::intake::stop();

    
    chassis.moveToPoint(-15.5, 17.5, 1000, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntil(5);
    subsystems::midGoalDescore::extend();

    pros::delay(1000);

    chassis.moveToPose(-6.25, 8.25, 315, 3000, {.forwards = false, .maxSpeed = 70, .minSpeed = 65});
    chassis.waitUntilDone();

    pros::delay(400);

}

void left_9_hold() {
    // not sure
    // Tested:
    // location: normal left

    chassis.setPose(-42, 12.25 ,90 );
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-17, 19, 4000, {.maxSpeed = 95, .minSpeed = 60});
    chassis.waitUntil(15);
    subsystems::matchload::extend();

    chassis.waitUntilDone();
    subsystems::matchload::retract();
    chassis.moveToPoint(-1.5, 39, 2000, {.maxSpeed = 70, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    chassis.waitUntilDone();
    pros::delay(200);

    chassis.moveToPoint(-10, 10, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.waitUntil(10);
    chassis.cancelMotion();
    chassis.tank(5,5);
    pros::delay(100);
    chassis.tank(0,0);

    //chassis.moveToPoint(-22, 25, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});


    //chassis.moveToPose(-40, 43, 135, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.moveToPoint(-38, 47.5, 4000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});

    chassis.waitUntilDone();
    //chassis.turnToHeading(270, 1000, {.minSpeed = 70, .earlyExitRange = 3});
    //chassis.waitUntilDone();
    
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.turnToHeading(270, 1000, {.minSpeed = 100, .earlyExitRange = 2});
    chassis.moveToPoint(-73.5, 47, 1100, {.maxSpeed = 65, .minSpeed = 50, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    pros::delay(100);
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    pros::delay(500); // 600 to 450
    chassis.tank(0, 0);

    chassis.moveToPoint(-16, 47.6, 3000, {.forwards = false, .maxSpeed = 70}); //41.5 to 42 and 3000 to 1700
    chassis.waitUntil(20);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.waitUntilDone();

    chassis.tank(-127, -127);
    pros::delay(3000);
}

void left_7_no_wing() {
    // 7 balls, preload + 3 mid, score in long goal, 3 matchloader, score mid, wait and descore mid
    // Tested: works
    // location: normal left

    chassis.setPose(-42, 12.25 ,90 );
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-17, 19, 4000, {.maxSpeed = 85, .minSpeed = 60});
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    chassis.waitUntilDone();
    chassis.moveToPose(-40, 43, 135, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000, {.minSpeed = 70, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(-18, 41.5, 3000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    chassis.tank(50, 50);
    pros::delay(50);
    chassis.tank(-127, -127);
    pros::delay(1300);
    chassis.setPose(-25, 48, chassis.getPose().theta);
    subsystems::intake::stop();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-73.5, 47, 1100, {.maxSpeed = 53, .minSpeed = 42, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.tank(40, 40);
    pros::delay(600);
    chassis.tank(0, 0);
    chassis.moveToPose(-7.5, 9, 315, 2500, {.forwards = false});
    chassis.waitUntil(10);
    subsystems::matchload::retract();
    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
    pros::delay(4000);
    subsystems::intake::stop();

    chassis.moveToPoint(-15.5, 17.5, 1000, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();

    subsystems::midGoalDescore::extend();
    pros::delay(500);
    chassis.moveToPose(-6, 8, 315, 3000, {.forwards = false, .minSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(200);
}

void left_mid_rush() {
    // 4 balls, preload + 3 mid, score in long goal, long goal wing
    // Tested:
    // location: normal left

    chassis.setPose(-42, 12.25 ,90 );
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(-17, 19, 4000, {.maxSpeed = 85, .minSpeed = 60});
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    chassis.waitUntilDone();
    chassis.moveToPose(-40, 44, 130, 2000, {.forwards = false, .maxSpeed = 110, .minSpeed = 30,  .earlyExitRange = 1}); //43 to 44 and 135 to 130
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000, {.minSpeed = 70, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(-18, 42.5, 3000, {.forwards = false, .maxSpeed = 110}); //41.5 to 42.5
    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    chassis.tank(50, 50);
    pros::delay(50);
    chassis.tank(-127, -127);
    pros::delay(1300);
    wing_balls();
}

void right() {
    // not sure
    // Tested:
    // Location: 

    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.setPose(-42, -12, 90 );
    chassis.moveToPoint(-25, -16, 4000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 1});
    chassis.waitUntil(16);
    subsystems::matchload::extend();
    chassis.waitUntilDone();
    
    pros::delay(500);

    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    chassis.moveToPoint(-42, -43.25, 2000,  {.maxSpeed = 90, .minSpeed = 20, .earlyExitRange = 1});
    chassis.waitUntil(5);
    subsystems::matchload::retract();
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    
    chassis.turnToPoint(-72, -42.5, 1100, {.maxSpeed = 70, .minSpeed = 40, .earlyExitRange = 3});

    chassis.moveToPoint(-73.5, -42.5, 1100, {.maxSpeed = 53, .minSpeed = 42, .earlyExitRange = 2});
    


    chassis.waitUntilDone();

    chassis.tank(10, 10);
    pros::delay(140); //150 to 140

    chassis.moveToPoint(-10, -42, 500, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 1}); //-42.5 to -42
    chassis.waitUntil(15);

    chassis.cancelMotion();
    chassis.moveToPoint(-10, -42, 1000, {.forwards = false, .maxSpeed = 40, .minSpeed = 25, .earlyExitRange = 1}); //-42.5 to -42
    chassis.waitUntil(10);

    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(150);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    chassis.waitUntilDone();
    subsystems::matchload::retract();

    pros::delay(1800);

    subsystems::matchload::retract();

    wing_balls();
    /*
        chassis.setPose(-15, -42, 270);
        pros::delay(100);

        
        chassis.moveToPoint(-30, -50, 1300, {.maxSpeed = 100, .minSpeed = 40, .earlyExitRange = 2});
        chassis.moveToPose(-20, -51.5, 270, 3500, {.forwards = false, .maxSpeed = 100});
        subsystems::wing::retract();


        chassis.moveToPoint(-12.55, -51.5, 3500, {.forwards = false, .minSpeed = 80});
        subsystems::matchload::retract();
        chassis.waitUntilDone();
        chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1500, {.minSpeed = 100});
        chassis.tank(0,0);
        pros::delay(100000);

        
        subsystems::hood::open();
        
        chassis.moveToPoint(-35, -39, 2000,  {.forwards = false, .maxSpeed = 90, .minSpeed = 20, .earlyExitRange = 1});
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
        chassis.turnToPoint(-59, -39.5, 800, {.maxSpeed = 70, .minSpeed = 40, .earlyExitRange = 5});
        subsystems::matchload::extend();
        chassis.moveToPoint(-61.5, -39.5, 850, {.maxSpeed = 67, .minSpeed = 50, .earlyExitRange = 2});
        chassis.waitUntilDone();    
        chassis.tank(-12, -12);
        pros::delay(200);
        
        chassis.tank(30, 30);
        pros::delay(600);
        chassis.tank(-7, -7);
        pros::delay(100);
        chassis.tank(7, 7);
        pros::delay(100);
        chassis.tank(0,0);
        subsystems::hood::open();
        subsystems::intake::stop();

        chassis.moveToPoint(-10, -43, 600, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 1});
        chassis.waitUntil(15);
        chassis.cancelMotion();
        chassis.moveToPoint(-9, -43, 1600, {.forwards = false, .maxSpeed = 40, .minSpeed = 25, .earlyExitRange = 1});
        chassis.waitUntil(12);
        subsystems::hood::close();
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

        chassis.waitUntilDone();
        pros::delay(600);
        chassis.setPose(-15, -42, 270);
        pros::delay(100);
        

        chassis.moveToPoint(-30, -50, 1300, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 2});
        chassis.moveToPose(-20, -52, 270, 1000, {.forwards = false, .maxSpeed = 75});

        
        chassis.moveToPoint(-6.5, -52, 3500, {.forwards = false, .maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 1});
        subsystems::wing::retract();
        subsystems::matchload::retract();
        chassis.waitUntilDone();
        chassis.tank(0,0);
        //chassis.swingToHeading(270, DriveSide::LEFT, 500, {.minSpeed = 60});
    */
}

void rightElim(){
    // 4 balls, preload + 3 mid, score in long goal, long goal wing
    // Tested: not working
    // location

    chassis.setPose(46, 12, 270);
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    chassis.moveToPoint(22, 22, 2000, {.maxSpeed = 110, .minSpeed = 90, .earlyExitRange = 1.5});
    chassis.waitUntil(15);
    subsystems::matchload::extend();
    chassis.moveToPoint(32, 42, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 70, .earlyExitRange = 1});
    chassis.waitUntilDone();    
    chassis.moveToPoint(20, 42, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 70, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.tank(-127, -127);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    pros::delay(1200);
    wing_balls();
}

void skills_96(){
    /*
        subsystems::wing::toggle();
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
        chassis.setPose(49, -17, 270);
        chassis.moveToPoint(40, -15.5, 2000, {.maxSpeed = 75, .earlyExitRange = 0.5});
        chassis.moveToPoint(22.5, -24.5, 2000, {.maxSpeed = 55});
        chassis.waitUntil(15);
        subsystems::matchload::extend();
        chassis.waitUntilDone();
        
        pros::delay(300);
        chassis.turnToHeading(135, 1000, {.maxSpeed = 70, .minSpeed = 30});
        //score middle goal


        chassis.moveToPose(7.75, -7.75, 135, 2000, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntil(15);
        subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);

        chassis.waitUntilDone();
        pros::delay(900);
        chassis.moveToPoint(40, -45, 2500, {.maxSpeed = 75, .earlyExitRange = 0.5});
        chassis.turnToPoint(68, -48, 2000, {.minSpeed = 30, .earlyExitRange = 7});

        subsystems::intake::run(subsystems::intake::GoalType::NONE);


        


        chassis.moveToPoint(70.5, -47, 1550, {.maxSpeed = 55, .minSpeed = 40});
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
        // matchload 1
        chassis.waitUntilDone();
        chassis.tank(-7, -7);
        pros::delay(150);
        chassis.tank(20, 20);
        pros::delay(700);
        chassis.tank(20, 20);
        pros::delay(200);
        chassis.tank(-20, -20);
        pros::delay(125);
        chassis.tank(35, 35);
        pros::delay(1000);

        chassis.moveToPoint(48, -47, 2500, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 0.5});
        chassis.turnToPoint(34, -59, 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});

        chassis.moveToPoint(34, -59, 2500, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 1});
        chassis.turnToPoint(60, -59, 1000, {.minSpeed = 30, .earlyExitRange = 3});
        chassis.moveToPoint(-35, -57, 2500, {.forwards = false, .maxSpeed = 95, .earlyExitRange = 1});
        subsystems::matchload::retract();

        chassis.moveToPoint(-45, -46, 2500, {.forwards = false, .maxSpeed = 85, .earlyExitRange = 0.5});
        chassis.moveToPoint(-23, -45.75, 1400, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntil(15);


        //score high goal
        // score first High goal
        chassis.waitUntilDone();
        
        
        chassis.tank(50, 50);
        subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);

        pros::delay(100);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

        chassis.tank(-127, -127);
        subsystems::matchload::extend();
        pros::delay(2300);
        chassis.setPose(-28, -44, 270);
        pros::delay(50);

        chassis.tank(0, 0);
        pros::delay(100);
        
        pros::delay(100);

        chassis.moveToPoint(-72.75, -44.25, 1420, {.maxSpeed = 52, .minSpeed = 40});

        chassis.waitUntil(5);
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        //matchload 2
        chassis.waitUntilDone();
        chassis.tank(-15, -15);
        pros::delay(150);
        chassis.tank(25, 25);
        pros::delay(1050);
        chassis.tank(-4, -4);
        pros::delay(75);
        chassis.tank(20, 20);
        pros::delay(950);

        chassis.tank(0, 0);
        pros::delay(50);



        chassis.moveToPoint(-24, -44, 1400, {.forwards = false, .maxSpeed = 70});
        

        chassis.waitUntilDone();
        //second High goal score
        subsystems::matchload::retract();

        
        chassis.tank(50, 50);
        subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);

        pros::delay(100);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
        chassis.tank(-127, -127);
        pros::delay(2500);
        
        chassis.setPose(-28, -44, 270);
        pros::delay(150);
        chassis.tank(0, 0);
        chassis.moveToPoint(-31.5, -44, 250, {.maxSpeed = 75, .minSpeed = 50, .earlyExitRange = 1});
        chassis.moveToPoint(-28, -44, 350, {.forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 1});

        


        //side two
        chassis.moveToPoint(-35, -44, 1000);
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
        
        

        chassis.turnToPoint(-29.5, -23, 2000, {.minSpeed = 30, .earlyExitRange = 3});
        chassis.moveToPoint(-30, -23, 2000, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 1});
        chassis.waitUntil(14);
        subsystems::matchload::extend();
        chassis.waitUntilDone();
        subsystems::matchload::retract();

        chassis.moveToPoint(-29, 23, 2000, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 1});
        chassis.waitUntil(35);
        subsystems::matchload::extend();

        chassis.cancelMotion();
        chassis.moveToPoint(-29, 21, 2000, {.maxSpeed = 50, .minSpeed = 40, .earlyExitRange = 1});


        chassis.moveToPoint(-46, 45 , 2000, {.maxSpeed = 70, .minSpeed = 40, .earlyExitRange = 1});


        chassis.turnToPoint(-70, 45, 1000, {.earlyExitRange = 2});
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        chassis.moveToPoint(-22, 48, 1000, {.forwards = false, .maxSpeed = 60});
        chassis.waitUntil(10);
        //score third high  goal
        //
        //

        chassis.waitUntilDone();
        
        chassis.tank(50, 50);
        subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);

        pros::delay(100);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

        chassis.tank(-127, -127);
        subsystems::matchload::extend();

        pros::delay(2300);
        chassis.setPose(-28, 43, 270);
        pros::delay(100);
        chassis.tank(0, 0);
        pros::delay(100);
        
        pros::delay(100);
        chassis.moveToPoint(-71.75, 41, 1600, {.maxSpeed = 50, .minSpeed = 45});
        chassis.waitUntil(8);
        //matchload 3
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        chassis.waitUntilDone();
        chassis.tank(-18, -18);
        pros::delay(160);
        chassis.tank(20, 20);
        pros::delay(925);
        chassis.tank(-7, -7);
        pros::delay(85);
        chassis.tank(25, 25);
        pros::delay(1200);
        chassis.tank(0, 0);
        pros::delay(100);

        
        chassis.moveToPoint(-28, 55, 2500, {.forwards = false, .maxSpeed = 65});

        chassis.waitUntilDone();
        chassis.waitUntil(10);
        subsystems::matchload::retract();

        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        chassis.moveToPoint(32, 53.25, 2500, {.forwards = false, .maxSpeed = 90, .earlyExitRange = 1});

        chassis.moveToPoint(39, 43.5, 2500, {.forwards = false, .maxSpeed = 70});

        chassis.moveToPoint(21, 43, 1300, {.forwards = false, .maxSpeed = 70});

        chassis.waitUntil(15);

        //second High goal score
        
        chassis.waitUntilDone();
        
        chassis.tank(50, 50);
        subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
        pros::delay(200);
        chassis.tank(-127, -110);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

        pros::delay(800);
        chassis.tank(-127, -127);
        subsystems::matchload::extend();

        pros::delay(1100);

        chassis.tank(0, 0);
        pros::delay(100);
        chassis.setPose(28, 44.5, chassis.getPose().theta);
        pros::delay(100);

    
        chassis.moveToPoint(71, 45, 1500, {.maxSpeed = 60, .minSpeed = 40});
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        //matchload 4
        chassis.waitUntilDone();
        chassis.tank(-10, -10);
        pros::delay(150);
        chassis.tank(25, 25);
        pros::delay(1050); 
        chassis.tank(-10, -10);
        pros::delay(100);
        chassis.tank(15, 15);
        pros::delay(625);
        chassis.tank(0, 0);

        chassis.moveToPoint(23, 43.5, 1800, {.forwards = false, .maxSpeed = 75});

        
        subsystems::matchload::retract();
        chassis.waitUntilDone();
        
        chassis.tank(50, 50);
        pros::delay(100);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
        chassis.tank(-127, -127);
        pros::delay(1800);
        chassis.tank(0, 0);
        pros::delay(100);


        // try parking
        chassis.setPose(28, 44, 90);
        pros::delay(100);

        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

        chassis.moveToPoint(57.25, 34, 2000, {.maxSpeed = 65});    
        chassis.waitUntil(15);
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        chassis.moveToPoint(65.5, 7.75, 4000, {.minSpeed = 127});
        pros::delay(450);
        subsystems::matchload::extend();
        pros::delay(1200);
        subsystems::matchload::retract();

        chassis.waitUntilDone();

    */

    skillsOne();
    chassis.waitUntilDone();
    skillsFour();

}

void wing_balls() {
    chassis.setPose(-15, -47.5, 270); //-47 to -47.5
    pros::delay(100);

    chassis.moveToPoint(-23, -35.75, 1300, {.maxSpeed = 85, .minSpeed = 70, .earlyExitRange = 0.5});
    subsystems::wing::retract();
    chassis.turnToHeading(270, 1000, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(-19, chassis.getPose().y, 3500, {.forwards = false, .maxSpeed = 95, .minSpeed = 60, .earlyExitRange = 4}); //-34.75 to -34.55 
    chassis.waitUntil(6);
    chassis.cancelMotion();
    chassis.moveToPose(3, chassis.getPose().y, 270, 7000, {.forwards = false, .maxSpeed = 95, .minSpeed = 85,});

    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1000);
    chassis.waitUntilDone();


    chassis.tank(-30,0);

    //subsystems::wing::retract();
    subsystems::matchload::retract();
    //chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1000);
    //chassis.tank(30,0);
    pros::delay(10000000);
}

void skillsOne(){
    
    /*
        chassis.setPose(47, -1.5, 90);
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
        chassis.tank(50,50);
        pros::delay(750);
        chassis.tank(15,15);
        pros::delay(1550);
        chassis.tank(-32, -32);
        pros::delay(400);
        chassis.moveToPoint(chassis.getPose().x + 5, chassis.getPose().y, 1500, {.maxSpeed = 90, .minSpeed = 65, .earlyExitRange = 1});
        chassis.waitUntilDone(); 
        chassis.tank(25,25);
        pros::delay(350);
        chassis.tank(-20, -20);
        pros::delay(200);
        chassis.tank(-50, 50);
        pros::delay(200);
        chassis.tank(50, -50);
        pros::delay(400);
        chassis.tank(-50, 50);
        pros::delay(200);
        chassis.tank(20,20);
        pros::delay(500);
        chassis.tank(-15,-15);
        pros::delay(250);
        chassis.tank(0,0);
        chassis.moveToPoint(chassis.getPose().x - 12, chassis.getPose().y, 2900, {.forwards = false,.maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1});
        chassis.waitUntilDone();
        chassis.tank(30, 30);
        pros::delay(550);
        chassis.tank(20, 20);
        pros::delay(750);
        chassis.tank(0,0);
        pros::delay(100);
        chassis.setPose(46, chassis.getPose().y, 90);
        pros::delay(100);
        chassis.moveToPoint(19, 0, 2000, {.forwards = false, .maxSpeed = 90, .minSpeed = 50});
        chassis.waitUntilDone();
        chassis.moveToPoint(18.5, -14.25, 2000, {.maxSpeed = 60});
        chassis.waitUntilDone();
        pros::delay(200);
        chassis.moveToPose(9, -7, 135, 2000, {.forwards = false, .maxSpeed = 70});
        chassis.waitUntilDone();
        chassis.tank(-15, -15);
        subsystems::matchload::extend();
        subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
        pros::delay(125);
        subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
        pros::delay(750);
        subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
        pros::delay(100);
        subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
        pros::delay(3200);
        chassis.tank(0,0);
        subsystems::matchload::retract();

        chassis.moveToPoint(27, -27, 2500, {.maxSpeed = 75, .earlyExitRange = 1});

        chassis.moveToPoint(38, -48.25, 2500, {.maxSpeed = 75, .earlyExitRange = 0.5});
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        chassis.moveToPoint(23, -48, 1000, {.forwards = false, .maxSpeed = 75, .earlyExitRange = 0.5});
        subsystems::matchload::extend();

        chassis.waitUntilDone();
        chassis.tank(50, 50);
        pros::delay(200);
        chassis.tank(-127, -127);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
        pros::delay(1000);
        chassis.tank(0,0);
    */


    //start
    chassis.setPose(-48, 14.5, 90);
    chassis.moveToPoint(-31, 17, 1800, {.maxSpeed = 60});
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.waitUntilDone();
    pros::delay(0); //400 to 350

    chassis.moveToPose(-9, 6, 315,  2000, {.forwards = false, .maxSpeed = 80});

    chassis.waitUntilDone();
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
    pros::delay(1100); //2000 to 1650 decrease if need more time to 1700
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    subsystems::matchload::extend();

    //move away from center goal
    chassis.moveToPoint(-40, 43.5, 2000, {.maxSpeed = 80});
    chassis.waitUntilDone();

    chassis.turnToPoint(-68, 46, 1600, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 1.5}); //44.5 to 46


    chassis.moveToPoint(-68, 46, 1000, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 1.5}); //44.5 to 46
    subsystems::wing::toggle();


    chassis.waitUntil(10);
    chassis.cancelMotion();
    chassis.moveToPoint(-67.5, 46, 00, {.maxSpeed = 50, .minSpeed = 40, .earlyExitRange = 1.5}); //46 to 47 and 450 to 700 and 1000 to 900
    
    // facing goal
    chassis.waitUntilDone();
    chassis.tank(16, 16);
    // in matchloader
    // pros::delay(200); //350 to 250
    chassis.tank(0,0);
    
    // out of first matchloader
    chassis.moveToPoint(-58, 44.5, 1500, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 0.5});
    chassis.moveToPoint(-35, 55.5, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 40, .earlyExitRange = 1});

    chassis.moveToPoint(29, 57, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, .earlyExitRange = 1.5});
    subsystems::matchload::retract();

    // turn to get into long goal scoring

    chassis.moveToPoint(36, 44, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1.5}); //45 to 44

    chassis.moveToPoint(20, 44, 1050, {.forwards = false, .maxSpeed = 90, .minSpeed = 50,  .earlyExitRange = 0.5});
    
    subsystems::matchload::extend();

    chassis.waitUntilDone();

    // into long goal 1
    chassis.tank(50,50);
    pros::delay(75);
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    pros::delay(50);
    chassis.tank(-127,-127);
    // outtaking into goal
    pros::delay(2300); //2200 to 2300
    skillsThree();

}
/*
    void skillsTwo(){  
        chassis.setPose(28, -48, chassis.getPose().theta);
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        chassis.moveToPoint(68, -47, 1600, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 1.5});
        subsystems::wing::toggle();

        chassis.waitUntil(20);
        chassis.cancelMotion();
        chassis.moveToPoint(67.75, -47, 1000, {.maxSpeed = 50, .minSpeed = 40, .earlyExitRange = 1.5});

        chassis.waitUntilDone();
        chassis.tank(25,25);
        pros::delay(1050);
        chassis.tank(0,0);
        chassis.moveToPoint(58, -48, 1500, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 0.5});
        chassis.moveToPoint(35, -60, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 40, .earlyExitRange = 1});
        chassis.moveToPoint(-29, -61, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, .earlyExitRange = 1.5});

        chassis.moveToPoint(-33, -48, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1.5});
        chassis.moveToPoint(-20, -48, 1000, {.forwards = false, .maxSpeed = 90, .minSpeed = 50,  .earlyExitRange = 0.5});
        
        subsystems::matchload::extend();

        chassis.waitUntilDone();
        chassis.tank(50,50);
        pros::delay(100);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
        pros::delay(25);
        chassis.tank(-127,-127);
        pros::delay(2500);
        skillsThree();
    }
*/
void hoodPush() {
    chassis.setPose(28, 48, 90);
    // hood push
    chassis.moveToPoint(30.5, 48, 1100, {.maxSpeed = 75, .minSpeed = 60, .earlyExitRange = 0.5});
    chassis.moveToPoint(20, 48, 1300, {.forwards = false, .maxSpeed = 15});
    chassis.waitUntilDone();
}

void skillsThree(){
    chassis.setPose(28, 48, 90);
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    //facing goal
    chassis.moveToPoint(68, 47, 1600, {.maxSpeed = 55, .minSpeed = 50, .earlyExitRange = 1.5});
    subsystems::matchload::extend(); // can take out?

    chassis.waitUntil(20);
    chassis.cancelMotion();
    chassis.moveToPoint(67, 47, 1100, {.maxSpeed = 45, .minSpeed = 40, .earlyExitRange = 1.5});

    chassis.waitUntilDone();

    // into matchloader 2
    chassis.tank(25,25); // both nums to 25 from 20
    pros::delay(700); //700 to 600
    chassis.tank(0,0);

    //out of matchloader 2 and into long goal
    chassis.moveToPoint(23, 48, 900, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 0.5}); //1000 to 900
    chassis.waitUntil(22);
    //outtake balls
    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    chassis.waitUntilDone();
    chassis.tank(-127,-127);
    pros::delay(2400); //2600 to 2400
    chassis.tank(0,0);
    hoodPush();
    // chassis.setPose(28, 44, 90);

}

void skillsFour() {
    /*
        chassis.setPose(28, 44, 90);
        subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

        chassis.moveToPoint(33, 44, 1500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 1.5});

        chassis.moveToPoint(25, 22, 1500, { .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1.5});
        chassis.moveToPoint(40, -50.5, 1500, {.maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1});
        chassis.moveToPoint(22, -49.5, 800, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});

        chassis.waitUntilDone();

        chassis.tank(50,50);
        pros::delay(100);
        chassis.tank(-127,-127);
        subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

        pros::delay(1000);
        subsystems::matchload::extend();
    */
    subsystems::wing::extend();
    subsystems::matchload::retract();
    chassis.setPose(28, 44, 90);

    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);

    pros::delay(100);

    // move to clear park zone
    chassis.moveToPoint(59.5, 30, 2000, {.maxSpeed = 65});
    chassis.waitUntilDone();

    chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 30, .earlyExitRange = 3});
    // clear park zone
    chassis.moveToPoint(68, 25, 1000, {.maxSpeed = 90, .minSpeed = 80, .earlyExitRange = 1.5});
    chassis.waitUntil(2);
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    chassis.moveToPoint(68, -18.5, 3000, {.maxSpeed = 85}); //69 to 68 and 4000 to 3000 and 90 to 85 and -17 to -18.5
    chassis.waitUntilDone();
    pros::delay(400); //400 to 300
    subsystems::localization::leftDistanceReset(chassis, subsystems::localization::Wall::RIGHT_X);
    pros::delay(400); //400 to 300

    chassis.turnToHeading(270,1000);
    chassis.waitUntilDone();
    pros::delay(50);
    subsystems::localization::leftDistanceReset(chassis, subsystems::localization::Wall::BOTTOM_Y);
    pros::delay(100);

    // getting extra ball
    chassis.moveToPose(35.75, -22.25, 270, 4000); //35.25 to 36
    chassis.waitUntilDone();

    //turn toward middle goal
    chassis.turnToPoint(11, -10, 1000, {.forwards = false});
    chassis.waitUntilDone();
    //move toward middle goal
    subsystems::matchload::extend();
    chassis.moveToPose(11, -11, 135, 2000, {.forwards = false, .lead = 0.3, .maxSpeed = 40, .minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();

    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(50);
    // in middle goal
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL_SLOW);
    chassis.tank(-30, -30);
    pros::delay(1500);
    subsystems::intake::run(subsystems::intake::GoalType::LOW_GOAL);
    pros::delay(100);
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL);
    pros::delay(1000); //1500 to 1000
    subsystems::intake::run(subsystems::intake::GoalType::MEDIUM_GOAL_SLOW);
    pros::delay(1000); //500 to 1000
    // 3600

    chassis.moveToPoint(40, -49, 2000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);
    subsystems::matchload::extend(); // not needed

    chassis.moveToPose(61, -51, 90,  1000, {.maxSpeed = 65, .minSpeed = 60, .earlyExitRange = 1.5});

    chassis.waitUntil(10);
    chassis.cancelMotion();
    chassis.moveToPose(62, -51, 90, 1000, {.maxSpeed = 60, .minSpeed = 45, .earlyExitRange = 1.5}); //61 to 62 and 525 to 1000 40 to 50

    chassis.waitUntilDone();
    chassis.tank(30, 30);
    // into 3rd matchloader
    pros::delay(1700); //2000 to 1700
    chassis.tank(0,0);

    chassis.moveToPoint(50, -50, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.moveToPose(29, -64, 90, 2000, {.forwards = false, .lead = 0.3, .maxSpeed = 110, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntilDone();
    subsystems::matchload::retract();
    chassis.moveToPoint(-31, -64, 3000, {.forwards = false, .maxSpeed = 110, .minSpeed = 50, .earlyExitRange = 3}); // -30 to -31
    chassis.waitUntilDone();
    chassis.moveToPoint(-31, -50.5, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.25}); // 52 to 50.5 and -30 to -31
    chassis.waitUntilDone();
    chassis.moveToPoint(-15, -50.5, 1000, {.forwards = false}); // 52 to 50.5
    chassis.waitUntilDone();

    chassis.tank(-127, -127);

    subsystems::intake::run(subsystems::intake::GoalType::LONG_GOAL);
    subsystems::matchload::extend();
    pros::delay(2900); //3000 to 2900

    skillsThree();

    subsystems::matchload::retract();

    skillsFive();
}

void skillsFive() {
    subsystems::wing::extend();
    subsystems::matchload::retract();
    chassis.setPose(28, 44, 90);

    subsystems::intake::run(subsystems::intake::GoalType::HOLD_BALLS);

    pros::delay(100);

    // move to clear park zone
    chassis.moveToPoint(61.5, 30, 2000, {.maxSpeed = 65, .minSpeed = 30, .earlyExitRange = 1.5});
    chassis.waitUntilDone();

    chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 30, .earlyExitRange = 3});
    // clear park zone
    chassis.moveToPoint(70, 5, 2000, {.maxSpeed = 110, .minSpeed = 105, .earlyExitRange = 1.5}); //2.25 to 1 and 127 to 110 and 110 to 105
    chassis.waitUntilDone();
    pros::delay(100000);
}

void skills_100() {

}

void skills_1() {

}