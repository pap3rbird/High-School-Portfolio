/*__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"1","name":"lbtrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":false},"bufferIndex":0},{"hwid":"2","name":"lftrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":false},"bufferIndex":1},{"hwid":"11","name":"rbtrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":true},"bufferIndex":2},{"hwid":"12","name":"rftrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":true},"bufferIndex":3},{"hwid":"19","name":"arm","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":true},"bufferIndex":4},{"hwid":"20","name":"claw","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":true},"bufferIndex":5},{"hwid":"triport_adi","name":"triport22","typeName":"triport","extraConfig":null,"bufferIndex":6},{"hwid":"drivetrain","name":"dt","typeName":"drivetrain","extraConfig":null,"bufferIndex":7},{"hwid":"controller","name":"con","typeName":"controller_one","extraConfig":null,"bufferIndex":8},{"hwid":"Axis1","name":"axis1","typeName":"controller_axis","extraConfig":null,"bufferIndex":9},{"hwid":"Axis2","name":"axis2","typeName":"controller_axis","extraConfig":null,"bufferIndex":10},{"hwid":"Axis3","name":"axis3","typeName":"controller_axis","extraConfig":null,"bufferIndex":11},{"hwid":"Axis4","name":"axis4","typeName":"controller_axis","extraConfig":null,"bufferIndex":12},{"hwid":"ButtonL1","name":"buttonL1","typeName":"controller_button","extraConfig":null,"bufferIndex":13},{"hwid":"ButtonL2","name":"buttonL2","typeName":"controller_button","extraConfig":null,"bufferIndex":14},{"hwid":"ButtonR1","name":"buttonR1","typeName":"controller_button","extraConfig":null,"bufferIndex":15},{"hwid":"ButtonR2","name":"buttonR2","typeName":"controller_button","extraConfig":null,"bufferIndex":16},{"hwid":"ButtonUp","name":"buttonUp","typeName":"controller_button","extraConfig":null,"bufferIndex":17},{"hwid":"ButtonDown","name":"buttonDown","typeName":"controller_button","extraConfig":null,"bufferIndex":18},{"hwid":"ButtonLeft","name":"buttonLeft","typeName":"controller_button","extraConfig":null,"bufferIndex":19},{"hwid":"ButtonRight","name":"buttonRight","typeName":"controller_button","extraConfig":null,"bufferIndex":20},{"hwid":"ButtonX","name":"buttonX","typeName":"controller_button","extraConfig":null,"bufferIndex":21},{"hwid":"ButtonB","name":"buttonB","typeName":"controller_button","extraConfig":null,"bufferIndex":22},{"hwid":"ButtonY","name":"buttonY","typeName":"controller_button","extraConfig":null,"bufferIndex":23},{"hwid":"ButtonA","name":"buttonA","typeName":"controller_button","extraConfig":null,"bufferIndex":24}]}*/
// VEX V5 C++ Project with Competition Template
#include "vex.h"
using namespace vex;

//#region config_globals
vex::brain      Brain;
vex::motor      lbtrain(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor      lftrain(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor      rbtrain(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor      rftrain(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor      arm(vex::PORT19, vex::gearSetting::ratio18_1, true);
vex::motor      claw(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::controller con(vex::controllerType::primary);
//#endregion config_globals


// Creates a competition object that allows access to Competition methods.
vex::competition Competition;

//#region functions
// Move forward for specified distance in parentheses.
void pdistance(float dist)
{
    float degrees = (360 * dist)/3.141592654;
    
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rbtrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

// Move backward for specified distance in parentheses.
void ndistance(float dist)
{
    float degrees = (360 * dist)/3.141592654;
    
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rbtrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Rotates 90 degrees to the left.
void rot90left()
{
    float dist = 2.5;
    float degrees = (360 * dist)/3.141592654;
    
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Rotates 90 degrees to the right.
void rot90right()
{
    float dist = 2.5;
    float degrees = (360 * dist)/3.141592654;
    
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

// Rotates 45 degrees to the right.
void rot45right()
{
    float dist = 1.25;
    float degrees = (360 * dist)/3.141592654;
    
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

// Rotates 45 degrees to the left.
void rot45left()
{
    float dist = 1.25;
    float degrees = (360 * dist)/3.141592654;
    
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Rotates 35 degrees to the right.
void rot35right()
{
    float dist = 0.75;
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

// Rotates 35 degrees to the left.
void rot35left()
{
    float dist = 0.75;
    float degrees = (360 * dist)/3.141592654;
    
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

//Picks up a cube: Opens claw, moves forward slightly, closes claw, lifts arm.
void pickupcube()
{
   float dist = 0.5;
   float degrees = (360 * dist)/3.141592654;
   
   arm.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
   
   dist = 0.1;
   rftrain .startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
   rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
   lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
   lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
   
   dist = 0.5;
   claw.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
   
   dist = 2;
   arm.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Lifts arm.
void armlift()
{
    float dist = 2;
    float degrees = (360 * dist)/3.141592654;
    
    arm.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Lowers arm.
void armlower()
{
    float dist =  2;
    float degrees = (360 * dist)/3.141592654;
    
    arm.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

// Closes the claw.
void openclaw()
{
    float dist = 0.4;
    float degrees = (360 * dist)/3.141592654;
    
    claw.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    sleepMs(300);
}

// Opens the claw.
void closeclaw()
{
    float dist = 1.15;
    float degrees = (360 * dist)/3.141592654;
    
    claw.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    sleepMs(300);
}

// Puts all motors (claw, arm, train) on coast (from hold).
void stopcoast()
{
    claw.stop(coast);

    arm.stop(coast);
    
    rftrain.stop(coast);
    rbtrain.stop(coast);
    lbtrain.stop(coast);
    lftrain.stop(coast);
}
//#endregion functions
void pre_auton() {
    // All activities that occur before competition start
    // Example: setting initial positions

}

void autonomous() {
    // Place autonomous code here
    
    //#region basic_autonomous
    // BASIC AUTONOMOUS (Works for both lower red and blue positions)
    // This program will make the robot push a preset cube into the goal.
    // Set velocity to 30% for all drivetrain motors.
    lbtrain.setVelocity(30, pct);
    lftrain.setVelocity(30, pct);
    rbtrain.setVelocity(30, pct);
    rftrain.setVelocity(30, pct);
    // Start with the robot facing away from the bigger goal.
    // The robot's back wheels should be on the seam of the lower end of the colored tile.
    // The preset cube should be placed adjacent to the wall and on the back side of the robot.
    // The robot will move backwards until it pushes the cube into the bigger goal.
    ndistance(9);
    sleepMs(100);
    // The robot will move forwards so that it is not making contact with the cube when the autonomous period ends.
    pdistance(4.5);
    sleepMs(100);
    // All drivetrain motors will be set to brake type coast.
    stopcoast();
    sleepMs(300);
    // End of autonomous.
    //endregion basic_autonomous
    
    /*
    //#region autonomous_red
    // AUTONOMOUS ADD-ON ***RED SIDE***
    // In addition to pushing one cube into the goal, the robot will also pick up another cube and place it next to the first one in the goal.
    // Rotates 45 degrees left in order to face the cube adjacent to the left most red tile, then moves towards the cube and closes the claw around it.
    // Rotates 45 degrees left.
    rot45left();
    sleepMs(100);
    // Moves forward towards the cube adjacent to the lower colored tile.
    pdistance(4.5);
    sleepMs(100);
    // Closes the claw around the cube.
    closeclaw();
    sleepMs(100);
    
    // Moves backwards towards the goal (total 170 degrees left), then turns around to face the bigger goal.
    // Moves backwards towards the goal.
    ndistance(4);
    sleepMs(100);
    // Rotates 170 degrees in succession (seperated into different functions for convenience to different tasks).
    rot45left();
    sleepMs(100);
    rot90left();
    sleepMs(100);
    rot35left();
    sleepMs(100);
    
    // Raises arm. Then moves forward towards the goal, lowers arm, then opens claw. Moves backwards.
    // Raises the arm as to not have it be caught on the rim of the goal.
    armlift();
    sleepMs(100);
    // Moves forward towards the goal.
    pdistance(3);
    sleepMs(100);
    // Lowers the arm into the goal.
    armlower();
    sleepMs(100);
    // Opens the claw in order to drop the cube into the goal.
    openclaw();
    sleepMs(100);
    // Moves backwards as to not make contact with the cube when the autonomous period ends.
    ndistance(4);
    sleepMs(100);
    // All drivetrain motors will be set to brake type coast.
    stopcoast();
    // End of autonomous.
    //#endregion autonomous_red
    */
    
    //#region autonomous_blue
    // AUTONOMOUS ADD-ON ***BLUE SIDE***
    // In addition to pushing one cube into the goal, the robot will also pick up another cube and place it next to the first one in the goal.
    // Rotates 45 degrees right in order to face the cube adjacent to the left most red tile, then moves towards the cube and closes the claw around it.
    // Rotates 45 degrees right.
    rot45right();
    sleepMs(100);
    // Moves forward towards the cube adjacent to the lower colored tile.
    pdistance(4.5);
    sleepMs(100);
    // Closes the claw around the cube.
    closeclaw();
    sleepMs(100);
    
    // Moves backwards towards the goal (total 170 degrees left), then turns around to face the bigger goal.
    // Moves backwards towards the goal.
    ndistance(4);
    sleepMs(100);
    // Rotates 170 degrees in succession (seperated into different functions for convenience to different tasks).
    rot45right();
    sleepMs(100);
    rot90right();
    sleepMs(100);
    rot35right();
    sleepMs(100);
    
    // Raises arm. Then moves forward towards the goal, lowers arm, then opens claw. Moves backwards.
    // Raises the arm as to not have it be caught on the rim of the goal.
    armlift();
    sleepMs(100);
    // Moves forward towards the goal.
    pdistance(3);
    sleepMs(100);
    // Lowers the arm into the goal.
    armlower();
    sleepMs(100);
    // Opens the claw in order to drop the cube into the goal.
    openclaw();
    sleepMs(100);
    // Moves backwards as to not make contact with the cube when the autonomous period ends.
    ndistance(4);
    sleepMs(100);
    // All drivetrain motors will be set to brake type coast.
    stopcoast();
    // End of autonomous.
    //#endregion blue_autonomous
    
    //#region skills_challenge
    // This autonomous program will put five cubes into the bigger blue goal.
        // 1) One preset block (green) 
        // 2) One block adjacent to the lower blue tile (green) 
        // 3) Two blocks closest to the bigger goal that are adjacent to the rightmost goblet (purple and green) 
        // 4) One block on the gray tile diagonal to the lower blue tile (purple) 
            // OR 
        // 5) One block on the upper blue tile (green).
    // All colors of blocks will be scored the same in this challenge.
    // The robot will score 1 first, then 2, purple 3, 4, and green 3.
        // OR
    // The robot will score 5 first, then 1, 2, purple 3, and green 3.
    
    // The robot will start facing away from the bigger goal, with its back wheels on the bottom seam of the lower blue tile.
    // The preset block will be placed adjacent to the wall and the back side of the robot.
        // Move backward to drop the preset cube into the goal. 
        // Move forward slightly to get the robot out of the goal.
        // Turn right to face directly at Block 2. Move forward and grab Block 2, then move backwards.
        // Turn around, then move forward and drop the cube into the goal.
        // Turn around, then turn directly to block purple 3.
        // Move forward and grab block purple 3, then move backwards.
        // Turn around, then move forward and drop the cube into the goal.
        // 
    
    //#endregion skills_challenge
}

void drivercontrol() {
    // Place drive control code here, inside the loop
    while (true) {
        //#region drivetrain
        // DRIVETRAIN MOTORS * 4 (Left front and back, right front and back) SPLIT CONTROL (currently active)
        // Moving the left stick up and down will move all of the drivetrain motors forward or backward.
        // Moving the right stick left or right will move all of the drivetrain motors left or right.
        lbtrain.spin(directionType::fwd, (con.Axis3.value() + con.Axis1.value())/2, velocityUnits::pct); //(Axis3+Axis4)/2;
        lftrain.spin(directionType::fwd, (con.Axis3.value() + con.Axis1.value())/2, velocityUnits::pct);//(Axis3-Axis4)/2;
        rftrain.spin(directionType::fwd, (con.Axis3.value() - con.Axis1.value())/2, velocityUnits::pct); //(Axis3+Axis4)/2;
        rbtrain.spin(directionType::fwd, (con.Axis3.value() - con.Axis1.value())/2, velocityUnits::pct);//(Axis3-Axis4)/2;
        //#endregion drivetrain
        
        //#region arm_motors
        // ARM MOTORS * 2 
        // Button L1 = Top Left Bumper
        // Makes the arm go up at a velocity of 50%.
        if (con.ButtonL1.pressing())
        {
            arm.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        // Button L2 = Bottom Left Bumper
        // Makes the arm go down at a velocity of 50%.
        else if (con.ButtonL2.pressing())
        {
            arm.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        }
        // This statement is active when no arm-related buttons are pressed.
        else
        {
            arm.stop(vex::brakeType::hold);
        }
        //#endregion arm_motors
        
        //#region claw_motors
        // CLAW MOTORS * 2
        // Button R1 = Top Right Bumper
        // Makes the claw open at a velocity of 50%.
        if (con.ButtonR1.pressing())
        {
            claw.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        // Button R2 = Bottom Right Bumper
        // Makes the claw close at a velocity of 50%.
        else if (con.ButtonR2.pressing())
        {
            claw.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        }
        // This statement is active when no claw-related buttons are pressed.
        else
        {
            claw.stop(vex::brakeType::hold);
        }
        //#endregion claw_motors
    }
}

int main() {
    // Do not adjust the lines below

    // Set up (but don't start) callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Robot Mesh Studio runtime continues to run until all threads and
    // competition callbacks are finished.
}