/*__CONFIG__
{"version":20,"widgetInfos":[{"hwid":"2","name":"claw","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":false},"bufferIndex":0},{"hwid":"4","name":"larm","typeName":"motor","extraConfig":{"gearSetting":0,"reverse":true},"bufferIndex":1},{"hwid":"5","name":"rarm","typeName":"motor","extraConfig":{"gearSetting":0,"reverse":false},"bufferIndex":2},{"hwid":"7","name":"rbtrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":true},"bufferIndex":3},{"hwid":"8","name":"rftrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":true},"bufferIndex":4},{"hwid":"9","name":"lbtrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":false},"bufferIndex":5},{"hwid":"10","name":"lftrain","typeName":"motor","extraConfig":{"gearSetting":1,"reverse":false},"bufferIndex":6},{"hwid":"triport_adi","name":"triport22","typeName":"triport","extraConfig":null,"bufferIndex":7},{"hwid":"drivetrain","name":"dt","typeName":"drivetrain","extraConfig":null,"bufferIndex":8},{"hwid":"controller","name":"con","typeName":"controller_one","extraConfig":null,"bufferIndex":9},{"hwid":"Axis1","name":"axis1","typeName":"controller_axis","extraConfig":null,"bufferIndex":10},{"hwid":"Axis2","name":"axis2","typeName":"controller_axis","extraConfig":null,"bufferIndex":11},{"hwid":"Axis3","name":"axis3","typeName":"controller_axis","extraConfig":null,"bufferIndex":12},{"hwid":"Axis4","name":"axis4","typeName":"controller_axis","extraConfig":null,"bufferIndex":13},{"hwid":"ButtonL1","name":"buttonL1","typeName":"controller_button","extraConfig":null,"bufferIndex":14},{"hwid":"ButtonL2","name":"buttonL2","typeName":"controller_button","extraConfig":null,"bufferIndex":15},{"hwid":"ButtonR1","name":"buttonR1","typeName":"controller_button","extraConfig":null,"bufferIndex":16},{"hwid":"ButtonR2","name":"buttonR2","typeName":"controller_button","extraConfig":null,"bufferIndex":17},{"hwid":"ButtonUp","name":"buttonUp","typeName":"controller_button","extraConfig":null,"bufferIndex":18},{"hwid":"ButtonDown","name":"buttonDown","typeName":"controller_button","extraConfig":null,"bufferIndex":19},{"hwid":"ButtonLeft","name":"buttonLeft","typeName":"controller_button","extraConfig":null,"bufferIndex":20},{"hwid":"ButtonRight","name":"buttonRight","typeName":"controller_button","extraConfig":null,"bufferIndex":21},{"hwid":"ButtonX","name":"buttonX","typeName":"controller_button","extraConfig":null,"bufferIndex":22},{"hwid":"ButtonB","name":"buttonB","typeName":"controller_button","extraConfig":null,"bufferIndex":23},{"hwid":"ButtonY","name":"buttonY","typeName":"controller_button","extraConfig":null,"bufferIndex":24},{"hwid":"ButtonA","name":"buttonA","typeName":"controller_button","extraConfig":null,"bufferIndex":25}]}*/
// VEX V5 C++ Project with Competition Template
#include "vex.h"
using namespace vex;

//#region config_globals
vex::brain      Brain;
vex::motor      claw(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor      larm(vex::PORT4, vex::gearSetting::ratio36_1, true);
vex::motor      rarm(vex::PORT5, vex::gearSetting::ratio36_1, false);
vex::motor      rbtrain(vex::PORT7, vex::gearSetting::ratio18_1, true);
vex::motor      rftrain(vex::PORT8, vex::gearSetting::ratio18_1, true);
vex::motor      lbtrain(vex::PORT9, vex::gearSetting::ratio18_1, false);
vex::motor      lftrain(vex::PORT10, vex::gearSetting::ratio18_1, false);
vex::controller con(vex::controllerType::primary);
//#endregion config_globals


// Creates a competition object that allows access to Competition methods.
vex::competition Competition;

// Move forward for specified distance in parentheses.
void pdistance(float dist)
{
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

// Move backward for specified distance in parentheses.
void ndistance(float dist)
{
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Rotates 90 degrees to the left.
void rot90left()
{
    float dist = 2.5;
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Rotates 90 degrees to the right.
void rot90right()
{
    float dist = 2.5;
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

void rot45right()
{
    float dist = 1.25;
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

void rot45left()
{
    float dist = 1.25;
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

void rot35right()
{
    float dist = 0.75;
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
}

void rot35left()
{
    float dist = 0.75;
    float degrees = (360 * dist)/3.141592654;
    
    rbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rftrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    lbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    lftrain.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

//Picks up a cube: Opens claw, moves forward slightly, closes claw, lifts arm.
void pickupcube()
{
   float dist = 0.5;
   float degrees = (360 * dist)/3.141592654;
   
   claw.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
   
   dist = 0.1;
   rbtrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
   rftrain.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
   lbtrain.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
   lftrain.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
   
   dist = 0.5;
   claw.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
   
   dist = 2;
   larm.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
   rarm.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
}

// Drops cube(s) in the robot's dclaw: Lifts arm, moves forward slightly.
void raisearm()
{
    float dist = 2;
    float degrees = (360 * dist)/3.141592654;
    
    larm.startRotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    rarm.rotateFor(vex::directionType::rev, degrees, vex::rotationUnits::deg);
    sleepMs(300);
}

void lowerarm()
{
    float dist = 2;
    float degrees = (360 * dist)/3.141592654;
    
    larm.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    rarm.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
    sleepMs(300);
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
    
    rarm.stop(coast);
    larm.stop(coast);
    
    rbtrain.stop(coast);
    rftrain.stop(coast);
    lbtrain.stop(coast);
    lftrain.stop(coast);
}

void pre_auton()
{
    // All activities that occur before competition start
    // Example: setting initial positions
    
    //empty
}

void autonomous() {
    //#region set_velocity
    // Set all drivetrain velocities to a certain percentage before beginning autonomous program.
    // Controls how fast or precise the robot will be when attempting actions.
    // (A faster velocity might result in the robot sliding after finishing its distance, overshooting where it should be).
    rbtrain.setVelocity(30, vex::velocityUnits::pct);
    rftrain.setVelocity(30, vex::velocityUnits::pct);
    lbtrain.setVelocity(30, vex::velocityUnits::pct);
    lftrain.setVelocity(30, vex::velocityUnits::pct);
    //#endregion set_velocity
    
    //#region blue_side
    /*// Class Competition Autonomous test  
        // BLUE SIDE
        // Puts two one-stack cubes into the bigger goal. One cube is preset and the other is adjacent to the right most blue tile.
        // Moves backwards to push cube into the goal, then moves forward.
        ndistance(9);
        sleepMs(100);
        pdistance(2);
        sleepMs(100);
        
        // Rotates 45 degrees right in order to face the cube adjacent to the right most blue tile. Then moves toward the cube and closes the claw around it.
        rot45right();
        sleepMs(100);
        pdistance(4.5);
        sleepMs(100);
        closeclaw();
        sleepMs(100);
        
        // Moves backwards towards the goal, then turns around to face the goal.
        ndistance(4);
        sleepMs(100);
        rot45right();
        sleepMs(100);
        rot90right();
        sleepMs(100);
        rot35right();
        sleepMs(100);
        
        // Raises arm. Then moves forward towards the goal, lowers arm, then opens claw. Moves backwards to move away from the goal. End of autonomous.
        dropcube();
        sleepMs(100);
        pdistance(3);
        sleepMs(100);
        float dist = 2;
        float degrees = (360 * dist)/3.141592654;
        larm.startRotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
        rarm.rotateFor(vex::directionType::fwd, degrees, vex::rotationUnits::deg);
        sleepMs(100);
        openclaw();
        sleepMs(100);
        ndistance(4);
        sleepMs(100);
        stopcoast();*/
        //#endregion blue_side
    
    //#region red_side
        // RED SIDE
        // Puts two one-stack cubes into the bigger goal. One cube is preset and the other is adjacent to the left most red tile.
        // Moves backwards to push cube into the goal, then moves forward.
        // Moves backwards.
        ndistance(9);
        sleepMs(100);
        //Moves forwards.
        pdistance(2);
        sleepMs(100);
        
        // Rotates 45 degrees right in order to face the cube adjacent to the left most red tile. Then moves toward the cube and closes the claw around it.
        // Rotates 45 degrees.
        rot45left();
        sleepMs(100);
        // Moves forwards.
        pdistance(4.5);
        sleepMs(100);
        // Closes claw.
        closeclaw();
        sleepMs(100);
        
        // Moves backwards towards the goal (total 170 degrees left), then turns around to face the goal.
        // Moves backwards.
        ndistance(4);
        sleepMs(100);
        // Rotates 45 degrees.
        rot45left();
        sleepMs(100);
        // Rotates 90 degrees.
        rot90left();
        sleepMs(100);
        // Rotates 35 degrees.
        rot35left();
        sleepMs(100);
        
        // Raises arm. Then moves forward towards the goal, lowers arm, then opens claw. Moves backwards to move away from the goal. End of autonomous.
        // Raises arm.
        raisearm();
        sleepMs(100);
        // Moves forward.
        pdistance(3);
        sleepMs(100);
        // Lowers arm.
        lowerarm();
        sleepMs(100);
        // Opens claw.
        openclaw();
        sleepMs(100);
        // Moves backwards.
        ndistance(4);
        sleepMs(100);
        // Stops all motors with brake type coast.
        stopcoast();
    //#endregion red_side
    
    //#region scrap_code
    /*// RED UPPER OR BLUE LOWER (actually all positions.......)
        // This code puts a single cube into the goal. The cube is set up by the driver and is not picked up during the code.
        // Makes the brain display the printed phrase when this version of autonomous is active.
        Brain.Screen.render(true,false);
        Brain.Screen.clearLine(0, color::black);
        Brain.Screen.clearLine(1, color::black);
        Brain.Screen.clearLine(2, color::black);
        Brain.Screen.setCursor(1,0);
        Brain.Screen.print("Running autonomous for:");
        Brain.Screen.setCursor(2,0);
        Brain.Screen.print("ALL POSITIONS :)");
        Brain.Screen.render();
        // Start facing into the arena with the back of the robot pushed against the wall. 
        // Move forward slightly.
        //pdistance(0.1);
        //sleepMs(300);
        // Rotate right 90 degrees.
        //rot90right();
        //sleepMs(300);
        // Move forward towards the goal.
        ndistance(9);
        sleepMs(300);
        // Drop cube into goal.
        /*dropcube();
        sleepMs(300);
        openclaw();
        sleepMs(300);
        //closeclaw();
        //sleepMs(300);*
        // Move backward slightly.
        pdistance(2);
        sleepMs(300);
        // Stop all motors.
        stopcoast();
        // Clear display screen.
        Brain.Screen.clearLine(0, color::black);
        Brain.Screen.clearLine(1, color::black);
        Brain.Screen.clearLine(2, color::black);*/
    
    /*
    // RED LOWER OR BLUE UPPER
    // This code puts a single cube into the goal. The cube is set up by the driver and is not picked up during the code.
        // Makes the brain display the printed phrase when this version of autonomous is active.
        Brain.Screen.render(true, false);
        Brain.Screen.clearLine(0, color::black);
        Brain.Screen.clearLine(1, color::black);
        Brain.Screen.clearLine(1, color::black);
        Brain.Screen.setCursor(1,0);
        Brain.Screen.print("Running autonomous for:");
        Brain.Screen.setCursor(2,0);
        Brain.Screen.print("Red lower or blue upper positions.");
        Brain.Screen.render();
        // Start facing into the arena with the back of the robot pushed against the wall.
        // Move forward slightly.
        pdistance(0.1);
        sleepMs(300);
        // Rotate right 90 degrees.
        rot90left();
        sleepMs(300);
        // Move forward towards the goal.
        pdistance(8);
        sleepMs(300);
        // Drop cube into goal.
        closeclaw();
        sleepMs(300);
        dropcube();
        sleepMs(300);
        openclaw();
        sleepMs(300);
        // Move backward slightly.
        ndistance(2);
        sleepMs(300);
        //Stop all motors.
        stopcoast();
        // Clear display screen.
        Brain.Screen.clearLine(0, color::black);
        Brain.Screen.clearLine(1, color::black);
    */
    
        // Test code.
        /*pdistance(6);
        sleepMs(300);
        ndistance(4);
        sleepMs(300);
        //schleep
        stopcoast();*/
    
    /*// BLUE LOWER
        // This code puts two cubes into the goal. One is set up by the driver and the other is picked up during the autonomous period.
        // Start facing into the arena, hugging the right wall and on the tile directly adjacent to the lower blue start tile.
        // Move forward slightly.
        // Rotate right 90 degrees, facing the goal.
        // Move forward slightly.
        // Rotate left 90 degrees, facing the cube adjacent to the blue tile that the robot is sitting on.
        // Move forward slightly.
        // Pick up the cube.
        // Move backward slightly.
        // Rotate right 90 degrees, facing the goal.
        // Move forward towards the goal.
        // Drop the cube into the goal.
    
    // BLUE UPPER
        // This code puts two cubes into the goal. One is set up by the driver and the otehr is picked up during the autonomous period.
        // Start facing into the arena, hugging the top wall and on the tile directly adjacent to the upper blue start tile.
        // Move forward slightly.
        // Rotate left 90 degrees, facing the goal.
        // Move forward towards the cube.
        // Pick up the cube.
        // Rotate left 90 degrees.
        // Move forward slightly.
        // Rotate right 90 degrees, facing the goal.
        // Move forward towards the goal.
        // Drop the cube into the goal.
        
    // RED LOWER edit
        // This code puts two cubes into the goal. One is set up by the driver and the other is picked up during the autonomous period.
        // Start facing into the arena, hugging the left wall and on the tile directly adjacent to the lower red start tile.
        // Move forward slightly.
        // Rotate left 90 degrees, facing the goal.
        // Move forward slightly.
        
    // RED UPPER edit
        // This code puts two cubes into the goal. One is set up by the driver and the other is picked up during the autonomous period.
        // Start facing into the arena, hugging the top wall and on the tile directly adjacent to the upper red start tile.*/
    //#endregion scrap_code
}

void drivercontrol() {
    // Place drive control code here, inside the loop
    while (true) 
    {
        //#region drivetrain
        /*// DRIVETRAIN MOTORS * 4 (Left front and back, right front and back) TANK CONTROL
        // Moving the left stick up or down will move the left drivetrain motors forward or backward.
        // Moving the right stick up or down will move the right drivetrain motors forward or backward.
        lbtrain.spin(vex::directionType::fwd, con.Axis3.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        lftrain.spin(vex::directionType::fwd, con.Axis3.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        rbtrain.spin(vex::directionType::fwd, con.Axis2.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        rftrain.spin(vex::directionType::fwd, con.Axis2.position(vex::percentUnits::pct), vex::velocityUnits::pct);*/
        
        // DRIVETRAIN MOTORS * 4 (Left front and back, right front and back) SPLIT CONTROL (currently active)
        // Moving the left stick up and down will move all of the drivetrain motors forward or backward.
        // Moving the right stick left or right will move all of the drivetrain motors left or right.
        lbtrain.spin(directionType::fwd, (con.Axis3.value() + con.Axis1.value())/2, velocityUnits::pct); //(Axis3+Axis4)/2;
        lftrain.spin(directionType::fwd, (con.Axis3.value() + con.Axis1.value())/2, velocityUnits::pct);//(Axis3+Axis4)/2;
        rbtrain.spin(directionType::fwd, (con.Axis3.value() - con.Axis1.value())/2, velocityUnits::pct); //(Axis3-Axis4)/2;
        rftrain.spin(directionType::fwd, (con.Axis3.value() - con.Axis1.value())/2, velocityUnits::pct);//(Axis3-Axis4)/2;
        //#endregion drivetrain
        
        //#region arm
        // ARM MOTORS * 2 (left and right)
        // Button R1 = Top Right Bumper
        // Makes the arm/elevator go up at a velocity of 75%.
        if (con.ButtonR1.pressing())
        {
            larm.spin(vex::directionType::rev, 85, vex::velocityUnits::pct);
            rarm.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
        }
        // Button R2 = Bottom Right Bumper
        // Makes the arm/elevator go down at a velocity of 50%.
        // (Velocity is lower than R1 so that the arm has less of a chance of slamming into the drivetrain or microcontroller.)
        else if (con.ButtonR2.pressing())
        {
            larm.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
            rarm.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        }
        // Button X = Top Left Face Button
        // When this button is held, the brake type of the arm motors changes from hold to coast.
        // The purpose of this button is to be able to manually reset the arms during testing without switching off driver mode.
        else if (con.ButtonX.pressing())
        {
            larm.stop(vex::brakeType::coast);
            rarm.stop(vex::brakeType::coast);
        }
        // This statement is active when no other arm-related buttons are pressed.
        // This will hold the arm motors in the place that they were in when you released the button.
        else
        {
            larm.stop(vex::brakeType::hold);
            rarm.stop(vex::brakeType::hold);
        }
        //#endregion arm
        
        //#region claw
        // Claw motor
        // Button L1 = Top Left Bumper
        // Makes the claw open at a velocity of 50%.
        if (con.ButtonL1.pressing())
        {
            claw.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        // Button L2 = Bottom Left Bumper
        // Makes the claw close at a velocity of 50%.
        else if (con.ButtonL2.pressing())
        {
            claw.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        }
        // This statement is active when no other claw-related buttons are pressed.
        // This will hold the claw motor in the place that it was in when you released the button.
        else
        {
            claw.stop(vex::brakeType::hold);
        }
        //#endregion claw
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