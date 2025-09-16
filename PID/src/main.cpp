/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       aqian2                                                    */
/*    Created:      9/5/2025, 1:03:58 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;

inertial spin = inertial (PORT4);

//motors
motor RF = motor(PORT8, ratio6_1);
motor RM = motor(PORT9, ratio6_1);
motor RB = motor(PORT10, ratio6_1);
motor LF = motor(PORT3, ratio6_1, true);
motor LM = motor(PORT2, ratio6_1, true);
motor LB = motor(PORT1, ratio6_1, true);

motor_group rALL = motor_group(RF, RM, RB);
motor_group lALL = motor_group(LF, LM, LB);

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
//callirating
spin.calibrate();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
float turnTolerance=1; //tolerance is 1 degree

void turnNoPID(float target){ //target: set turning amount , global on global frame

  while(1){

float currentHeading = spin.heading(); //set the current heading
float rTurn = target-currentHeading;
float lTurn = currentHeading-target;
//correct turns if negative
if (lTurn<0){
  lTurn+=360;
}
if (rTurn<0){
  rTurn+=360;
}
//how much degrees you need to turn locally
float turnDirection = 0;
//find most efficent path
if (rTurn<lTurn){
  turnDirection=1;
}
else{
  turnDirection=-1;
}
//check if we are close enough to stop (tolerance
if(currentHeading<=target+turnTolerance and currentHeading >= target-turnTolerance){
  rALL.stop(hold);
  lALL.stop(hold);
  break; //out of while loop
}
else
//spin motors
{ rALL.spin(reverse, 20*turnDirection, pct); //lower speed
lALL.spin(fwd,20*turnDirection,pct);
}
wait(20,msec);
  }

}


//k: turning values
float kP=0.15; //potential, speed up if there is a longer distance, slow down if shorter (.0655 is within tolerace)
float kI=0; //intergral, not used in turning (keep in 0), faster based on distance travelled
float kD=1; //Derivative, smooth stop, based on speed, faster if high speed

void turnPID(float target){ //target: set turning amount , global on global frame
 float turnIntegral =0; //PID, total distance travelled
 float prevTurnDist=0;
  while(1){

float currentHeading = spin.heading(); //set the current heading
float rTurn = target-currentHeading;
float lTurn = currentHeading-target;
//correct turns if negative
if (lTurn<0){
  lTurn+=360;
}
if (rTurn<0){
  rTurn+=360;
}
//how much degrees you need to turn locally
float turnDistance = 0;
//find most efficent path
if (rTurn<lTurn){
  turnDistance=rTurn;
}
else{
  turnDistance=-lTurn;
}
//check if we are close enough to stop (tolerance
if(currentHeading<=target+turnTolerance and currentHeading >= target-turnTolerance){
  rALL.stop(brake);
  lALL.stop(brake);
  break; //out of while loop
}

//PID calculations:
float turnDerivative=turnDistance-prevTurnDist;
turnIntegral += turnDistance;
float turnSpeed=(kP*turnDistance)+(kI*turnIntegral)+(kD*turnDerivative); 

//spin motors
rALL.spin(reverse, turnSpeed, volt); 
lALL.spin(fwd,turnSpeed,volt); //use volt bc reduce strain on vex brain
//update the prev dist
prevTurnDist=turnDistance;
wait(10,msec);
  }

}


void autonomous(void) {
waitUntil(spin.isCalibrating()==false); // wait for it to stop calibrating
spin.setHeading(0,deg);
// turnNoPID(270);

turnPID(100);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(usercontrol);
  Competition.drivercontrol(autonomous);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
