/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nolanhenderson                                            */
/*    Created:      1/26/2025, 4:06:50 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
vex::controller Controller;
vex::brain Brain;
vex::inertial Inert(vex::PORT7);

vex::motor BLMotor(vex::PORT8,vex::gearSetting::ratio6_1);
vex::motor MLMotor(vex::PORT9,vex::gearSetting::ratio6_1);
vex::motor FLMotor(vex::PORT10,vex::gearSetting::ratio6_1);

vex::motor FRMotor(vex::PORT18,vex::gearSetting::ratio6_1);
vex::motor MRMotor(vex::PORT19,vex::gearSetting::ratio6_1);
vex::motor BRMotor(vex::PORT20,vex::gearSetting::ratio6_1);

bool IntakeState = false;
vex::motor IntakeLower(vex::PORT1);
vex::motor IntakeUpperL(vex::PORT12);
vex::motor IntakeUpperR(vex::PORT11);



//---------------------------------------------------------------------------//
void driveTo(int R, int L, int speed) {
  BLMotor.spinToPosition(L, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  MLMotor.spinToPosition(L, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  FLMotor.spinToPosition(L, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  BRMotor.spinToPosition(R, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  MRMotor.spinToPosition(R, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
  FRMotor.spinToPosition(R, vex::rotationUnits::rev, speed, vex::velocityUnits::pct, false);
}
void turnToAngle(double target_angle) {
  const double kP = 0.1; // Proportional gain
  const double kD = 0.3; // Derivative gain
  double previous_error = 0;
  Inert.resetRotation();

  while (true) {
    double current_angle = Inert.rotation(vex::rotationUnits::deg);
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Current Angle: %.2f", current_angle);
    double error = target_angle - current_angle;
    double derivative = error - previous_error;
    double turn_speed = (error * kP) + (derivative * kD);

    // Set motor speeds for turning
    BLMotor.spin(vex::directionType::rev, turn_speed, vex::velocityUnits::pct);
    MLMotor.spin(vex::directionType::rev, turn_speed, vex::velocityUnits::pct);
    FLMotor.spin(vex::directionType::rev, turn_speed, vex::velocityUnits::pct);

    BRMotor.spin(vex::directionType::rev, turn_speed, vex::velocityUnits::pct);
    MRMotor.spin(vex::directionType::rev, turn_speed, vex::velocityUnits::pct);
    FRMotor.spin(vex::directionType::rev, turn_speed, vex::velocityUnits::pct);

    // Break the loop if the robot is close enough to the target angle
    if (fabs(error) < 1 || current_angle > target_angle) {
      break;
    }

    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }

  // Stop the motors
  BLMotor.stop();
  MLMotor.stop();
  FLMotor.stop();
  BRMotor.stop();
  MRMotor.stop();
  FRMotor.stop();
}

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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
    int left = Controller.Axis3.position();
    int right = Controller.Axis2.position();

    BLMotor.spin(vex::directionType::rev, left, vex::velocityUnits::pct);
    MLMotor.spin(vex::directionType::rev, left, vex::velocityUnits::pct);
    FLMotor.spin(vex::directionType::rev, left, vex::velocityUnits::pct);
    BRMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    MRMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    FRMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);

    if (Controller.ButtonR1.pressing()) {
      IntakeState = !IntakeState;
      if (IntakeState) {
        IntakeLower.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
        IntakeUpperL.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
        IntakeUpperR.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    } else {
      IntakeLower.stop();
      IntakeUpperL.stop();
      IntakeUpperR.stop();
    }
    wait(200, msec);
    }

    if(Controller.ButtonR2.pressing())
    {
      //DriveTo(5000,5000,50);
      turnToAngle(90);
      wait(200, msec);
    }
    //Display motor values to screen
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("BLMotor: %d", BLMotor.position(rev));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("MLMotor: %d", MLMotor.position(rev));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("FLMotor: %d", FLMotor.position(rev));
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("BRMotor: %d", BRMotor.position(rev));
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("MRMotor: %d", MRMotor.position(rev));
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("FRMotor: %d", FRMotor.position(rev));
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
