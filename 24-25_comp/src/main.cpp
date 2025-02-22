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
vex::gps GPSSensor(vex::PORT6, 0);

vex::motor BLMotor(vex::PORT8,vex::gearSetting::ratio6_1);
vex::motor MLMotor(vex::PORT9,vex::gearSetting::ratio6_1);
vex::motor FLMotor(vex::PORT10,vex::gearSetting::ratio6_1);

vex::motor FRMotor(vex::PORT18,vex::gearSetting::ratio6_1);
vex::motor MRMotor(vex::PORT19,vex::gearSetting::ratio6_1);
vex::motor BRMotor(vex::PORT20,vex::gearSetting::ratio6_1);

bool IntakeState = false;
bool SavedState = false;
int insped = 50;
vex::motor IntakeLower(vex::PORT1);
vex::motor IntakeUpperL(vex::PORT12);
vex::motor IntakeUpperR(vex::PORT11);

bool mogoState = false;
vex::pneumatics MogoPiston(Brain.ThreeWirePort.F);
vex::pneumatics FeedExpander(Brain.ThreeWirePort.G);

vex::motor_group LeftDriveGroup(MLMotor, BLMotor, FLMotor);
vex::motor_group RightDriveGroup(MRMotor, BRMotor, FRMotor);


//---------------------------------------------------------------------------//
void Intake(){
  IntakeState = !IntakeState;
      if (IntakeState) {
        IntakeLower.spin(vex::directionType::rev, insped, vex::velocityUnits::pct);
        IntakeUpperL.spin(vex::directionType::rev, insped, vex::velocityUnits::pct);
        IntakeUpperR.spin(vex::directionType::fwd, insped, vex::velocityUnits::pct);
    } else {
        IntakeLower.stop();
        IntakeUpperL.stop();
        IntakeUpperR.stop();
    }
}
void ReverseIntake(){
  IntakeLower.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
  IntakeUpperL.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
  IntakeUpperR.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
}
void StopIntake(){
  IntakeLower.stop();
  IntakeUpperL.stop();
  IntakeUpperR.stop();
  if (IntakeState){
    IntakeLower.spin(vex::directionType::rev, insped, vex::velocityUnits::pct);
    IntakeUpperL.spin(vex::directionType::rev, insped, vex::velocityUnits::pct);
    IntakeUpperR.spin(vex::directionType::fwd, insped, vex::velocityUnits::pct);
  }
}
void RightDrive(){
  while(true){
    double right = Controller.Axis3.position() - Controller.Axis1.position();
    BRMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    MRMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    FRMotor.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
  }
}
void LeftDrive(){
  while(true){
    double left = Controller.Axis3.position() + Controller.Axis1.position();
    BLMotor.spin(vex::directionType::rev, left, vex::velocityUnits::pct);
    MLMotor.spin(vex::directionType::rev, left, vex::velocityUnits::pct);
    FLMotor.spin(vex::directionType::rev, left, vex::velocityUnits::pct);
  }
}
void Mogo(bool state = mogoState){
  if (mogoState){
    MogoPiston.open();
    mogoState = false;
  } else {
    MogoPiston.close();
    mogoState = true;
  }
}
void SpedUp(){
  insped = insped+5;
}
void SpedDown(){
  insped = insped-5;
}
void TogFeedPneumatic(){
  if (FeedExpander.value() == 0){
    FeedExpander.open();
  } else {
    FeedExpander.close();
  }
}
void driveTo(double dist, double targetSpeed) {
  Brain.Screen.clearScreen();
  Brain.Screen.drawRectangle(0, 0, 480, 240, vex::color::red);
    // Constants for slew control
    const double SLEW_RATE = 10.0;  // Increase in speed percentage per 20ms
    const double MIN_SPEED = 20.0;   // Minimum starting speed percentage
    
    // Reset all motor positions
    BLMotor.resetPosition();
    MLMotor.resetPosition();
    FLMotor.resetPosition();
    BRMotor.resetPosition();
    MRMotor.resetPosition();
    FRMotor.resetPosition();
    
    // Initialize current speed
    double currentSpeed = MIN_SPEED;
    
    // Set initial speeds
    LeftDriveGroup.setVelocity(currentSpeed, vex::velocityUnits::pct);
    RightDriveGroup.setVelocity(currentSpeed, vex::velocityUnits::pct);
    
    // Start the motion
    LeftDriveGroup.spinFor(-dist, vex::rotationUnits::deg, currentSpeed, vex::velocityUnits::pct, false);
    RightDriveGroup.spinFor(dist, vex::rotationUnits::deg, currentSpeed, vex::velocityUnits::pct, false);
    
    // Ramp up speed while motors are spinning
    while((BLMotor.isSpinning() || MLMotor.isSpinning() || FLMotor.isSpinning() ||
           BRMotor.isSpinning() || MRMotor.isSpinning() || FRMotor.isSpinning()) && 
           currentSpeed < targetSpeed) {
        Brain.Screen.clearScreen();
        Brain.Screen.drawRectangle(0, 0, 480, 240, vex::color::blue);
        // Increment speed
        currentSpeed = currentSpeed + SLEW_RATE;
        //Controller.Screen.setCursor(1, 1);
        //Controller.Screen.print("Current Speed: %.2f", currentSpeed);
        
        // Update motor velocities
        LeftDriveGroup.setVelocity(currentSpeed, vex::velocityUnits::pct);
        RightDriveGroup.setVelocity(currentSpeed, vex::velocityUnits::pct);
        
        wait(20, msec);  // Small delay to control acceleration rate
    }
    Brain.Screen.clearScreen();
    Brain.Screen.drawRectangle(0, 0, 480, 240, vex::color::yellow);
    // Wait for completion 
    while(BLMotor.isSpinning() || MLMotor.isSpinning() || FLMotor.isSpinning() ||
          BRMotor.isSpinning() || MRMotor.isSpinning() || FRMotor.isSpinning()) {
      if (BLMotor.temperature(vex::temperatureUnits::celsius) >=50 || MLMotor.temperature(vex::temperatureUnits::celsius) >=50 || FLMotor.temperature(vex::temperatureUnits::celsius) >=50 ||
          BRMotor.temperature(vex::temperatureUnits::celsius) >=50 || MRMotor.temperature(vex::temperatureUnits::celsius) >=50 || FRMotor.temperature(vex::temperatureUnits::celsius) >=50){
        break;
      }
      wait(20, msec);
    }
}

void turnToAngle(double target_angle) {
  Brain.Screen.clearScreen();
  Brain.Screen.drawRectangle(0, 0, 480, 240, vex::color::green);
  const double kP = 0.2; // Proportional gain
  const double kD = 0.1; // Derivative gain
  double previous_error = 0;
  //Inert.resetRotation();

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
    if (fabs(error) < 0.75) {
      BLMotor.stop();
      MLMotor.stop();
      FLMotor.stop();
      BRMotor.stop();
      MRMotor.stop();
      FRMotor.stop();
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

int displayGPSData() {
  while(true){
    Brain.Screen.clearScreen();
    // Get GPS sensor data
    // Get GPS sensor data
    double x_position = GPSSensor.xPosition(mm);
    double y_position = GPSSensor.yPosition(mm);
    double heading = GPSSensor.heading();
    int sped = insped;
    // Print GPS data to the brain screen
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: %.2f mm", x_position);
    Brain.Screen.newLine();
    Brain.Screen.print("Y: %.2f mm", y_position);
    Brain.Screen.newLine();
    Brain.Screen.print("Heading: %.2f degrees", heading);
    Brain.Screen.newLine();
    Brain.Screen.print("Left Temp: %.2f C", IntakeUpperL.temperature(vex::temperatureUnits::celsius));
    Brain.Screen.newLine();
    Brain.Screen.print("Right Temp: %.2f C", IntakeUpperR.temperature(vex::temperatureUnits::celsius));
    Brain.Screen.newLine();
    Brain.Screen.print("Mogo State: %s", mogoState ? "Closed, flase" : "Open, true");
    Brain.Screen.newLine();
    Brain.Screen.print("Feed Speed: %.0f", sped);

    wait(500, msec); // Update the screen every 500 milliseconds
  }
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

void auton() {
driveTo(-1400, 100);
Mogo(false); // open
turnToAngle(40); // turn to goal
driveTo(-980, 100); // drive to mogo
Mogo(true); //close
driveTo(1000,100); // drive out from latter
FeedExpander.open();
Intake();
turnToAngle(45); // turn to first ring
driveTo(1500, 100); //drive to first ring
turnToAngle(290); // turn next ring
driveTo(2200, 100); //drive to 2nd ring
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
  vex::task displayTask(displayGPSData);
  thread driveRight(RightDrive);
  thread driveLeft(LeftDrive);
  FeedExpander.open();
  // User control code here, inside the loop
  insped = 100;
  while (1) {
    double current_angle = Inert.rotation(vex::rotationUnits::deg);
    //Display motor values to screen
    //Brain.Screen.setCursor(1, 1);
    //Brain.Screen.print("BLMotor: %f", BLMotor.position(deg));
    //Brain.Screen.setCursor(2, 1);
    //Brain.Screen.print("MLMotor: %f", MLMotor.position(deg));
    //Brain.Screen.setCursor(3, 1);
    //Brain.Screen.print("FLMotor: %f", FLMotor.position(deg));
    //Brain.Screen.setCursor(4, 1);
    //Brain.Screen.print("BRMotor: %f", BRMotor.position(deg));
    //Brain.Screen.setCursor(5, 1);
    //Brain.Screen.print("MRMotor: %f", MRMotor.position(deg));
    //Brain.Screen.setCursor(6, 1);
    //Brain.Screen.print("FRMotor: %f", FRMotor.position(deg));
    Controller.Screen.setCursor(1, 1);
    //Controller.Screen.print("Current Angle: %.2f", current_angle);
    Controller.Screen.print("Feed Speed: %.0f", insped);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);
  Controller.ButtonR2.pressed(ReverseIntake);
  Controller.ButtonR2.released(StopIntake);
  Controller.ButtonR1.pressed(Intake);
  Controller.ButtonB.pressed([]{ Mogo(mogoState); });
  Controller.ButtonUp.pressed(SpedUp);
  Controller.ButtonDown.pressed(SpedDown);
  Controller.ButtonL1.pressed(TogFeedPneumatic);

  // Run the pre-autonomous function.
  pre_auton();


  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
