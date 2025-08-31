#include "robot-config.h"
/*---------------------------------------------------------------------------*/
/* */
/* Description: Competition template for VCS VEX V5 */
/* */
/*---------------------------------------------------------------------------*/
// Creates a competition object that allows access to Competition methods.
vex::competition competiton;
bool in_autonomous = false;
bool intakeSpinning = false;
bool clampActive = false;
bool motorBrake = false;
bool LBBrake = false;
bool LB_PID_turned_on = false;
bool intakeUp = false;
bool doinkerDown = false;
char LB_state = 'R';
int motor_speed = 100;
int intake_speed = 80;
int lb_speed = 100;
int intakeLiftCooldown = 0;
int clampCooldown = 0;
int LBtoggleCooldown = 0;
int doinkerCooldown = 0;




//pneumatics
digital_out mogoClamp = digital_out(Brain.ThreeWirePort.A);
digital_out Doinker = digital_out(Brain.ThreeWirePort.B);
digital_out IntakeLift = digital_out(Brain.ThreeWirePort.C);
//motor groups
motor_group right_drive(RightFMotor, RightUMotor, RightBMotor);
motor_group left_drive(LeftFMotor, LeftUMotor, LeftBMotor);
motor_group full_drive(RightFMotor, RightUMotor, RightBMotor, LeftFMotor, LeftUMotor, LeftBMotor);




/*
User defined function move_forward()
Moves the robot forward
*/
void move_forward()
{
full_drive.spin(vex::directionType::fwd, motor_speed, vex::velocityUnits::pct);
}
/*
User defined functon move_forward()
Moves the robot backward
*/
void move_backward()
{
full_drive.spin(vex::directionType::rev, motor_speed, vex::velocityUnits::pct);
}
/*
User defined function turn_right()
Turns the robot right
*/
void turn_right()
{
left_drive.spin(vex::directionType::fwd, motor_speed, vex::velocityUnits::pct);
right_drive.spin(vex::directionType::rev, motor_speed, vex::velocityUnits::pct);
}
/*
User defined function turn_right()
Turns the robot right
*/
void turn_left()
{
left_drive.spin(vex::directionType::rev, motor_speed, vex::velocityUnits::pct);
right_drive.spin(vex::directionType::fwd, motor_speed, vex::velocityUnits::pct);
}
/*
User defined function stop_robot()
Stops the robot
*/
void stop_robot()
{/*************////
full_drive.stop();
}
/*
User defined function brake_robot()
brakes the robot
*/
void brake_robot(double x)
{
full_drive.stop(brakeType::hold);
task::sleep(x);
}




/*
User defined function spin_intakes(char c, int speed)
intakes any combination of upper or lower intake at a given speed
*/
void spin_intakes(char c, int speed) //char can be U, L, or B
{
 if (c == 'U')
 {
   UpperIntake.spin(fwd, speed, pct);
 }
 else if (c == 'L')
 {
   LowerIntake.spin(fwd, speed, pct);
 }
 else if (c == 'B')
 {
   UpperIntake.spin(fwd, speed, pct);
   LowerIntake.spin(fwd, speed, pct);
 }
}




/*
User defined function doinker_down()
drops the doinker
*/
void doinker_down()
{
 doinkerDown = true;
 Doinker.set(true);
}




/*
User defined function doinker_up()
lifts the doinker
*/
void doinker_up()
{
 doinkerDown = false;
 Doinker.set(false);
}




/*
User defined function clamp_down()
mogo clamp clamps
*/
void clamp_down()
{
 clampActive = true;
 mogoClamp.set(true);
}


/*
User defined function clamp_up()
mogo clamp unclamps
*/
void clamp_up()
{
 clampActive = false;
 mogoClamp.set(false);
}




/*
User defined function lift_intake()
intake lifts up
*/
void lift_intake()
{
IntakeLift.set(true);
intakeUp = true;
}




/*
User defined function lower_intake()
intake lifts up
*/
void lower_intake()
{
IntakeLift.set(false);
intakeUp = false;
}




//rotation based functions
//turns with precise values
void precise_turn(double x, double s, bool w)
{
left_drive.spinToPosition(x, rotationUnits::rev, s, velocityUnits::rpm, false);
right_drive.spinToPosition(-x, rotationUnits::rev, s, velocityUnits::rpm, w);
}
//moves forward with rotations
void precise_move(double x, double s, bool w)
{
full_drive.spinToPosition(x, rotationUnits::rev, s, velocityUnits::rpm, w);
}






/*---------------------------------------------------------------------------*/
/* Pre-Autonomous Functions */
/* */
/* You may want to perform some actions before the competition starts. */
/* Do them in the following function. You must return from this function */
/* or the autonomous and usercontrol tasks will not be started. This */
/* function is only called once after the cortex has been powered on and */
/* not every time that the robot is disabled. */
/*---------------------------------------------------------------------------*/




void calibrateInertialSensor()
{
Inertial.calibrate();


while (Inertial.isCalibrating())
{
  Controller1.Screen.clearLine(3);
  Controller1.Screen.print("calibrating...");
}
Controller1.Screen.clearLine(3);
Controller1.Screen.print("done!");
Controller1.Screen.clearLine(3);
Controller1.rumble("...");
wait(10, msec);
}












void pre_auton(void) { //resets all positions and calibrates the inertial sensor
LBsensor.resetPosition();
LadyBrown.resetPosition();
calibrateInertialSensor();
};






//GRAPHING ONTO BRAIN
#define GRAPH_WIDTH 480  // Width of the Brain screen
#define GRAPH_HEIGHT 240 // Height of the Brain screen
#define GRAPH_MAX_ERROR 60  // Max error value to scale the graph
#define DATA_POINTS 120  // Number of data points on the graph


double errorHistory[DATA_POINTS] = {0};  // Array to store the error history
int errorIndex = 0;  // Index for the next error point






void drawGraph() {
   Brain.Screen.clearScreen();
   Brain.Screen.setPenColor(vex::color::green);
   Brain.Screen.setFillColor(vex::color::black);


   // Draw the axes
   Brain.Screen.drawLine(0, GRAPH_HEIGHT / 2, GRAPH_WIDTH, GRAPH_HEIGHT / 2);  // X-axis
   Brain.Screen.drawLine(0, 0, 0, GRAPH_HEIGHT);  // Y-axis


   // Plot error values
   for (int i = 1; i < DATA_POINTS; i++) {
       int x1 = (i - 1) * (GRAPH_WIDTH / DATA_POINTS);
       int y1 = GRAPH_HEIGHT / 2 - (errorHistory[i - 1] / GRAPH_MAX_ERROR) * (GRAPH_HEIGHT / 2);
       int x2 = i * (GRAPH_WIDTH / DATA_POINTS);
       int y2 = GRAPH_HEIGHT / 2 - (errorHistory[i] / GRAPH_MAX_ERROR) * (GRAPH_HEIGHT / 2);


       Brain.Screen.drawLine(x1, y1, x2, y2);  // Connect the points
   }
}






































/////////////////////////////PID
//Settings (Constants)
double kP = 0.78;
double kI = 0.35;
double kD = 0.575;


double turnkP = 0.134;
double turnkI = 0.011; //0.011
double turnkD = 0.11;


double currentkP = kP; //tracks for something that was not working in the next PID move
double currentkI = kI;
double currentkD = kD;


double currentTurnkI = turnkI;


/* old settings
double kP = 0.049;
double kI = 0.01;
double kD = 0.025;


double turnkP = 0.134;
double turnkI = 0.011;
double turnkD = 0.106;
*/


//Autonomous Settings
int desiredValue = 0;
int desiredTurnValue = 0;
double maxDriveSpeed = 1.00;
double maxTurnSpeed = 1.00;


double error; //SensorValue - DesiredValue : position
double prevError = 0; //Position 20 miliseconds
double derivative; // error - prevError : Speed
double totalError = 0; //totalError = totalError + error


double turnError; //SensorValue - DesiredValue : position
double turnPrevError = 0; //Position 20 miliseconds
double turnDerivative; // error - prevError : Speed
double turnTotalError = 0; //totalError = totalError + error


//reset variables
bool resetDriveSensors = false;
bool resetTurnSensors = false;


//Variables modified for use
bool enableDrivePID = false;


//variables for tracking time and if settled
bool isSettled = false;
int PIDrunTime = 0;
int PIDsettleTime = 0;
int minSettleTime = 0;
double driveSettleMargin = 0;
double rotationSettleMargin = 0;


//drive PID loop
int drivePID() {


//disables if necessary
while (enableDrivePID) {
if (resetDriveSensors) { //resets all motor values of drive motors
  resetDriveSensors = false;
  LeftFMotor.setPosition(0, degrees);
  LeftBMotor.setPosition(0, degrees);
  LeftUMotor.setPosition(0, degrees);
  RightFMotor.setPosition(0, degrees);
  RightBMotor.setPosition(0, degrees);
  RightUMotor.setPosition(0, degrees);
  Parallel_Odom.setPosition(0, degrees);
}


if (resetTurnSensors) { // resets inertial sensor rotation value
  resetTurnSensors = false;
  Inertial.resetRotation();
}


//////////////////////////////////////////
//Lateral movement PID using Parallel_Odom
////////////////////////////////////////////////////////////////////////////////
//Get the position from the parallel odometry encoder
double rawLateralPosition = Parallel_Odom.position(deg); //gets position in degrees rotated
double convertedLateralPosition = (rawLateralPosition/360) * 2.75 * 3.1415; //converts raw degrees into inches travelled


//Potential
error = desiredValue - convertedLateralPosition; //distance from target


//Derivative
derivative = error - prevError; //instantaneous change in velocity - between consecutive iterations of PID loop


//Integral
if (error<1 && error>-1 && !motorBrake) // if statement to only accumalate totalError when close to target
  totalError += error; //accumalation


if ((prevError<=0 && error>=0) || (prevError>=0 && error<=0)) //if target is crossed, reset accumulation
  totalError = 0;


double lateralMotorPower = error * kP + derivative * kD; //initial sum for lateral motor power
if (totalError * kI > 0.9) //cap for integral's influence on total motor power
  lateralMotorPower += 0.9;
else
  lateralMotorPower += totalError * kI;
////////////////////////////////////////////////////////////////////////////////
if (lateralMotorPower > (11.0*maxDriveSpeed)) { //if statememt to cap max lateralMotorPower to control PID drive speed
  lateralMotorPower = 11.0 * maxDriveSpeed;
}
if (lateralMotorPower < -(11.0*maxDriveSpeed)) {
  lateralMotorPower = -11.0 * maxDriveSpeed;
}






//////////////////////////////////////////
//Turning movement PID
////////////////////////////////////////////////////////////////////////////////
//int turnDifference = leftDrivePosition - rightDrivePosition; //old version
//Potential
//turnError = turnDifference - desiredTurnValue; //old version
//double convertedTurnPosition = Perp_Odom.position(deg); //figure out how many degrees Perp_Odom rotates for the robot to rotate 360 degrees
turnError = desiredTurnValue - Inertial.rotation();


//Derivative
turnDerivative = turnError - turnPrevError; //instantaneous rotational velocity


//Integral
if (turnError<10 && turnError>-10 && !motorBrake) //build up of error while close to the target
  turnTotalError += turnError; //accumulation


if ((turnError<=0 && turnPrevError>=0) || (turnError>=0 && turnPrevError<=0)) //if target is crossed, reset the I accumulation
  turnTotalError = 0;


double turnMotorPower = turnError * turnkP + turnDerivative * turnkD; //initial sum of the turnMotorPower


if (turnTotalError * turnkI > 1.15) //limit to I's influence
  turnMotorPower += 1.15;
else
  turnMotorPower += turnTotalError * turnkI;
  ////////////////////////////////////////////////////////////////////////////////
if (turnMotorPower > (11.0*maxTurnSpeed)) { //limits the max turning speed
  turnMotorPower = 11.0 * maxTurnSpeed;
}
if (turnMotorPower < -(11.0*maxTurnSpeed)) {
  turnMotorPower = -11.0 * maxTurnSpeed;
}




if (!motorBrake) //force brake if necessary
{
  left_drive.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
  right_drive.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
}
else {
  left_drive.stop(vex::brakeType::hold);
  right_drive.stop(vex::brakeType::hold);
}
Controller1.Screen.clearLine(3);
Controller1.Screen.print(turnError); //to track the error




// Update error history
errorHistory[errorIndex] = turnError;  // Store the current error
errorIndex = (errorIndex + 1) % DATA_POINTS;  // Move to the next index


// Draw the graph
drawGraph();




//updates previous trackers
prevError = error;
turnPrevError = turnError;


//time updates
if (fabs(error) < driveSettleMargin/2 && fabs(turnError) < rotationSettleMargin/2)
  PIDsettleTime += 10;
else
  PIDsettleTime = 0;
PIDrunTime += 10; //increments this always


//updates settle status
if (!isSettled)
  if (PIDsettleTime > minSettleTime)
    isSettled = true; //confirms that robot has settled long enough and updates boolean


task::sleep(10);
}


return 1;
}






/////////////////////////////FUNCTIONS TO AUTOMATE PID
//turn the drive PID off
void drivePIDdisable()
{
 kP = 0;
 kI = 0;
 kD = 0;
}


//turn the drive PID on
void drivePIDenable()
{
 kP = currentkP;
 kI = currentkI;
 kD = currentkD;
}


//turns the kI off
void turnIdisable()
{
 turnkI = 0;
}


//turns the kI on
void turnIenable()
{
 turnkI = currentTurnkI;
}






//function that automates drive PID
void nextDrivePIDmove(double lateralMovement, double DriveSpeed)
{
desiredValue += lateralMovement; //increments desiredValue and desiredTurn Value
maxDriveSpeed = DriveSpeed; //updates max speeds


kP = currentkP;
kI = currentkI;
kD = currentkD;


turnkI = 0;
}




//function that automates drive PID
void nextTurnPIDmove(double turnMovement, double turnSpeed)
{
desiredTurnValue += turnMovement; //increments desiredValue and desiredTurn Value
maxTurnSpeed = turnSpeed; //updates max speeds


kP = 0;
kI = 0;
kD = 0;


turnkI = currentTurnkI;
}










//commented out the loop itself to test motor command based lady brown task
//LADY BROWN PID
//Settings (Constants)
double LBkP = 0.25; //originally 0.23
double LBkI = 0.0; //originally 0.00
double LBkD = 0.01; //originally 0.01


//Changeable Settings
double desiredLBposition = 0;
double maxLBspeed = 1;


double LBerror; //SensorValue - DesiredValue : position
double LBprevError = 0; //Position 20 miliseconds
double LBderivative; // error - prevError : Speed
double LBtotalError = 0; //totalError = totalError + error


bool resetLBsensor = false;
//Variables modified for use
bool enableLB_PID = false;
////heres the commented out section


//lady brown PID loop
int LB_PID() {


//disables if necessary
while (enableLB_PID) {


  if (resetLBsensor) { //resets the rotational sensor attached to the lady brown
    resetLBsensor = false;
    LBsensor.resetPosition();
  }


  //Get the position of motors
  double LBposition = LBsensor.position(deg);
  //////////////////////////////////////////
  //LB PID
  ////////////////////////////////////////////////////////////////////////////////
  //Potential
  LBerror = desiredLBposition - LBposition; //angle away from target


  //Derivative
  LBderivative = LBerror - LBprevError; //velocity - uses between consecutive iterations of PID loop


  //Integral
  if (LBerror<8 && LBerror>-8 && !LBBrake) // if statement to only accumalate totalError when close to target
    LBtotalError += LBerror; //accumalation


  if ((LBprevError<=0 && LBerror>=0) || (LBprevError>=0 && LBerror<=0)) //of the ladybrown crosses over the target, reset the total error accumulated
    LBtotalError = 0;


  double LBMotorPower = LBerror * LBkP + LBderivative * LBkD; //initial sum for lateral motor power
  if (LBtotalError * LBkI > 1) //cap for integral's influence on total motor power
    LBMotorPower += 1;
  else
    LBMotorPower += LBtotalError * LBkI;
  ////////////////////////////////////////////////////////////////////////////////
  if (LBMotorPower > (11.0*maxLBspeed)) { //if statememt to cap max lateralMotorPower to control PID drive speed
    LBMotorPower = 11.0 * maxLBspeed;
  }
  if (LBMotorPower < -(11.0*maxLBspeed)) {
    LBMotorPower = -11.0 * maxLBspeed;
  }




  if (!LBBrake)
  {
    LadyBrown.spin(forward, LBMotorPower, voltageUnits::volt);
  }
  else {
    LadyBrown.stop(vex::brakeType::hold);
  }
  Controller1.Screen.clearLine(3);
  //Controller1.Screen.print(LBerror); //to track the error
  //code
  LBprevError = LBerror; //update of the previous iteration's variable
  task::sleep(20); //sleep to save resources
}


return 1;
}








/*
//motor command lady brown task
bool enableLBLoop = false;
char currentLBstate = 'R';
int LadyBrownLoop()
{
while (enableLBLoop) //can be turned off completely
{
  if (currentLBstate != LB_state) //only occurs if position changes
  {
    currentLBstate = LB_state; //updates current variable
    if (LB_state == 'R' || LB_state == 'S')
      LadyBrown.spinToPosition(desiredLBposition, deg, maxLBspeed * 200, rpm, false); //spins to position
    else if (LB_state == 'L') //holds if it is in loading or scoring state
    {
      LadyBrown.spinToPosition(desiredLBposition, deg, maxLBspeed * 200, rpm, true);
      LadyBrown.stop(hold);
    }
  }


  task::sleep(20); //sleep to save resources
}


return 1;
}
*/




/*---------------------------------------------------------------------------*/
/* */
/* Autonomous Task */
/* */
/* This task is used to control your robot during the autonomous phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/
// ..........................................................................
// Insert autonomous user code here.
// ..........................................................................
void autonomous(void) {
in_autonomous = true;
//standerdized measurements
//precise_turn(1.43, 50, true); for precise 180 turn


enableDrivePID = true;
resetDriveSensors = true;
resetTurnSensors = true;
LadyBrown.resetPosition();
maxDriveSpeed = 1;
maxTurnSpeed = 1;
maxLBspeed = 1;


//starting the PIDs
task(drivePID, 3);
task(LB_PID, 4); //non pid lady brown loop
LB_PID_turned_on = true;
enableLB_PID = true;
//enableLBLoop = true; // turns on the LB mechanism


//nextPIDmove(double lateralMovement, double DriveSpeed, double rotationalMovement, double rotationSpeed)
//dSM is typically 1
//rSM is typically 5


//AUTONS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////RED SIDE RUSH WINPOINT
/////////////////////////SLOT 1


nextDrivePIDmove(33, 1); //initial dash with intake spinning and a large margin so that the doinker drops and grabs the goal
desiredLBposition = 400/5; //drops the intake
LB_state = 'S';
spin_intakes('L', 100);
task::sleep(200);
desiredLBposition = 0; //bromgs it back down
LB_state = 'R';


task::sleep(425);
doinker_down(); //drops the doinker
task::sleep(350); //time for it to drop


nextDrivePIDmove(-13, 1); //whips the goal back
task::sleep(500);
doinker_up(); //releases the goal
task::sleep(200);


nextTurnPIDmove(167, 1); //turns around
task::sleep(1000);


nextDrivePIDmove(-15, 0.6); //goes to clamp the goal
task::sleep(700);
clamp_down();
task::sleep(100);


nextTurnPIDmove(180, 1); //turns the goal around
task::sleep(1100);


nextDrivePIDmove(-38.5, 0.8); //drives the goal to the wall
task::sleep(200);
spin_intakes('B', 100);//Set back to 100
task::sleep(450);
spin_intakes('U', 0); //CHANGE THIS IF NECESSARY
task::sleep(500);
clamp_up();
task::sleep(50);
nextDrivePIDmove(7.5, 0.8); //moves away from the goal
task::sleep(650);


nextTurnPIDmove(135, 1); //turns to the next goal
task::sleep(900);


nextDrivePIDmove(-23, 0.6); //grabs the goal
task::sleep(850);
clamp_down();
spin_intakes('B', 100);
task::sleep(100);


nextTurnPIDmove(77, 1); //turns toward ring 3
task::sleep(800);


nextDrivePIDmove(27, 0.8); //goes and picks up ring 3
lift_intake(); //lifts and lowers it to pick up the ring
spin_intakes('U', 0);
task::sleep(700);
lower_intake();
task::sleep(400);


nextDrivePIDmove(-30, 1); //drives back
task::sleep(1200);


nextTurnPIDmove(-94, 1); //turns to the ladder
task::sleep(700);


nextDrivePIDmove(35, 1); //goes to the ladder
doinker_down();
task::sleep(1300);


























//////////////////////////////BLUE SIDE RUSH WINPOINT
/////////////////////////SLOT 2
/*
desiredLBposition = 400/5; //intake drop
LB_state = 'S';


spin_intakes('L', 100); //initial dash with intake spinning and a large margin so that the doinker drops and grabs the goal
nextDrivePIDmove(34.5, 1);
task::sleep(700);
doinker_down();
desiredLBposition = 0; //lowers the LB back down
LB_state = 'R';
task::sleep(350); //the time it takes for the doinker to complete dropping


nextDrivePIDmove(-15.1, 1); //whips the goal back
task::sleep(550);
doinker_up(); //releases the goal
task::sleep(175);


nextTurnPIDmove(173, 1); //spins the back of the robot to the goal
task::sleep(950);


nextDrivePIDmove(-16.6, 0.6); //goes and clamps the goal
task::sleep(750);
clamp_down();
task::sleep(250);


nextTurnPIDmove(-214, 1); //turns around to prepare for backing up and releasing the mogo to a safe position near our corner
task::sleep(950);
spin_intakes('B', 100); //starts intaking the two rings into the goal
task::sleep(350);


nextDrivePIDmove(-36, 0.8); //drives forward to the drop off location and releases
task::sleep(1000);
clamp_up(); //releases it
task::sleep(250);


nextDrivePIDmove(7, 0.8); //moves away a little
task::sleep(500);
spin_intakes('B', 0); //turns off intakes


nextTurnPIDmove(-108, 1); //turns and faces the other mobile goal
task::sleep(950);


nextDrivePIDmove(-30.5, 0.6); //clamps the goal
task::sleep(1150);
clamp_down();
task::sleep(100);
nextDrivePIDmove(3, 1);
task::sleep(500);


nextTurnPIDmove(-91, 1); //turns to face stack of two
task::sleep(800);


nextDrivePIDmove(30, 0.5); //lifts intake and goes to stop directly over the stack of two rings
spin_intakes('B', 100);
lift_intake();
task::sleep(1050);
lower_intake();
task::sleep(200);
*/




//NEW CODE
/*
nextDrivePIDmove(-5, 0.8); //drives back
task::sleep(500);


nextTurnPIDmove(110, 1); //turns to the corner
task::sleep(700);


nextDrivePIDmove(40, 1); //goes to corner
task::sleep(200);
doinker_down();
task::sleep(1000);




nextTurnPIDmove(120, 1); //ends with the turn
task::sleep(500);


nextDrivePIDmove(20, 1);
*/






//OLD CODE
/*
nextDrivePIDmove(-30, 0.8); //reverses back while picking up one ring
task::sleep(1300);


nextTurnPIDmove(-87, 1); //turns to the ladder
desiredLBposition = 655/5;
LB_state = 'S';
task::sleep(850);


nextDrivePIDmove(11, 0.5); //touches ladder with zip ties and finishes
*/












//////////////////////////////RED SIDE NON-RUSH WINPOINT
/////////////////////////SLOT 3
/*
desiredLBposition = 655/5; //ramps the lady brown up to scoring position to score on the alliance stake
LB_state = 'S';
spin_intakes('L', 100);
task::sleep(500);
nextDrivePIDmove(29, 0.8); //drives the ring into the alliance stake while driving backwards to line up with the first mobile goal
task::sleep(500);
desiredLBposition = 0/5; //brings the lady brown back down
LB_state = 'R';


task::sleep(600);


nextTurnPIDmove(-113, 1); //turns to the mobile goal
task::sleep(900);
spin_intakes('L', -100);


nextDrivePIDmove(-34, 0.6); //Goes and picks up the first mobile goal
task::sleep(1150);
clamp_down();
task::sleep(100);


nextTurnPIDmove(98, 1); //turns toward ring 2
task::sleep(800);


nextDrivePIDmove(19, 0.8); //goes and picks up ring 2
spin_intakes('B', 100); //starts the intakes
task::sleep(1050);


nextTurnPIDmove(98, 1); //turns to ring 3
task::sleep(800);


nextDrivePIDmove(15.5, 0.6); //goes and picks up the ring
task::sleep(850);


nextDrivePIDmove(-14, 0.8); //drives back a bit before turning to ring 4
task::sleep(450);
spin_intakes('U', 0); //stops the upper intake
task::sleep(250);


nextTurnPIDmove(-31, 1); //turns to ring 4
task::sleep(500);


nextDrivePIDmove(14.5, 0.6); //goes and picks up the ring
spin_intakes('B', 100);
task::sleep(800);


nextDrivePIDmove(-5, 0.8); //drives back a bit before turning to ring 5
task::sleep(500);


nextTurnPIDmove(151, 1); //turns to ring 5
task::sleep(1300);


nextDrivePIDmove(46, 0.8); //drives onto ring 5
lift_intake();
task::sleep(100);
spin_intakes('B', 100);
task::sleep(1200);
lower_intake();
task::sleep(200);
desiredLBposition = 655/5;
LB_state = 'S';


nextDrivePIDmove(-29, 1); //drives back to go to the ladder
task::sleep(100);
spin_intakes('U', 0);
task::sleep(800);


nextTurnPIDmove(-75, 1); //turns to ladder
task::sleep(400);
spin_intakes('B', 100);
task::sleep(100);


nextDrivePIDmove(23, 0.5); //goes and touches the ladder
*/










//////////////////////////////BlUE SIDE NON-RUSH WINPOINT
/////////////////////////SLOT 4
/*
desiredLBposition = 655/5; //ramps the lady brown up to scoring position to score on the alliance stake
LB_state = 'S';
task::sleep(350);
nextDrivePIDmove(11, 0.8); //drives the ring into the alliance stake
task::sleep(600);
nextDrivePIDmove(-19, 0.8); //drives to lineup with the goal
desiredLBposition = 0/5; //brings the lady brown back down
LB_state = 'R';
task::sleep(950);


nextTurnPIDmove(-67, 1); //turns to the mobile goal
task::sleep(700);


nextDrivePIDmove(-33, 0.6); //Goes and picks up the first mobile goal
task::sleep(1150);
clamp_down();
task::sleep(100);


nextTurnPIDmove(-95, 1); //turns toward ring 2
task::sleep(800);


nextDrivePIDmove(18, 0.8); //goes and picks up ring 2
spin_intakes('B', 100); //starts the intakes
task::sleep(1050);


nextTurnPIDmove(-98, 1); //turns to ring 3
task::sleep(800);


nextDrivePIDmove(15, 0.6); //goes and picks up the ring
task::sleep(850);


nextDrivePIDmove(-14, 0.8); //drives back a bit before turning to ring 4
task::sleep(450);
spin_intakes('U', 0); //stops the upper intake
task::sleep(250);


nextTurnPIDmove(31, 1); //turns to ring 4
task::sleep(500);


nextDrivePIDmove(14.5, 0.6); //goes and picks up the ring
spin_intakes('B', 100);
task::sleep(800);


nextDrivePIDmove(-5, 0.8); //drives back a bit before turning to ring 5
task::sleep(500);


nextTurnPIDmove(-152, 1); //turns to ring 5
task::sleep(1150);


nextDrivePIDmove(49, 0.8); //drives onto ring 5
lift_intake();
task::sleep(100);
spin_intakes('B', 100);
task::sleep(1200);
lower_intake();
task::sleep(200);
desiredLBposition = 655/5;
LB_state = 'S';


nextDrivePIDmove(-28, 1); //drives back to go to the ladder
task::sleep(100);
spin_intakes('U', 0);
task::sleep(950);


nextTurnPIDmove(75, 1); //turns to the ladder
task::sleep(400);
spin_intakes('B', 100);
task::sleep(200);


nextDrivePIDmove(21, 1); //goes and touches the ladder
*/
















//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////SKILLS
/////////////////////////SLOT 5
/*
desiredLBposition = 655/5; //ramps the lady brown up to scoring position to score on the alliance stake
LB_state = 'S';
task::sleep(450);
nextDrivePIDmove(6, 1); //drives the ring into the alliance stake
task::sleep(500);


nextDrivePIDmove(-29, 0.6); //moves toward the first mobile goal
task::sleep(1000);
desiredLBposition = 400/5; //drops the intake
LB_state = 'S';
clamp_down(); // clamps the goal
task::sleep(200);


nextTurnPIDmove(98, 1); //turns to ring 1 close to the ladder
task::sleep(400);
desiredLBposition = 0;
LB_state = 'R'; //lowers the lady brown back down
task::sleep(500);
spin_intakes('B', 100); //turns on intake


nextDrivePIDmove(16.5, 1); //goes to the ring to intake it
task::sleep(950);


nextTurnPIDmove(40, 1); //turns toward ring 2 across the field
task::sleep(600);


nextDrivePIDmove(50, 1); //goes and picks up ring 2
task::sleep(1000);
nextDrivePIDmove(12, 0.8);
task::sleep(800);
nextDrivePIDmove(-21.5, 1); //backs up to get a better angle at the third ring
task::sleep(1000);


nextTurnPIDmove(86, 1); //turns toward ring 3 which will be lady brown'd
task::sleep(900);
spin_intakes('U', 0); //JUST NOW ADDED THIS
//desiredLBposition = 120/5; //sets the lady brown into the loading position
//LB_state = 'L';


nextDrivePIDmove(13, 1); //picks up ring 3
task::sleep(900);


nextTurnPIDmove(-21, 1); //turns toward the wall stake
task::sleep(700);


nextDrivePIDmove(40, 0.8); //goes into the wall stake and uses it to correct
task::sleep(500);
spin_intakes('U', 0);
//desiredLBposition = 655/5; //launches the lady brown and scores it
//UpperIntake.spinFor(-15, deg, 100, rpm, true); //reverses the intake a little bit
//LB_state = 'S';
task::sleep(750);
spin_intakes('B', -100); //JUST NOW ADDED THIS
task::sleep(100);
resetDriveSensors = true; //resets the sensors at the ladybrown where it self corrects
desiredValue = 0; //resets the target value as well


nextDrivePIDmove(-12, 0.5); //moves back to line up with the line of 3 rings
task::sleep(900);


nextTurnPIDmove(88, 1); //turns to face the line of 3 rings
desiredLBposition = 0; //lowers lady brown
LB_state = 'R';
task::sleep(950);
spin_intakes('B', 100); //spins the intakes


nextDrivePIDmove(55.5, 0.7); //picks up the 3 rings
task::sleep(1800);


nextDrivePIDmove(-21.5, 0.9); // backs up from the wall a little bit
task::sleep(1050);


nextTurnPIDmove(-42, 1); //turns to ring 7
task::sleep(600);


nextDrivePIDmove(15, 1); //picks up that ring
task::sleep(900);


nextTurnPIDmove(-156, 1); //spins the goal around to face the corner
task::sleep(150);
spin_intakes('B', 0); //turns it off temporarily
task::sleep(850);
spin_intakes('B', 100); //turns it back on


nextDrivePIDmove(-9, 1); //backs the goal into the scoring zone
task::sleep(650);
clamp_up(); //lets go of the goal
spin_intakes('B', -100);
task::sleep(50);


nextDrivePIDmove(7.5, 1); //moves away from the goal to prepare to lineup the dash into the center
task::sleep(650);
//COMPLETES THE FIRST FULL GOAL
//time at this point is 20000


nextTurnPIDmove(-31.5, 1); //turns to face ring 8
spin_intakes('B', 0); //stop both before starting just the lower one
task::sleep(650);
int intake_position = UpperIntake.position(deg);
int mod_position = intake_position % 1800; //SPINS THE INTAKE OUT OF THE WAY
UpperIntake.spinFor(fwd, 1800 - mod_position, deg, 600, rpm, false);


nextDrivePIDmove(69, 1); //dashes at the ring
spin_intakes('B', 0); //stop both before starting just the lower one
spin_intakes('L', 100); //starts the lower intake
task::sleep(1800);


nextTurnPIDmove(-83, 1); //turns to ring 9
task::sleep(800);


nextDrivePIDmove(38, 1); //goes and picks up the ring 9
task::sleep(500);
spin_intakes('B', 100); //does this entire ring thing to get both rings into the intake
task::sleep(325);
spin_intakes('U', 0);
task::sleep(325);


nextTurnPIDmove(129, 1); //turns the back of the robot to the second mobile goal
task::sleep(900);


nextDrivePIDmove(-19, 0.6); //reverses into it clamping onto the goal
task::sleep(800);
clamp_down();
task::sleep(150);


nextTurnPIDmove(-45, 1); //turns to ring 10
task::sleep(650);
spin_intakes('B', 100); //starts up the intake


nextDrivePIDmove(30, 1); //goes and picks up ring 10
task::sleep(1050);


nextTurnPIDmove(-131, 1); //turns to ring 11 & 12
task::sleep(100);
spin_intakes('U', 0); //turns it off temporarily because it is turning
task::sleep(850);


nextDrivePIDmove(34, 0.5); //drives at 50% speed through both rings to pick them both up
spin_intakes('U', 100); //turns it back on now
task::sleep(1650);
nextDrivePIDmove(-24, 1); //backs up to turn to ring 13
task::sleep(800);
spin_intakes('U', 0); //stops the upper intake temporarily to prevent the launching of a ring
task::sleep(250);


nextTurnPIDmove(55, 1); //turns towared ring 13
task::sleep(300);
spin_intakes('B', 100); //starts up the intakes again
task::sleep(400);


nextDrivePIDmove(15, 1); //picks up ring 13
task::sleep(800);
spin_intakes('U', 0); //stops the top intake


nextTurnPIDmove(143.75, 1); //turns back of the robot to the scoring zone and the front of the robot to past ring 14
task::sleep(800);
spin_intakes('B', 100); //starts the intake up to score ring 13
task::sleep(150);


nextDrivePIDmove(-12.5, 1); //backs the goal into the scoring zone
task::sleep(600);
clamp_up();
spin_intakes('B', -100); //reverses while in the corner to prevent stuck hooks from dragging the goal
task::sleep(100);


nextDrivePIDmove(59.5, 1); //goes to the midline
task::sleep(800);
spin_intakes('B', 100);
task::sleep(600);


nextTurnPIDmove(-106, 1); //turns to the wall stake and ring 14
//desiredLBposition = 120/5; //sets the lady brown into the loading position
//LB_state = 'L';
task::sleep(800);
spin_intakes('U', 0); //JUST NOW ADDED THIS


nextDrivePIDmove(15, 0.8); //goes into the wall stake and uses it to correct
task::sleep(950);
nextDrivePIDmove(28, 1);
task::sleep(900);
spin_intakes('U', 0);
//desiredLBposition = 655/5; //launches the lady brown and scores it
//UpperIntake.spinFor(-15, deg, 100, rpm, true); //reverses the intake a little bit
//LB_state = 'S';
task::sleep(750);
spin_intakes('B', -100); //JUST NOW ADDED THIS
task::sleep(100);
resetDriveSensors = true; //resets the sensors at the ladybrown where it self corrects
desiredValue = 0; //resets the target value as well


nextDrivePIDmove(-15, 1); //backs away to turn to ring 15
task::sleep(850);
desiredLBposition = 0; //lowers lady brown
LB_state = 'R';


nextTurnPIDmove(88, 1); //turns to ring 15
task::sleep(800);
spin_intakes('B', 0);


nextDrivePIDmove(23, 1); //picks up ring 15
spin_intakes('L', 100);
task::sleep(1000);


nextTurnPIDmove(91, 1); //turns to ring 16
spin_intakes('B', 100); //turns on the upper intake te6mporarily
task::sleep(400);
spin_intakes('U', 0); //turns it back off
task::sleep(400);


nextDrivePIDmove(24, 1); //picks up ring 16
task::sleep(1100);


nextTurnPIDmove(128, 1); //turns to the third mobile goal
task::sleep(800);


nextDrivePIDmove(-32.5, 0.6); //backs up and picks up the third mobile goal
task::sleep(1050);
clamp_down(); //clamps down on it
task::sleep(100);


nextTurnPIDmove(-86, 1); //turns to ring 17
task::sleep(800);


nextDrivePIDmove(32, 1); //goes and picks up ring 17 while scoring all 3
spin_intakes('B', 100);
task::sleep(1100);


nextTurnPIDmove(-47, 1); //turns to where the robot will line up to ring 18 and 19
task::sleep(650);


nextDrivePIDmove(23, 1); //goes to lineup spot
task::sleep(1000);


nextTurnPIDmove(-86, 1); //turns to ring 18 and 19
task::sleep(900);


nextDrivePIDmove(27.5, 1); //plows through and intake
task::sleep(1150);
spin_intakes('L', 0);


nextTurnPIDmove(-117, 1); //turns to the scoring zone
task::sleep(900);


nextDrivePIDmove(-18, 1); //pushes the goal back letting it go right from the start
task::sleep(700);
spin_intakes('B', -100); //stops intake
clamp_up(); //releases it
task::sleep(100);


nextDrivePIDmove(50, 1); //drives to line up to the last mobile goal
task::sleep(1250);


nextTurnPIDmove(-125, 1); //turns to final mobile goal
task::sleep(800);


nextDrivePIDmove(-100, 1); //plows it into the scoring zone
task::sleep(2000);


nextDrivePIDmove(25, 0.5); //tries to arc
task::sleep(250);
nextTurnPIDmove(200, 1); //turns to hang //previously 197
task::sleep(1200);


nextDrivePIDmove(-100, 1); //goes to hang
desiredLBposition = 655/5; //launches the lady brown
LB_state = 'S';
intake_position = UpperIntake.position(deg);
mod_position = intake_position % 1800; //SPINS THE INTAKE OUT OF THE WAY
UpperIntake.spinFor(fwd, 1800 - mod_position, deg, 600, rpm, false);


task::sleep(5000);
*/














//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////SOLO WIN POINTS
///////////////////RedSoloWP6
/*
desiredLBposition = 655/5; //ramps the lady brown up to scoring position to score on the alliance stake
LB_state = 'S';
spin_intakes('L', 100);
task::sleep(500);
nextDrivePIDmove(29, 0.8); //drives the ring into the alliance stake while driving backwards to line up with the first mobile goal
task::sleep(500);
desiredLBposition = 0/5; //brings the lady brown back down
LB_state = 'R';
task::sleep(600);


nextTurnPIDmove(-113, 1); //turns to the mobile goal
task::sleep(900);
spin_intakes('L', -100);


nextDrivePIDmove(-34, 0.6); //Goes and picks up the first mobile goal
task::sleep(1150);
clamp_down();
task::sleep(100);


nextTurnPIDmove(98, 1); //turns toward ring 2
task::sleep(800);


nextDrivePIDmove(19, 0.8); //goes and picks up ring 2
spin_intakes('B', 100); //starts the intakes
task::sleep(1100);


nextTurnPIDmove(-146, 1); //turns to ring 3
spin_intakes('U', 0);
task::sleep(950);


nextDrivePIDmove(52, 0.8); //drives to ring 3
lift_intake(); //lifts the intake to get an elevated ring
spin_intakes('B', 100);
task::sleep(700);
clamp_up(); //drops the goal on the way
task::sleep(600);
lower_intake();
spin_intakes('U', 0); //stops the upper intake
task::sleep(100);
nextDrivePIDmove(-4, 0.8); //backs up a bit
task::sleep(650);


nextTurnPIDmove(105, 1); //turns to mobile goal 2
task::sleep(900);


nextDrivePIDmove(-37.5, 0.6); //drives back and clamps goal 2
task::sleep(1250);
clamp_down();
task::sleep(100);


nextTurnPIDmove(-125, 1); //turns to ring 4
task::sleep(900);


nextDrivePIDmove(16, 0.8); //picks up ring 4
desiredLBposition = 655/5;
LB_state = 'S';
spin_intakes('B', 100); //scores ring 3 on the way
task::sleep(1000);


nextTurnPIDmove(-170, 1); //turns around
spin_intakes('U', 0); //stop the upper intake
task::sleep(850);


nextDrivePIDmove(26, 0.5); //drives back to touch the pole
spin_intakes('B', 100); //scores ring 4
*/












//////////////////////////////SOLO WIN POINTS
///////////////////BlueSoloWP7
/*
desiredLBposition = 655/5; //ramps the lady brown up to scoring position to score on the alliance stake
LB_state = 'S';
task::sleep(350);
nextDrivePIDmove(11, 0.8); //drives the ring into the alliance stake
task::sleep(600);
nextDrivePIDmove(-19, 0.8); //drives to lineup with the goal
desiredLBposition = 0/5; //brings the lady brown back down
LB_state = 'R';
task::sleep(950);


nextTurnPIDmove(-67, 1); //turns to the mobile goal
task::sleep(700);


nextDrivePIDmove(-33, 0.6); //Goes and picks up the first mobile goal
task::sleep(1150);
clamp_down();
task::sleep(100);


nextTurnPIDmove(-95, 1); //turns toward ring 2
task::sleep(800);


nextDrivePIDmove(19, 0.8); //goes and picks up ring 2
spin_intakes('B', 100); //starts the intakes
task::sleep(1100);


nextTurnPIDmove(146, 1); //turns to ring 3
spin_intakes('U', 0);
task::sleep(950);


nextDrivePIDmove(52, 0.8); //drives to ring 3
lift_intake(); //lifts the intake to get an elevated ring
spin_intakes('B', 100);
task::sleep(700);
clamp_up(); //drops the goal on the way
task::sleep(600);
lower_intake();
spin_intakes('U', 0); //stops the upper intake
task::sleep(100);
nextDrivePIDmove(-4, 0.8); //backs up a bit
task::sleep(650);


nextTurnPIDmove(-105, 1); //turns to mobile goal 2
task::sleep(900);


nextDrivePIDmove(-37.5, 0.6); //drives back and clamps goal 2
task::sleep(1250);
clamp_down();
task::sleep(100);


nextTurnPIDmove(122, 1); //turns to ring 4
task::sleep(900);


nextDrivePIDmove(16, 0.8); //picks up ring 4
desiredLBposition = 655/5;
LB_state = 'S';
spin_intakes('B', 100); //scores ring 3 on the way
task::sleep(1000);


nextTurnPIDmove(172, 1); //turns around
spin_intakes('U', 0); //stop the upper intake
task::sleep(1000);


nextDrivePIDmove(29.75, 0.8); //drives back to touch the pole
spin_intakes('B', 100); //scores ring 4
*/




//enableDrivePID = false;
in_autonomous = false;
}
/*----------------------------------------------------------------------------*/
/* */
/* User Control Task */
/* */
/* This task is used to control your robot during the user control phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*----------------------------------------------------------------------------*/
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
enableDrivePID = false;
enableLB_PID = true;
//enableLBLoop = true; //whats being used currently


if (!LB_PID_turned_on) //ensures that the LB pid is turned on
{
//task velocity_LB(LB_PID);
//task(LadyBrownLoop, 4); //runs the motor command based lady brown controller instead
task(LB_PID, 4);
LB_PID_turned_on = true;
}
if (in_autonomous) continue;






//tank drive
if (Controller1.Axis3.value() != 0 || Controller1.Axis2.value() != 0)
{
left_drive.spin(vex::directionType::fwd, Controller1.Axis3.value() * 0.12, vex::voltageUnits::volt);
right_drive.spin(vex::directionType::fwd, Controller1.Axis2.value() * 0.12, vex::voltageUnits::volt);
}
else {
left_drive.stop(coast);
right_drive.stop(coast);
}


//joystick drive
/*
left_drive.spin(vex::directionType::fwd, Controller1.Axis3.value() + 1.05 * Controller1.Axis1.value(), vex::velocityUnits::pct);
right_drive.spin(vex::directionType::fwd, Controller1.Axis3.value() - 1.05 * Controller1.Axis1.value(), vex::velocityUnits::pct);
*/




if (Controller1.ButtonR1.pressing()) //press and hold L1 to intake
{
UpperIntake.spin(vex::directionType::fwd, intake_speed, vex::velocityUnits::pct); //full speed while intaking
LowerIntake.spin(vex::directionType::fwd, intake_speed, vex::velocityUnits::pct);
}
else if (Controller1.ButtonR2.pressing()) //press and hold L2 to outtake
{
UpperIntake.spin(vex::directionType::rev, intake_speed * 0.75, vex::velocityUnits::pct); //slower while outtaking for more control
LowerIntake.spin(vex::directionType::rev, intake_speed * 0.75, vex::velocityUnits::pct);
}
else if (Controller1.ButtonB.pressing()) //half intake when necessary
{
LowerIntake.spin(vex::directionType::fwd, intake_speed, vex::velocityUnits::pct);
UpperIntake.stop();
}
else
{
UpperIntake.stop();
LowerIntake.stop();
}




//lady brown triple toggle system
if (LBtoggleCooldown >= 40)
switch (LB_state)
{
case 'R': //in rest position
   if (Controller1.ButtonY.pressing()) //in rest position, pressing X toggles to loading position
   {
     desiredLBposition = 120/5; // old 125 modLB 135 skills 300 &*&*&*&*&*&*&*&*&*
     LB_state = 'L';
     LBtoggleCooldown = 0; //resetting cooldown
   }
   else if (Controller1.ButtonRight.pressing()) //in rest position, pressing Up toggles to scoring position
   {
     desiredLBposition = 655/5;
     UpperIntake.spinFor(-15, deg, 100, rpm, true);
     LB_state = 'S';
     LBtoggleCooldown = 0; //resetting cooldown
   }
   break;
case 'L': //in loading position
   if (Controller1.ButtonY.pressing()) //in loading position, pressing X toggles to rest position
   {
     desiredLBposition = 0;
     LB_state = 'R';
     LBtoggleCooldown = 0; //resetting cooldown
   }
   else if (Controller1.ButtonRight.pressing()) //in loading position, pressing Up toggles to scoring position
   {
     desiredLBposition = 655/5;
     UpperIntake.spinFor(-15, deg, 100, rpm, true);
     LB_state = 'S';
     LBtoggleCooldown = 0; //resetting cooldown
   }
   break;
case 'S': //in scoring position
   if (Controller1.ButtonY.pressing()) //in scoring position, pressing X toggles to loading position
   {
     desiredLBposition = 120/5; //typically 125
     LB_state = 'L';
     LBtoggleCooldown = 0; //resetting cooldown
   }
   else if (Controller1.ButtonRight.pressing()) //in scoring position, pressing Up toggles to rest position
   {
     desiredLBposition = 0;
     LB_state = 'R';
     LBtoggleCooldown = 0; //resetting cooldown
   }
   break;
}




//temporarily created braking mechanism for lady brown PID - not being used anymore
/*
if (LB_state == 'S' && (LBerror < 10 && LBerror > -10))
LBBrake = true;
else
LBBrake = false;
*/




//initial lady brown code for testing
/*
if (Controller1.ButtonX.pressing()) //press and hold X to score ladybrown
{
LadyBrown.spin(vex::directionType::fwd, lb_speed, vex::velocityUnits::pct); //full speed while intaking
}
else if (Controller1.ButtonUp.pressing()) //press and hold Up to retract ladybrown
{
LadyBrown.spin(vex::directionType::rev, lb_speed * 0.75, vex::velocityUnits::pct); //slower while outtaking for more control
}
else
{
LadyBrown.stop();
}
*/




//toggle R1 to clamp or unclamp the locking clamp
if (Controller1.ButtonL1.pressing() && clampCooldown >= 40)
{ //clampCooldown is to ensure that there is time between toggles being registered
if (clampActive == false) //if clamp is inactive
{
 clamp_down();
}
else if (clampActive == true) //if clamp is active
{
 clamp_up();
}


clampCooldown = 0;
}




//toggle A to lift up the intake or lower it
if (Controller1.ButtonA.pressing() && intakeLiftCooldown >= 40)
{ //intakeLiftCooldown is to ensure that there is time between toggles being registered
if (intakeUp == false) //if intake is lowered
{
  lift_intake();
}
else if (intakeUp == true) //if intake is lifted
{
IntakeLift.set(false);
intakeUp = false;
}


intakeLiftCooldown = 0;
}




//toggle Left to lower doinker or lift it back up
if (Controller1.ButtonL2.pressing() && doinkerCooldown >= 40)
{ //doinkerCooldown is to ensure that there is time between toggles being registered
if (doinkerDown == false) //if doinker is in the vertical position
{
Doinker.set(true);
doinkerDown = true;
}
else if (doinkerDown == true) //if doinker is in the lowered position
{
Doinker.set(false);
doinkerDown = false;
}


doinkerCooldown = 0;
}






//FOR SKILLS ONLY
if (Controller1.ButtonUp.pressing() && Controller1.ButtonX.pressing())
{
enableDrivePID = true;
resetDriveSensors = true;
resetTurnSensors = true;
motorBrake = false;
task velocity_drive(drivePID);


desiredLBposition = 655/5; //ramps the lady brown up to scoring position to score on the alliance stake
LB_state = 'S';
task::sleep(450);
nextDrivePIDmove(6, 1); //drives the ring into the alliance stake
task::sleep(500);


nextDrivePIDmove(-29, 0.6); //moves toward the first mobile goal
task::sleep(1000);
desiredLBposition = 400/5; //drops the intake
LB_state = 'S';
clamp_down(); // clamps the goal
task::sleep(100);


desiredLBposition = 0;
LB_state = 'R'; //lowers the lady brown back down
}




//increments toggle cooldowns
clampCooldown++;
LBtoggleCooldown++;
intakeLiftCooldown++;
doinkerCooldown++;
task::sleep(10); // Sleep the task for a short amount of time to
// prevent wasted resources. used to be 20
}
}








//
// Main will set up the competition functions and callbacks.
//
int main() {
// Run the pre-autonomous function.
pre_auton();
// Set up callbacks for autonomous and driver control periods.
Competition.autonomous(autonomous);
Competition.drivercontrol(usercontrol);
// Prevent main from exiting with an infinite loop.
while (1) {
  task::sleep(100); // Sleep the task for a short amount of time to
  // prevent wasted resources.
}
}