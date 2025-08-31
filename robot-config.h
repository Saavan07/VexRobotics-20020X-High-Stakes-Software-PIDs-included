#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "v5.h"
#include "v5_vcs.h"
//
using namespace vex;
vex::brain Brain;


//left drive train
vex::motor LeftFMotor (vex::PORT2, vex::gearSetting::ratio18_1,true);
vex::motor LeftUMotor (vex::PORT1, vex::gearSetting::ratio18_1,false);
vex::motor LeftBMotor (vex::PORT10, vex::gearSetting::ratio18_1,true);


//right drive train
vex::motor RightFMotor (vex::PORT3, vex::gearSetting::ratio18_1,false);
vex::motor RightUMotor (vex::PORT4, vex::gearSetting::ratio18_1,true);
vex::motor RightBMotor (vex::PORT9, vex::gearSetting::ratio18_1,false);


vex::motor LadyBrown (vex::PORT13, vex::gearSetting::ratio18_1,false);
vex::motor UpperIntake (vex::PORT19, vex::gearSetting::ratio18_1,false);
vex::motor LowerIntake (vex::PORT11, vex::gearSetting::ratio18_1,false);


//sensors
inertial Inertial(PORT5);
rotation LBsensor(vex::PORT8, false); //rotational sensor for lady brown
rotation Parallel_Odom(vex::PORT6, true); //sensor for the parallel odom wheel


vex::controller Controller1;
vex::competition Competition;
#define RED_ALLIANCE 'R'
#define BLUE_ALLIANCE 'B'