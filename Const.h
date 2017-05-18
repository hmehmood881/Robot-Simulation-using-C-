//This header file contains all the constants used in the project.
#ifndef _CONST_H_
#define _CONST_H_

double const l1 = 0.142;
double const l2 = 0.195;
double const l3 = 30;

//Limited velocity and acceleration of each joint
int const JointLimit[] = { -150,-100,-200,-160,150,100,-100,160 };
int const velocityLimit[] = { -150,-150,-50,-150,150,50,150,150 };
int const velLimit[] = { -150,-150,-50,-150 };
int const accLimit[] = { -600,-600,-200,-600,600,600, 200, 600 };


//Sampling time (seconds)
double const TrajSampleTime = 0.02; // * pow(10.0, -3);
double const ControlSampleTime = 0.002; // * pow(10.0, -3);
double const SimSampleTime = 0.0002; // * pow(10.0, -6);

//Interval for simulation and control intervals
int const Sim_interval = 350;
int const Control_interval = 4000000000;
int const Trajectory_planner = 4000000000;


//Dynamic Simulator constants
//Joint masses m1, m2, m3, m4 (unit in kg)
double const m1 = 1.7;
double const m2 = 1.0;
double const m3 = 1.7;
double const m4 = 1.0;
double const g = 9810;// * pow(10.0,3); //Acceleration due to gravity (unit in mm/s^2)
double const Veff[4] = {2*sqrt(m1), 2, 2*sqrt(m3), 2}; //Viscous damping values of the joints
double const stepsize = 0.0004; //Integration stepsize


//Proportional control law: Kp = position gain, Kv = velocity gain
double const Kp[4] = {175.0, 110.0, 40.0, 20.0};
double const Kv[4] = {2*sqrt(Kp[0]), 2*sqrt(Kp[1]), 2*sqrt(Kp[2]), 2*sqrt(Kp[3])};


//D-H constant parameters of robotic arm
double const A1 = 195;
double const A2 = 142;
double const Alpha2 = 180;
double const D1 = 405;
double const D2 = 70;
double const D4 = 410;
double const D5 = 140;

#endif