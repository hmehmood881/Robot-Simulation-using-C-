#pragma once
#ifndef _GLOBALVAR_H_
#define _GLOBALVA_H_



double determinant;
double squareTerm[3];

LARGE_INTEGER StartingTime_T3, EndingTime_T3, ElapsedMicroseconds_T3;
LARGE_INTEGER Frequency_T3;
LARGE_INTEGER StartingTime_T2, EndingTime_T2, ElapsedMicroseconds_T2;
LARGE_INTEGER Frequency_T2;
LARGE_INTEGER StartingTime_T1, EndingTime_T1, ElapsedMicroseconds_T1;
LARGE_INTEGER Frequency_T1;


//Array to store the joint applied torques/forces obtained from trajectory planner
double tau[4];

//Temporary variables to store sine and cosine terms involving the desired joint displacements
double c2; //cos( theta_2 )
double c4; //cos( theta_4 )
double cA; //cos( theta_2 - theta_4 )
double cB; //cos( theta_2 - (2 * theta_4) )
double c2A; //cos( 2*(theta_2 - theta_4) )
double s2; //sin( theta_2 )
double s4; //sin( theta_4 )
double sA; //sin( theta_2 - theta_4 )

//Euler integration initial value variables
double curr_pos[4] = { 0 };
double curr_vel[4] = { 0 };
double curr_acc[4] = { 0 };
double output_pos[4] = { 0 };
double output_vel[4] = { 0 };
double ref_acc[4] = { 0 };
double ref_pos[4] = { 0 };
double ref_vel[4] = { 0 };
double control_pos[4];
double control_vel[4];
double control_acc[4];
double joint_acc[4];
double Tau[4] = { 0,0,0,0 };
double GV[4] = { 0 };
double** inv ;
double** a ;
double** A ;
double** c ;


#endif