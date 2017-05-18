#pragma once
#pragma once
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <Windows.h>
#include <conio.h>
#include <iostream>
#include <fstream>

bool VoilateLimits(double *p, bool RW, int sol)
{
	bool voilation = false;
	for (int w = 0; w < 4; w = w + 1)
	{
		if (p[w] < JointLimit[w] || p[w] > JointLimit[w + 4])
		{
			if (RW)
			{
				sol = 1;
			}
			else
			{
				sol = 2;
			}
			voilation = true;
			cout << "Joint " << w + 1 << " limit violated" << " for solution" << sol << endl;
		}
	}
	return voilation;
}

double CheckAngle(double angle)
{
	double result = 0;
	if (angle>360)
	{
		return result = angle - (floor(angle / 360) * 360);

	}
	else
	{
		return result = angle;

	}


}


bool CheckJointlimits(double *theta, double* near_sol, double *far_sol)
{
	int sol1 = 0;
	bool RW = false;
	bool voilation = false;
	voilation = VoilateLimits(near_sol, RW, sol1);
	if (voilation)
	{
		RW = 0;
		voilation = false;
		voilation = VoilateLimits(far_sol, RW, sol1);
		if (voilation)
		{
			//cout << "No solution exists in the workspace" << endl;
			return true;
		}
		else
		{
			//cout << "farther solution exists in the workspace" << endl;
			theta[0] = far_sol[0];
			theta[1] = far_sol[1];
			theta[2] = far_sol[2];
			theta[3] = far_sol[3];
			return false;
		}
	}

	else
	{
		//cout << "near Solution exists in the workspace" << endl;
		theta[0] = near_sol[0];
		theta[1] = near_sol[1];
		theta[2] = near_sol[2];
		theta[3] = near_sol[3];
		return false;

	}
}

void KIN(double *theta, double *pw)
{
	pw[0] = l1 * 1000* cos(DEG2RAD((theta[0] + theta[1]))) + l2 * 1000 * cos(DEG2RAD((theta[0])));
	pw[1] = l1 * 1000 * sin(DEG2RAD((theta[0] + theta[1]))) + l2 * 1000 * sin(DEG2RAD((theta[0])));
	pw[2] = -theta[2] - 75;
	pw[3] = theta[0] + theta[1] - theta[3];

	//cout << " px " << pw[0] << " py " << pw[1] << " pz " << pw[2] << " phi " << pw[3] << endl;
}



void INVKIN(double *pose, double *current, double *near_sol, double *far_sol, bool sol)
{
	double summation = 0;
	double x, y, z, d3 = 0;
	double theta1[2], theta2[2], theta4[2];
	double weight[3] = { 1,1,1 };
	x = pose[0];
	y = pose[1];
	z = pose[2];;

	summation = (x*x + y*y - 58189) / (55380);
	theta2[0] = acos(summation);
	theta2[1] = -(theta2[0]);
	theta1[0] = atan2(((195 + 142 * cos(theta2[0]))*y - (142 * sin(theta2[0]))*x), ((195 + 142 * cos(theta2[0]))*x + (142 * sin(theta2[0]))*y));
	theta1[1] = atan2(((195 + 142 * cos(theta2[1]))*y - (142 * sin(theta2[1]))*x), ((195 + 142 * cos(theta2[1]))*x + (142 * sin(theta2[1]))*y));
	d3 = -z - 75;
	theta4[0] = CheckAngle(RAD2DEG((theta1[0])) + RAD2DEG((theta2[0])) - pose[3]);
	theta4[1] = CheckAngle(RAD2DEG((theta1[1])) + RAD2DEG((theta2[1]))) - pose[3];

	//cout << "The first solution is theta1: " << RAD2DEG(theta1[0]) << " theta2 : " << RAD2DEG(theta2[0]) << " d3 : " << d3 << " theta4 : " << theta4[0] << endl;
	//cout << "The second solution is theta1: " << RAD2DEG(theta1[1]) << " theta2 : " << RAD2DEG(theta2[1]) << " d3 : " << d3 << " theta4 : " << theta4[1] << endl;

	if ((abs(weight[0] * RAD2DEG(theta1[0]) - current[0] + weight[1] * RAD2DEG(theta2[0]) - current[1] + weight[2] * theta4[0] - current[3]))< abs(weight[0] * RAD2DEG(theta1[1]) - current[0] + weight[1] * RAD2DEG(theta2[1]) - current[1] + weight[2] * theta4[1] - current[3]))
	{
		near_sol[0] = RAD2DEG(theta1[0]);
		near_sol[1] = RAD2DEG(theta2[0]);
		near_sol[2] = d3;
		near_sol[3] = theta4[0];
		far_sol[0] = RAD2DEG(theta1[1]);
		far_sol[1] = RAD2DEG(theta2[1]);
		far_sol[2] = d3;
		far_sol[3] = theta4[1];
	}
	else
	{
		near_sol[0] = RAD2DEG(theta1[1]);
		near_sol[1] = RAD2DEG(theta2[1]);
		near_sol[2] = d3;
		near_sol[3] = theta4[1];
		far_sol[0] = RAD2DEG(theta1[0]);
		far_sol[1] = RAD2DEG(theta2[0]);
		far_sol[2] = d3;
		far_sol[3] = theta4[0];
	}



}
#endif
