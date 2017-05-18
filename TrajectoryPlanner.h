#pragma once
#ifndef _TRAJECTORYPLANNER_H_
#define _TRAJECTORYPLANNER_H_

using namespace std;

bool VelAccLimits(double vel, double acc, int joint)
{
	bool voilation = false;

	if (vel < velocityLimit[joint - 1] || (vel > velocityLimit[joint + 3]))
	{
		voilation = true;
		cout << "Velocity " << joint << " limit violated" << endl;
	}

	if (vel < accLimit[joint - 1] || (vel > accLimit[joint + 3]))
	{
		voilation = true;
		cout << "Acceleration " << joint << " limit violated" << endl;
	}

	return voilation;
}

void TrajectoryPlanner(double** pose, double **theta, double t, double h, double ** v, double** a_coeff, double** b_coeff, double** c_coeff, double** d_coeff)
{
	JOINT q3 = { 0,0,0,0 };
	double Delta[4] = { 0,0,0,0 };
	double Delta2[4] = { 0,0,0,0 };
	double Delta3[4] = { 0,0,0,0 };
	double Delta4[4] = { 0,0,0,0 };
	double near_sol[4] = { 0,0,0,0 };
	double far_sol[4] = { 0,0,0,0 };
	bool sol = false;

	double Y[4] = { 0,0,0,0 };
	double Y2[4] = { 0,0,0,0 };
	double Y3[4] = { 0,0,0,0 };
	double Y4[4] = { 0,0,0,0 };
	GetConfiguration(q3);
	INVKIN(pose[0], q3, near_sol, far_sol, sol);
	CheckJointlimits(theta[0], near_sol, far_sol);
	q3[0] = theta[0][0];
	q3[1] = theta[0][1];
	q3[2] = theta[0][2];
	q3[3] = theta[0][3];
	MoveToConfiguration(q3);
	cout << "Please input the total duration" << endl;
	cin >> t;
	h = double(t / 4);
		INVKIN(pose[1], q3, near_sol, far_sol, sol);
		CheckJointlimits(theta[1], near_sol, far_sol);
		q3[0] = theta[1][0];
		q3[1] = theta[1][1];
		q3[2] = theta[1][2];
		q3[3] = theta[1][3];
		INVKIN(pose[2], q3, near_sol, far_sol, sol);
		CheckJointlimits(theta[2], near_sol, far_sol);
		q3[0] = theta[2][0];
		q3[1] = theta[2][1];
		q3[2] = theta[2][2];
		q3[3] = theta[2][3];
		INVKIN(pose[3], q3, near_sol, far_sol, sol);
		CheckJointlimits(theta[3], near_sol, far_sol);
		q3[0] = theta[3][0];
		q3[1] = theta[3][1];
		q3[2] = theta[3][2];
		q3[3] = theta[3][3];
		INVKIN(pose[4], q3, near_sol, far_sol, sol);
		CheckJointlimits(theta[4], near_sol, far_sol);
		A[0][0] = 8 / h;
		A[0][1] = 2 / h;
		A[0][2] = 0;
		A[1][0] = 2 / h;
		A[1][1] = 8 / h;
		A[1][2] = 2 / h;

		A[2][0] = 0;
		A[2][1] = 2 / h;
		A[2][2] = 8 / h;
		Delta[0] = (theta[1][0] - theta[0][0]);
		Delta[1] = (theta[2][0] - theta[1][0]);
		Delta[2] = (theta[3][0] - theta[2][0]);
		Delta[3] = (theta[4][0] - theta[3][0]);
		Y[0] = 6 * ((Delta[0] + Delta[1]) / (h*h));
		Y[1] = 6 * ((Delta[1] + Delta[2]) / (h*h));
		Y[2] = 6 * ((Delta[2] + Delta[3]) / (h*h));
		////////Joint 2////////////////////////////
		Delta2[0] = (theta[1][1] - theta[0][1]);
		Delta2[1] = (theta[2][1] - theta[1][1]);
		Delta2[2] = (theta[3][1] - theta[2][1]);
		Delta2[3] = (theta[4][1] - theta[3][1]);
		Y2[0] = 6 * ((Delta2[0] + Delta2[1]) / (h*h));
		Y2[1] = 6 * ((Delta2[1] + Delta2[2]) / (h*h));
		Y2[2] = 6 * ((Delta2[2] + Delta2[3]) / (h*h));
		///////////////Joint 3/////////////////////////
		Delta3[0] = (theta[1][2] - theta[0][2]);
		Delta3[1] = (theta[2][2] - theta[1][2]);
		Delta3[2] = (theta[3][2] - theta[2][2]);
		Delta3[3] = (theta[4][2] - theta[3][2]);
		Y3[0] = 6 * ((Delta3[0] + Delta3[1]) / (h*h));
		Y3[1] = 6 * ((Delta3[1] + Delta3[2]) / (h*h));
		Y3[2] = 6 * ((Delta3[2] + Delta3[3]) / (h*h));
		/////////////////Joint 4/////////////////////
		Delta4[0] = (theta[1][3] - theta[0][3]);
		Delta4[1] = (theta[2][3] - theta[1][3]);
		Delta4[2] = (theta[3][3] - theta[2][3]);
		Delta4[3] = (theta[4][3] - theta[3][3]);
		Y4[0] = 6 * ((Delta4[0] + Delta4[1]) / (h*h));
		Y4[1] = 6 * ((Delta4[1] + Delta4[2]) / (h*h));
		Y4[2] = 6 * ((Delta4[2] + Delta4[3]) / (h*h));

		MatrixInversion(A, 3, inv);

		for (int i = 0; i < 3; ++i)
		{
			for (int k = 0; k < 3; ++k)
			{
				v[0][i + 1] = v[0][i + 1] + (inv[i][k] * Y[k]);
				v[1][i + 1] = v[1][i + 1] + (inv[i][k] * Y2[k]);
				v[2][i + 1] = v[2][i + 1] + (inv[i][k] * Y3[k]);
				v[3][i + 1] = v[3][i + 1] + (inv[i][k] * Y4[k]);

			}

		}

		for (int i = 0; i < 4; ++i)
		{
			a_coeff[0][i] = theta[i][0];
			b_coeff[0][i] = v[0][i] * h;
			c_coeff[0][i] = 3 * (theta[i + 1][0] - theta[i][0]) - 2 * v[0][i] * h - v[0][i + 1] * h;
			d_coeff[0][i] = -2 * (theta[i + 1][0] - theta[i][0]) + v[0][i] * h + v[0][i + 1] * h;
			///////////////////////
			a_coeff[1][i] = theta[i][1];
			b_coeff[1][i] = v[1][i] * h;
			c_coeff[1][i] = 3 * (theta[i + 1][1] - theta[i][1]) - 2 * v[1][i] * h - v[1][i + 1] * h;
			d_coeff[1][i] = -2 * (theta[i + 1][1] - theta[i][1]) + v[1][i] * h + v[1][i + 1] * h;
			/////////////////////////////
			a_coeff[2][i] = theta[i][2];
			b_coeff[2][i] = v[2][i] * h;
			c_coeff[2][i] = 3 * (theta[i + 1][2] - theta[i][2]) - 2 * v[2][i] * h - v[2][i + 1] * h;
			d_coeff[2][i] = -2 * (theta[i + 1][2] - theta[i][2]) + v[2][i] * h + v[2][i + 1] * h;
			///////////////////////////
			a_coeff[3][i] = theta[i][3];
			b_coeff[3][i] = v[3][i] * h;
			c_coeff[3][i] = 3 * (theta[i + 1][3] - theta[i][3]) - 2 * v[3][i] * h - v[3][i + 1] * h;
			d_coeff[3][i] = -2 * (theta[i + 1][3] - theta[i][3]) + v[3][i] * h + v[3][i + 1] * h;

		}



	}
	void TrajectoryExecution(double h, int segment, double** a_coeff, double** b_coeff, double** c_coeff, double** d_coeff)
	{
		double time = 0;
		for (int j = 0; j < 4; j++)
		{
			for (int ti = 0; ti < (h * 51.3); ti++)
			{
				time = time + 0.02;
				ref_pos[0] = a_coeff[0][j] + b_coeff[0][j] * time / h + c_coeff[0][j] * time*time / (h*h) + d_coeff[0][j] * time*time*time / (h*h*h);
				ref_vel[0] = (b_coeff[0][j] + 2 * c_coeff[0][j] * time / h + 3 * d_coeff[0][j] * time*time / (h*h)) / h;
				ref_acc[0] = (2 * c_coeff[0][j] + 6 * d_coeff[0][j] * time / h) / (h*h);
				ref_pos[1] = a_coeff[1][j] + b_coeff[1][j] * time / h + c_coeff[1][j] * time*time / (h*h) + d_coeff[1][j] * time*time*time / (h*h*h);
				ref_vel[1] = (b_coeff[1][j] + 2 * c_coeff[1][j] * time / h + 3 * d_coeff[1][j] * time*time / (h*h)) / h;
				ref_acc[1] = (2 * c_coeff[1][j] + 6 * d_coeff[1][j] * time / h) / (h*h);
				ref_pos[2] = a_coeff[2][j] + b_coeff[2][j] * time / h + c_coeff[2][j] * time*time / (h*h) + d_coeff[2][j] * time*time*time / (h*h*h);
				ref_vel[2] = (b_coeff[2][j] + 2 * c_coeff[2][j] * time / h + 3 * d_coeff[2][j] * time*time / (h*h)) / h;
				ref_acc[2] = (2 * c_coeff[2][j] + 6 * d_coeff[2][j] * time / h) / (h*h);
				ref_pos[3] = a_coeff[3][j] + b_coeff[3][j] * time / h + c_coeff[3][j] * time*time / (h*h) + d_coeff[3][j] * time*time*time / (h*h*h);
				ref_vel[3] = (b_coeff[3][j] + 2 * c_coeff[3][j] * time / h + 3 * d_coeff[3][j] * time*time / (h*h)) / h;
				ref_acc[3] = (2 * c_coeff[3][j] + 6 * d_coeff[3][j] * time / h) / (h*h);
				VelAccLimits(ref_vel[0], ref_acc[0], 1);
				VelAccLimits(ref_vel[1], ref_acc[1], 2);
				VelAccLimits(ref_vel[2], ref_acc[2], 3);
				VelAccLimits(ref_vel[3], ref_acc[3], 4);

			}
			time = 0;
		}

	}
#endif
