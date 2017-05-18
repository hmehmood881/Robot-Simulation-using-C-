#pragma once
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

void EULERINTEGRAL(double* next_pos, double* next_vel)
{
	double acc_term;
	int joints;

	for (joints = 0; joints<4; joints++)
	{
		acc_term = (stepsize * control_acc[joints]);
		next_vel[joints] = curr_vel[joints] + acc_term;
		next_pos[joints] = curr_pos[joints] + (curr_vel[joints] * stepsize) + ((stepsize / 2) * acc_term);

		//Saturate the output velocity and displacement to the joint limits should they ever exceed them
		if (fabs(next_vel[joints]) > velLimit[joints])
		{
			if (next_vel[joints] < 0)
			{
				next_vel[joints] = -velLimit[joints];
			}
			else
			{
				next_vel[joints] = velLimit[joints];
			}
		}

		switch (joints)
		{
		case 0:
			if (fabs(next_pos[joints]) > 150)
			{
				if (next_pos[joints] < 0)
				{
					next_pos[joints] = -150;
				}
				else
				{
					next_pos[joints] = 150;
				}
			}
			break;

		case 1:
			if (fabs(next_pos[joints]) > 100)
			{
				if (next_pos[joints] < 0)
				{
					next_pos[joints] = -100;
				}
				else
				{
					next_pos[joints] = 100;
				}
			}
			break;

		case 2:
			if (next_pos[joints] > -100)
			{
				next_pos[joints] = -100;
			}
			else if (next_pos[joints] < -200)
			{
				next_pos[joints] = -200;
			}
			break;

		case 3:
			if (fabs(next_pos[joints]) > 160)
			{
				if (next_pos[joints] < 0)
				{
					next_pos[joints] = -160;
				}
				else
				{
					next_pos[joints] = 160;
				}
			}
			break;
		}
	}
}

void DYNSIM(double ** MassMat)

{
	//Pre-compute terms for removing delay
	double squareTerm[4] = { 0,0,0,0 };
	double g = 9810;
	double c1, c2, c4, s1, s2, s4;

	s1 = sin(DEG2RAD(curr_pos[0]));
	c1 = cos(DEG2RAD(curr_pos[0]));
	s2 = sin(DEG2RAD(curr_pos[1]));
	c2 = sin(DEG2RAD(curr_pos[1]));
	s4 = sin(DEG2RAD(curr_pos[3]));
	c4 = sin(DEG2RAD(curr_pos[3]));

	//Calculate Mass Matrix
	MassMat[0][0] = l2*s2*(l2*m2*s2 + l2*m3*s2 + m4*c4*(l2*c4*s2 - s4*(l2*c2 + l1)) + m4*s4*(l2*s2*s4 + c4*(l2*c2 + l1) + 30)) + 30 * m4*(l2*s2*s4 + c4*(l2*c2 + l1) + 30) + l1*m3*(l2*c2 + l1) + l2*c2*(m3*(l2*c2 + l1) + l2*m2*c2 + m4*c4*(l2*s2*s4 + c4*(l2*c2 + l1) + 30) - m4*s4*(l2*c4*s2 - s4*(l2*c2 + l1))) + l1*m4*c4*(l2*s2*s4 + c4*(l2*c2 + l1) + 30) - l1*m4*s4*(l2*c4*s2 - s4*(l2*c2 + l1));
	MassMat[0][1] = 20164 * m3 + 20164 * m4*pow(s4, 2) + 30 * m4*(l1*c4 + 30) + l2*s2*(m4*s4*(l1*c4 + 30) - l1*m4*c4*s4) + l2*c2*(l1*m3 + l1*m4*pow(s4, 2) + m4*c4*(l1*c4 + 30)) + l1*m4*c4*(l1*c4 + 30);
	MassMat[0][2] = 0;
	MassMat[0][3] = -900 * m4 - 4260 * m4*c4 - 5850 * m4*c2*c4 - 5850 * m4*s2*s4;


	MassMat[1][0] = 30 * m4*(l2*s2*s4 + c4*(l2*c2 + l1) + 30) + l1*m3*(l2*c2 + l1) + l1*m4*c4*(l2*s2*s4 + c4*(l2*c2 + l1) + 30) - l1*m4*s4*(l2*c4*s2 - s4*(l2*c2 + l1));
	MassMat[1][1] = 20164 * m3 + 20164 * m4*pow(s4, 2) + 30 * m4*(l1*c4 + 30) + l1*m4*c4*(l1*c4 + 30);
	MassMat[1][2] = 0;
	MassMat[1][3] = -900 * m4 - 4260 * m4*c4;


	MassMat[2][0] = 0;
	MassMat[2][1] = 0;
	MassMat[2][2] = m3 + m4;
	MassMat[2][3] = 0;

	MassMat[3][0] = -30 * m4*(l2*s2*s4 + c4*(l2*c2 + l1) + 30);
	MassMat[3][1] = -30 * m4*(l1*c4 + 30);
	MassMat[3][2] = 0;
	MassMat[3][3] = 900 * m4;
	//Calculate Velocity Square Term
	squareTerm[0] = pow((curr_vel[0] + curr_vel[1]), 2);
	squareTerm[1] = pow(curr_vel[0], 2);
	squareTerm[2] = pow((curr_vel[0] + curr_vel[1] - curr_vel[3]), 2);
	//Calculate G(0)+V(0.,0)
	GV[0] = l2*c2*(l2*m2*squareTerm[1] * s2 - m4*c4*(s4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) - l2*squareTerm[1] * c4*s2) + l2*m3*squareTerm[1] * s2 + m4*s4*(c4*(l2*squareTerm[1] * c2 + squareTerm[0]) + 30 * squareTerm[2] + l2*squareTerm[1] * s2*s4)) - 30 * m4*(s4*(l2*squareTerm[1] * c2 + l1*squareTerm[2]) - l2*squareTerm[1] * c4*s2) - l2*s2*(m3*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) + m4*s4*(s4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) - l2*squareTerm[1] * c4*s2) + l2*m2*squareTerm[1] * c2 + m4*c4*(c4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) + 30 * squareTerm[2] + l2*squareTerm[1] * s2*s4)) - l1*m4*c4*(s4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) - l2*squareTerm[1] * c4*s2) + 27690 * m3*squareTerm[1] * s2 + l1*m4*s4*(c4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) + 30 * squareTerm[2] + l2*squareTerm[1] * s2*s4);
	GV[1] = 27690 * m3*squareTerm[1] * s2 - l1*m4*c4*(s4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) - l2*squareTerm[1] * c4*s2) - 30 * m4*(s4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) - l2*squareTerm[1] * c4*s2) + l1*m4*s4*(c4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) + 30 * squareTerm[2] + l2*squareTerm[1] * s2*s4);
	GV[2] = -g*m3 - g*m4;
	GV[3] = 30 * m4*(s4*(l2*squareTerm[1] * c2 + l1*squareTerm[0]) - l2*squareTerm[1] * c4*s2);
	//Calculate Torque
	Tau[0] = (MassMat[0][0] * joint_acc[0] + MassMat[0][1] * joint_acc[1] + MassMat[0][2] * joint_acc[2] + MassMat[0][3] * joint_acc[3] + GV[0]);
	Tau[1] = (MassMat[1][0] * joint_acc[0] + MassMat[1][1] * joint_acc[1] + MassMat[1][2] * joint_acc[2] + MassMat[1][3] * joint_acc[3] + GV[1]);
	Tau[2] = (MassMat[2][0] * joint_acc[0] + MassMat[2][1] * joint_acc[1] + MassMat[2][2] * joint_acc[2] + MassMat[2][3] * joint_acc[3] + GV[2]);
	Tau[3] = (MassMat[3][0] * joint_acc[0] + MassMat[3][1] * joint_acc[1] + MassMat[3][2] * joint_acc[2] + MassMat[3][3] * joint_acc[3] + GV[3]);

	//PLANT(ref_pos, ref_vel, Tau, MassMat,GV, output_pos, output_vel, joint_acc);

}


void PLANT(double** MassMat)
{
	double** Mas_inv = 0;
	Mas_inv = new double*[4];

	for (int h = 0; h < 4; h++)
	{
		Mas_inv[h] = new double[4];

		for (int w = 0; w < 4; w++)
		{
			Mas_inv[h][w] = 0;

		}
	}

	//Update the joint acceleration terms with the new applied torques and other parameters
	MatrixInversion(MassMat, 4, Mas_inv);
	//Joint 1 acceleration calculation
	joint_acc[0] = Mas_inv[0][0] * (Tau[0] - GV[0]) + Mas_inv[0][1] * (Tau[1] - GV[1]) + Mas_inv[0][2] * (Tau[2] - GV[2]) + Mas_inv[0][3] * (Tau[3] - GV[3]);


	//Joint 2 acceleration calculation
	joint_acc[1] = Mas_inv[1][0] * (Tau[0] - GV[0]) + Mas_inv[1][1] * (Tau[1] - GV[1]) + Mas_inv[1][2] * (Tau[2] - GV[2]) + Mas_inv[1][3] * (Tau[3] - GV[3]);

	//Joint 3 acceleration calculation
	joint_acc[2] = Mas_inv[2][0] * (Tau[0] - GV[0]) + Mas_inv[2][1] * (Tau[1] - GV[1]) + Mas_inv[2][2] * (Tau[2] - GV[2]) + Mas_inv[2][3] * (Tau[3] - GV[3]);


	//Joint 4 acceleration calculation	
	joint_acc[3] = Mas_inv[3][0] * (Tau[0] - GV[0]) + Mas_inv[3][1] * (Tau[1] - GV[1]) + Mas_inv[3][2] * (Tau[2] - GV[2]) + Mas_inv[3][3] * (Tau[3] - GV[3]);

	//Now to integrate the joint accelerations to find the joint output displacements and velocities
}

void CONTROLLAW()
{
	//For loop index to loop for all four joints
	int joints;

	//Calculate control signals of the four joints' displacements and velocities
	for (joints = 0; joints<4; joints++)
	{
		control_pos[joints] = Kp[joints] * (ref_pos[joints] - output_pos[joints]);
		control_vel[joints] = Kv[joints] * (ref_vel[joints] - output_vel[joints]);
		control_acc[joints] = control_pos[joints] + control_vel[joints] + ref_acc[joints];
	}
}
#endif
