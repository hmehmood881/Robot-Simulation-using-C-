#pragma once
//This header file contains all the constants used in the project.
#ifndef _TIMER_DELAY_
#define _TIMER_DELAY__

bool Delay_Timer(double Delay)
{
	LARGE_INTEGER StartingTime, EndingTime, ElapsedMicroseconds;
	LARGE_INTEGER Frequency;
	ElapsedMicroseconds.QuadPart = 0;
	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);
	ElapsedMicroseconds.QuadPart = 0;
	QueryPerformanceFrequency(&Frequency);
	QueryPerformanceCounter(&StartingTime);

	while (ElapsedMicroseconds.QuadPart < Delay)
	{
		QueryPerformanceCounter(&EndingTime);
		ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
		ElapsedMicroseconds.QuadPart *= 1000000;
		ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;


		if (ElapsedMicroseconds.QuadPart >= Delay)
		{
			return true;
		}
	}
	return false;
}

#endif
