/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.

	Xudong: 2020/11/10 Reimplement this thread-safe timer class in Linux
*/

#include <Utils/Timer.h>
#include <sys/time.h>
/**
	This constructor initializes a timer.
*/
Timer::Timer()
{
    //first, get an idea of the frequency...
    // DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread(), (DWORD_PTR)1);
    // QueryPerformanceFrequency((LARGE_INTEGER *)&this->frequency);
    // SetThreadAffinityMask(GetCurrentThread(), oldmask);
    restart();
}

/**
	Default timer destructor - doesn't do much.
*/

Timer::~Timer() {}

/**
	This method resets the starting time.
*/

void Timer::restart()
{
    // DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread(), (DWORD_PTR)1);
    // QueryPerformanceCounter((LARGE_INTEGER *)&this->startTime);
    // SetThreadAffinityMask(GetCurrentThread(), oldmask);
    struct timeval tv;
    gettimeofday(&tv, NULL); // microseconds, 1e6 s
    startTime = (long long)(tv.tv_sec) * 1e6 + (long long)tv.tv_usec;
}

/**
	This method returns the number of milliseconds that has ellapsed since the timer was restarted.
*/
// #include <iostream>
double Timer::timeEllapsed()
{
    // long long int tempTime;
    // //force the thread to run on CPU 0 because the QPC method is buggy
    // DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread(), (DWORD_PTR)1);
    // QueryPerformanceCounter((LARGE_INTEGER *)&tempTime);
    // //let it run wild and free again
    // SetThreadAffinityMask(GetCurrentThread(), oldmask);
    // if (tempTime < startTime)
    //     return 0;
    struct timeval tv;
    gettimeofday(&tv, NULL); // microseconds, 1e6 s
    long long tempTime = (long long)(tv.tv_sec) * 1e6 + (long long)tv.tv_usec;
    // std::cout << "start time = " << startTime << " cur time = " << tempTime
    //           << std::endl;
    return (tempTime - startTime) / 1e6; //return seconds
}
