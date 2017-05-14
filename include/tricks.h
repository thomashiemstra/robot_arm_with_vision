/*
	trick and routines for the arm to perform
*/

#ifndef TRICKS_H
#define TRICKS_H

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <windows.h>
#include <unistd.h>
#include <stdint.h>
#include <conio.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "IK.h"
#include "Serial.h"
#include "cam.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

extern condition_variable cond;
extern mutex mu,grabmu;
extern Serial *arduino;
extern cam CAM;
extern IK ik;

using namespace std;

class Tricks
{
	public:
		Tricks();
		void setArmPos(struct Pos Pos, int flip);
		void line(struct Pos start, struct Pos stop, double speed, int flip);
		void pointToPoint(struct Pos start, struct Pos stop, double time, int flip);
		void setPos(struct Pos* pos, double x, double y, double z, double alpha, double beta, double gamma,int grip);
		void showOff(double speed);
		int returnBlock(double x, double y, double z, double temptheta, double speed, int flip, struct Pos drop,int counter);
		void monkeySeeMonkeyDo();
		void stacking(double speed, int flip);
	private:
        void sendStuff(int16_t *val);
        void commandArduino(double angles[7], int grip);
        void msleep(long ms);
        int wait();
        double fixtheta(double x,double theta);
};

#endif
