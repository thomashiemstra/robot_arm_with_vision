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
		void setArmPosNN(struct Pos Pos, int flip, double anglesInternal[6]);
		void line(struct Pos start, struct Pos stop, double speed, int flip);
		void lineNN(struct Pos start, struct Pos stop, double speed, int flip, double anglesInternal[6]);
		void pointToPoint(struct Pos start, struct Pos stop, double time, int flip);
		void anglesToAngles(double startAngles[7], double stopAngles[7], double time, int flip, int grip);
		void setPos(struct Pos* pos, double x, double y, double z, double alpha, double beta, double gamma,int grip);
		int wait();
        void sendStuff(int16_t *val);
        void commandArduino(double anglez[7], int grip);
        void msleep(long ms);
        double fixtheta(double theta);
	private:

};

#endif
