#ifndef ROUTINES_H
#define ROUTINES_H

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
#include "tricks.h"
#include "PathPlanning.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>


extern condition_variable cond;
extern mutex mu,grabmu;
extern Serial *arduino;
extern cam CAM;
extern IK ik;
extern Tricks tricks;
extern double angles[7];
extern double t[3][3];
extern PathPlanning pp;

class Routines
{
    public:
        Routines();
        void stacking(double speed, int flip);
        void stackingOO(double speed, int flip);
        void showOff(double speed);
        void monkeySeeMonkeyDo();
        int returnBlock(double x, double y, double z, double temptheta, double speed, int flip, struct Pos& drop,int counter);
        int returnBlockOO(double x, double y, double z, double temptheta, double speed, int flip, struct Pos& drop,int counter, vector<vector<vector<double > > >& objectPoints, vector<int >& foundMarkers);

    private:


};

#endif // ROUTINES_H
