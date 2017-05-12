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
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324
#define ramp 2.0 /* distance over which to accelerate. a is a result, not an input*/
#define precision 5.0 /* steps per centimeter (it gets adjusted a little)*/

using namespace cv;
using namespace std;
/* I feel safer globally defining these*/
mutex mu,grabmu;
condition_variable cond;

//IK ik1 = IK();
//cam CAM1 = cam(0,30); /* 30 is as high as she'll go*/
char const * portName = "\\\\.\\COM3";
Tricks tricks = Tricks(portName);

//double w[3][3]= {{0,1,0},    //the target rotation matrix R
//                 {0,0,1},
//                 {1,0,0}};

struct Pos{
    double x; double y; double z;
    double alpha; double beta; double gamma; /* euler angles for the target orientation */
    int grip;
};

int main(void){
    double speed = 25;
    //int flip = 0;

    tricks.showOff(speed);
    //monkeySeeMonkeyDo();
    //ashestacking(speed,flip);

    return 1;
}
