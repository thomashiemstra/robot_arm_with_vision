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
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324
#define ramp 2.0 /* distance over which to accelerate. a is a result, not an input*/
#define precision 5.0 /* steps per centimeter (it gets adjusted a little)*/

using namespace cv;
using namespace std;

void msleep(long ms);

Serial *arduino;
IK ik = IK();
cam CAM = cam(0,30);

mutex mu,grabmu;
condition_variable cond;

const float arucoSquareDimension = 0.0265f; //in meters
double angles[7] = {0};
char outPut[10] = {0};
char const * portName = "\\\\.\\COM3";
double t[3][3]= {{0,1,0},    //the target rotation matrix R
                 {0,0,1},
                 {1,0,0}};

struct Pos{
    double x; double y; double z;
    double alpha; double beta; double gamma; /* euler angles for the target orientation */
    int grip;
};

void sendStuff(int16_t *val){ //sending 7 2 byte ints over serial
	uint8_t bytes[16];
    for(int i =0; i < 8; i++){
        bytes[2*i] = (val[i] >> 8) &0xff;
        bytes[2*i+1] = val[i] & 0xff;
    }
	arduino->WriteData(bytes,16);
}

/* gripper position is a percentage, 100% is open*/
void commandArduino(double angles[7], int grip){
    int16_t ticks[8];
    ticks[0] = ik.getServoTick(angles[1],0);
    ticks[1] = ik.getServoTick(pi - angles[2],1);
    ticks[2] = ik.getServoTick(angles[2],2);
    ticks[3] = ik.getServoTick(angles[3],3);
    ticks[4] = ik.getServoTick((pi - angles[4]),4);
    ticks[5] = ik.getServoTick(angles[5],5);
    ticks[6] = ik.getServoTick((pi - angles[6]),6);
    ticks[7] = 700 - 3.5*grip;
    sendStuff(ticks);
}

void commandArduinoTest(vector<double>& angles, int grip){
    int16_t ticks[8];
    ticks[0] = ik.getServoTick(angles[1],0);
    ticks[1] = ik.getServoTick(angles[2],1);
    ticks[2] = ik.getServoTick(pi - angles[2],2);
    ticks[3] = ik.getServoTick(angles[3],3);
    ticks[4] = ik.getServoTick((pi - angles[4]),4);
    ticks[5] = ik.getServoTick(angles[5],5);
    ticks[6] = ik.getServoTick((pi - angles[6]),6);
    ticks[7] = 700 - 3.5*grip;
    sendStuff(ticks);
}

void msleep(long ms){  /* delay function in miliseconds*/
    long us;
    us = 1000*ms;
    struct timespec wait;
    wait.tv_sec = us / (1000 * 1000);
    wait.tv_nsec = (us % (1000 * 1000)) * 1000;
    nanosleep(&wait, NULL);
}

/* this function is a mess and also it's cheating, please ignore */
void line(struct Pos start, struct Pos stop, double speed, int flip){
    double j;
    double dx,dy,dz,dr,r,x,y,z;
    double wait,current_r;
    double dalpha,dbeta,dgamma;
    double alpha,beta,gamma;
    double temp;
    double ramp_distance;
    double dv;
    vector<vector<double >> anglesArray;
    double r_a; /* sum of the delta angles*/
    int dgrip;
    int steps;
    int ramp_steps;

    double v_max = speed/1000.0; /* the delay function is in milliseconds so we convert to cm per millisecond*/

    dx = stop.x - start.x; dy = stop.y - start.y; dz = stop.z - start.z;
    r = sqrt(dx*dx+dy*dy+dz*dz); /* total path length in centimeters */
    dalpha = stop.alpha - start.alpha; dbeta  = stop.beta - start.beta; dgamma = stop.gamma - start.gamma;
    r_a = (dalpha + dbeta + dgamma)/(2*pi); /* a verry rough estimate for the path traveled by the wrist*/
    dgrip = stop.grip - start.grip;
    steps = floor(r*precision); /* steps has to be a whole number resulting in dr >= 1/precision*/
    dr = r/steps;
    ramp_steps = ramp*precision;
    ramp_distance = ramp_steps*dr; /* ramp_distance >= ramp now */
    double min_delay = dr/v_max;

    if(r == 0){


    }


    /* notice j=1, we should already be at start because of the previous step, otherwise... chaos anyway*/
    for(j=1; j<=steps; j++){
        current_r = dr*j;
        x = start.x + ((j/steps)*dx);
        y = start.y + ((j/steps)*dy);
        z = start.z + ((j/steps)*dz);
        alpha = start.alpha + ((j/steps)*dalpha);
        beta = start.beta + ((j/steps)*dbeta);
        gamma = start.gamma + ((j/steps)*dgamma);
        ik.eulerMatrix(alpha,beta,gamma,t);
        ik.inverseKinematics(x,y,z,t,angles,flip);

        if(r<2*ramp){
            msleep(2*min_delay); /* path too short, half max speed without acceleration*/
        }

        if(current_r <= ramp + 0.01 ){ /* 0.01 in case dr gets rounded down a bit somehow*/
            dv = j*(v_max/ramp_steps);
            wait = dr/dv;    /* dt = dr/dv, dv=j*(speed/ramp_steps)*/
            msleep(wait);
        }
        else if(current_r > ramp && current_r <= r - ramp + 0.01){
            msleep(min_delay);
        }
        else if(current_r > r - ramp + 0.01){
            dv = v_max - ((j - (steps-ramp_steps)-1) *(v_max/ramp_steps));
            wait = dr/dv;
            msleep(wait);
        }
        commandArduino(angles,start.grip);
    }
}

void setPos(struct Pos* pos, double x, double y, double z, double alpha, double beta, double gamma,int grip){
    pos->x=x; pos->y=y; pos->z=z;
    pos->alpha=alpha; pos->beta=beta; pos->gamma=gamma;
    pos->grip =  grip;
}

int wait(){
    cout << "press any key to continue or esc to quit" << endl;
    if(getch() == 27)
        return 0;
    else
        return 1;
}

double fixtheta(double x,double theta){
        if(x >= 0){
        if(theta > 0)
            theta = -fmod( -(theta - pi),pi/2.0);
        if(theta < -pi/2.0)
            theta = -fmod(-theta,pi/2.0);
        if(theta < -pi/4.0 && x <= 10)
            theta += pi/2.0;
        }
        if(x < 0){
            if(theta < 0)
                theta = fmod(theta + pi,pi/2.0);
            if(theta > pi/2.0)
                theta = fmod(theta,pi/2.0);
            if(theta > pi/4.0 && x >= -10)
                theta -= pi/2.0;
        }
        return theta;
}
/* picks up the block found by "findVecsCharuco" and puts it at dumps location*/
int returnBlock(double x, double y, double z, double temptheta, double speed, int flip, struct Pos dump){
    unique_lock<mutex> locker(grabmu,defer_lock);
    if(!locker.try_lock()){
        cout << "already in use!" << endl;
        msleep(100);
        return 0;
    }
    double theta;
    double pitchdown = 45*degtorad;
    int grip = 100;
    if(y < 12){
        cout << "ain't gonna wreck myself!!!" << endl;
        return 0;
    }
    theta = fixtheta(x,temptheta);
    cout << "x=" << x << "  y=" << y << "   theta=" <<theta << endl;
    struct Pos tempopen, tempclosed, obj, objup, objuprotated, checkPos;

    setPos(&objup,x,y,z+10,0,0,-pitchdown,grip);
    setPos(&objuprotated,x,y,z+10,theta,0,-pitchdown,grip);

    line(dump,objup,speed,flip);
    msleep(100);

    grip = 0;
    setPos(&obj,x,y,z,theta,0,-pitchdown,grip);
    line(objup,objuprotated,speed,flip);
    msleep(100);
    line(objuprotated,obj,speed,flip);
    msleep(100);

    setPos(&objup,x,y,z+10,0,0,-pitchdown,grip);
    setPos(&objuprotated,x,y,z+10,theta,0,-pitchdown,grip);
    line(obj,objuprotated,speed,flip);
    msleep(100);
    line(objuprotated,objup,speed,flip);
    msleep(100);
    line(objup,dump,speed,flip);
    locker.unlock();
}

void setArmPos(struct Pos Pos, int flip){
    double x,y,z,a,b,g;
    x = Pos.x; y = Pos.y;  z = Pos.z;
    a = Pos.alpha; b = Pos.beta; g = Pos.gamma;
    int grip = Pos.grip;

    ik.eulerMatrix(a,b,g,t);
    ik.inverseKinematics(x,y,z,t,angles,flip);
    commandArduino(angles,grip);
}

int main(void)
{
    double x,y,z,temptheta;
    double speed = 15; /* in cm/s */
    int flip = 1; /* implement this in a better way later plz...*/
    int toFind = 0;
    int looptieloop = 1;
    vector<double> relPos1(3);
    bool getVecs = false;

    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    Mat relativeMatrix = Mat::zeros(3,3, CV_64F);
    CAM.getMatrixFromFile("CameraCalibration", cameraMatrix, distanceCoefficients);
    /* connect to arduino*/
    arduino = new Serial(portName);
    cout << "is connected: " << arduino->IsConnected() << std::endl;
    /* go to starting position */
    struct Pos dump;
    struct Pos temp;
    setPos(&dump, -20,25,0.5,0,0,-45*degtorad,100);
    setArmPos(dump, flip);

    thread t(&cam::startWebcamMonitoring, &CAM, ref(cameraMatrix), ref(distanceCoefficients), ref(arucoSquareDimension),ref(relPos1) ,ref(relativeMatrix) ,ref(toFind), ref(getVecs), ref(looptieloop) );
    t.detach();
    looptieloop = wait();
    toFind = 42;
    int counter = 0;
    getVecs = true;
    while(looptieloop){
        unique_lock<mutex> locker(mu);
        cond.wait(locker, [&]{return !getVecs;});// I dunno how this lambda thing is doing it, but it works...
        x = 100*relPos1[0]; y = 100*relPos1[1] + 11; z = 0.5;
        temptheta = atan2(relativeMatrix.at<double>(1,0),relativeMatrix.at<double>(0,0));
        locker.unlock();
        getVecs = true;
        toFind++;
        returnBlock(x,y,z,temptheta,speed,flip,dump);
        counter++;
        setPos(&dump, -20,25,0.5 + 3*counter,0,0,-45*degtorad,100);
        //looptieloop = wait();
        if(toFind>44){
            looptieloop = 0;
            break;
        }
    }
    return 1;
}
