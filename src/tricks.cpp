#include "tricks.h"
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


const float arucoSquareDimension = 0.0265f; //in meters
double angles[7] = {0};
double t[3][3]= {{0,1,0},    //the target rotation matrix R
                 {0,0,1},
                 {1,0,0}};

struct Pos{
    double x; double y; double z;
    double alpha; double beta; double gamma; /* euler angles for the target orientation */
    int grip;
};

Tricks::Tricks(){
    return;
}

void Tricks::msleep(long ms){  /* delay function in miliseconds*/
    long us;
    us = 1000*ms;
    struct timespec wait;
    wait.tv_sec = us / (1000 * 1000);
    wait.tv_nsec = (us % (1000 * 1000)) * 1000;
    nanosleep(&wait, NULL);
}

void Tricks::sendStuff(int16_t *val){ //sending 7 2 byte ints over serial
	uint8_t bytes[16];
    for(int i =0; i < 8; i++){
        bytes[2*i] = (val[i] >> 8) &0xff;
        bytes[2*i+1] = val[i] & 0xff;
    }
	arduino->WriteData(bytes,16);
}
/* gripper position is a percentage, 100% is open*/
void Tricks::commandArduino(double angles[7], int grip){
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

void Tricks::setArmPos(struct Pos Pos, int flip){
    double x,y,z,a,b,g;
    x = Pos.x; y = Pos.y;  z = Pos.z;
    a = Pos.alpha; b = Pos.beta; g = Pos.gamma;
    int grip = Pos.grip;
    ik.eulerMatrix(a,b,g,t);
    ik.inverseKinematics(x,y,z,t,angles,flip);
    commandArduino(angles,grip);
}

void Tricks::line(struct Pos start, struct Pos stop, double speed, int flip){

    int dgrip = stop.grip - start.grip;
    double dx = stop.x - start.x;
    double dy = stop.y - start.y;
    double dz = stop.z - start.z;
    double r = sqrt(dx*dx+dy*dy+dz*dz);
    double time = r/speed; /* speed in cm/second, time in seconds*/
    int steps = ceil(time*20);

    for(int k=0; k<steps; k++){
        double t1 = (double)k/steps; /* t1 has to go from 0 to 1*/
        double x = start.x + 3*(stop.x - start.x)*pow(t1,2) - 2*(stop.x - start.x)*pow(t1,3);
        double y = start.y + 3*(stop.y - start.y)*pow(t1,2) - 2*(stop.y - start.y)*pow(t1,3);
        double z = start.z + 3*(stop.z - start.z)*pow(t1,2) - 2*(stop.z - start.z)*pow(t1,3);
        ik.eulerMatrix(start.alpha, start.beta, start.gamma,t);
        ik.inverseKinematics(x, y, z, t,angles, flip);
        commandArduino(angles,start.grip);
        msleep(50);
    }
    if(abs(dgrip) > 0){
        for (int j=0;j<20;j++){
            int temp = ceil(start.grip + (j/20)*dgrip);
            commandArduino(angles,temp);
            msleep(20);
        }
    }
}

/*move from point to point in x amount of seconds, */
void Tricks::pointToPoint(struct Pos start, struct Pos stop, double time, int flip){
    double startAngles[7] = {0};
    double stopAngles[7] = {0};
    double tempAngle[7] = {0};
    int dgrip = stop.grip - start.grip;

    ik.eulerMatrix(start.alpha, start.beta, start.gamma,t);
    ik.inverseKinematics(start.x, start.y, start.z, t,startAngles, flip);
    ik.eulerMatrix(stop.alpha, stop.beta, stop.gamma,t);
    ik.inverseKinematics(stop.x, stop.y, stop.z, t,stopAngles, flip);

    int steps = ceil(time*20); /* maybe steps should scale not with time but with path length, dunno */

    for(int k=0; k<steps; k++){
        double t = (double)k/steps; /* t has to go from 0 to 1*/
        for(int i=1; i<7; i++){
            tempAngle[i] = startAngles[i] + 3*(stopAngles[i] - startAngles[i])*pow(t,2) - 2*(stopAngles[i] - startAngles[i])*pow(t,3);
        }
        commandArduino(tempAngle, start.grip); /* grip not implemented yet */
        msleep(50);
    }
    if(abs(dgrip) > 0){
        for (int j=0;j<20;j++){
            int temp = ceil(start.grip + (j/20)*dgrip);
            commandArduino(angles,temp);
            msleep(20);
        }
    }
}

void Tricks::anglesToAngles(double startAngles[7], double stopAngles[7], double time, int flip, int grip){
    double tempAngle[7] = {0};

    int steps = ceil(time*20); /* maybe steps should scale not with time but with path length, dunno */

    for(int k=0; k<steps; k++){
        double t = (double)k/steps; /* t has to go from 0 to 1*/
        for(int i=1; i<7; i++){
            tempAngle[i] = startAngles[i] + 3*(stopAngles[i] - startAngles[i])*pow(t,2) - 2*(stopAngles[i] - startAngles[i])*pow(t,3);
        }
        commandArduino(tempAngle, grip); /* grip not implemented yet */
        msleep(50);
    }

}

void Tricks::setPos(struct Pos* pos, double x, double y, double z, double alpha, double beta, double gamma,int grip){
    pos->x=x; pos->y=y; pos->z=z;
    pos->alpha=alpha; pos->beta=beta; pos->gamma=gamma;
    pos->grip =  grip;
}

int Tricks::wait(){
    cout << "press any key to continue or esc to quit" << endl;
    if(getch() == 27)
        return 0;
    else
        return 1;
}

//TODO look at behaviour for abs(x)>10
double Tricks::fixtheta(double theta){

        if(theta > pi/4.0 && theta <= 3*pi/4.0)
            theta -= pi/2.0;

        else if(theta > 3*pi/4.0)
            theta -= pi;

        else if(theta < -pi/4.0 && theta >= -3*pi/4.0)
            theta += pi/2.0;

        else if(theta < -3*pi/4.0)
            theta += pi;

        return theta;
}

void Tricks::showOff(double speed){
    int flip = 0;
    int j;
    struct Pos start, leftlow, rightlow, leftup, rightup;
    struct Pos start1,start2,start3;
    setPos(&start,0,25,20,0,0,0,10);
    setPos(&leftlow,-20,30,6,0,0,0,10);
    setPos(&rightlow,20,30,6,0,0,0,10);
    setPos(&leftup,-20,30,30,0,0,0,10);
    setPos(&rightup,20,30,30,0,0,0,10);

    setArmPos(start,flip);
    wait();
    line(start,leftlow,speed,flip);
    line(leftlow,leftup,speed,flip);
    flip = 1;
    setArmPos(leftup,flip);
    msleep(500);
    line(leftup,rightup,speed,flip);
    flip = 0;
    setArmPos(rightup,flip);
    msleep(500);
    line(rightup,rightlow,speed,flip);
    line(rightlow,leftlow,speed,flip);
    line(leftlow,start,speed,flip);

    setPos(&start1,0,25,20,pi/2,0,0,10);
    setPos(&start2,0,25,20,-pi/2,0,0,10);
    setPos(&start3,0,25,20,pi/2,0,0,10);
    line(start,start1,speed,flip);
    line(start1,start2,speed,flip);
    line(start2,start3,speed,flip);
    double dummy = 70;

    /* I don't have a function to draw circles with the wrist, don't think I need one either tbh.*/
    for (j=0; j<=dummy; j++){
       ik.eulerMatrix(cos((j/dummy)*pi)*pi/2,0,sin((j/dummy)*pi)*pi/2,t);
       ik.inverseKinematics(0,25,20,t,angles,flip);
       commandArduino(angles,10);
       msleep(50);
   }

   for (j=0; j<=dummy; j++){
       ik.eulerMatrix(-cos((j/dummy)*pi)*pi/2,0,-sin((j/dummy)*pi)*pi/2,t);
       ik.inverseKinematics(0,25,20,t,angles,flip);
       commandArduino(angles,10);
       msleep(50);
    }
    line(start3,start,speed,flip);
}
/* I hate this function with a burning passion*/
int Tricks::returnBlock(double x, double y, double z, double temptheta, double speed, int flip, struct Pos& drop,int counter){
    unique_lock<mutex> locker(grabmu,defer_lock);
    if(!locker.try_lock()){
        cout << "already in use!" << endl;
        msleep(100);
        return 0;
    }
    double delta = 3.1; /* height of the block */
    double theta,r,time;
    double pitchdown = -45*degtorad;
    if(y < 12){
        cout << "ain't gonna wreck myself!!!" << endl;
        return 0;
    }
    theta = fixtheta(temptheta);
    cout << "x=" << x << "  y=" << y << "   theta=" <<theta << endl;
    struct Pos  obj, objup, dropup;

    setPos(&dropup, drop.x,drop.y,drop.z + delta,0,0,drop.gamma,drop.grip);
    setPos(&objup, x,y,z + 10,theta,0,pitchdown,100);
    setPos(&obj,x,y,z,theta,0,pitchdown,0);

    r = sqrt(pow(dropup.x-objup.x,2) + pow(dropup.x - objup.x,2));
    time = r/speed;

    line(drop,dropup,speed,flip);
    pointToPoint(dropup,objup,time,flip);
    line(objup,obj,speed/2,flip);

    /* there should be a better way to close the gripper at all points...*/
    setPos(&objup, x,y,z + 10,theta,0,pitchdown,0);
    setPos(&dropup, drop.x,drop.y,drop.z + delta,0,0,drop.gamma,0);

    line(obj,objup,speed,flip);
    pointToPoint(objup,dropup,time,flip);
    line(dropup,drop,speed/4,flip);

    setPos(&dropup, drop.x,drop.y,drop.z + delta,0,0,drop.gamma,100);
    line(drop,dropup,speed,flip);

    setPos(&drop,drop.x,drop.y,drop.z + delta,0,0,drop.gamma,drop.grip);

    locker.unlock();
    return 1;
}

void Tricks::stacking(double speed, int flip){
    double x,y,z,temptheta;
    int toFind = 42;
    int looptieloop = 1;
    vector<double> relPos1(3);
    bool getVecs = false;
    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    Mat relativeMatrix = Mat::zeros(3,3, CV_64F);
    CAM.getMatrixFromFile("CameraCalibration720.dat", cameraMatrix, distanceCoefficients);

    struct Pos drop;
    setPos(&drop, -20,25,2,0,0,-45*degtorad,100);
    setArmPos(drop, flip);

    thread t(&cam::startWebcamMonitoring, &CAM, ref(cameraMatrix), ref(distanceCoefficients), ref(arucoSquareDimension),ref(relPos1) ,ref(relativeMatrix) ,ref(toFind), ref(getVecs), ref(looptieloop) );
    t.detach();
    looptieloop = wait();
    int counter = 0;
    while(looptieloop){
        getVecs = true;
        unique_lock<mutex> locker(mu);
        cond.wait(locker, [&]{return !getVecs;});
        x = 90*relPos1[0] + 1; y = 100*relPos1[1] + 10.5; z = 2;
        temptheta = atan2(relativeMatrix.at<double>(1,0),relativeMatrix.at<double>(0,0));
        locker.unlock();
        toFind++;
        returnBlock(x,y,z,temptheta,speed,flip,drop,counter);
        counter++;
        if(toFind>46){
            looptieloop = 0;
            break;
        }
    }
}

void Tricks::monkeySeeMonkeyDo(){
    double x,y,z;
    vector<double> relPos1(3);
    bool getVecs = false;
    int looptieloop = 1;
    int flip = 0;
    double w[3][3]={{0,1,0},    //the target rotation matrix R
                    {0,0,1},
                    {1,0,0}};
    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    Mat relativeMatrix = Mat::zeros(3,3, CV_64F);
    CAM.getMatrixFromFile("CameraCalibration720.dat", cameraMatrix, distanceCoefficients);

    thread t(&cam::copyMovement, &CAM, ref(cameraMatrix), ref(distanceCoefficients),ref(relPos1) ,ref(relativeMatrix), ref(getVecs), ref(looptieloop) );
    t.detach();
    looptieloop = wait();
    getVecs = true;
    while(true){
        unique_lock<mutex> locker(mu);
        cond.wait(locker, [&]{return !getVecs;});
            x = 90*relPos1[0] - 4*relativeMatrix.at<double>(0,1) - 27 ; y = 100*relPos1[1] + 30 - 4*relativeMatrix.at<double>(1,1); z = 100*relPos1[2] + 5 - 6*relativeMatrix.at<double>(2,1);
            //cout << "\r" << " x=" << x << " y=" << y << "  z" << z <<"                   " << flush;
            w[0][0] = relativeMatrix.at<double>(0,2);   w[0][1] = relativeMatrix.at<double>(0,0);   w[0][2] = relativeMatrix.at<double>(0,1);
            w[1][0] = relativeMatrix.at<double>(1,2);   w[1][1] = relativeMatrix.at<double>(1,0);   w[1][2] = relativeMatrix.at<double>(1,1);
            w[2][0] = relativeMatrix.at<double>(2,2);   w[2][1] = relativeMatrix.at<double>(2,0);   w[2][2] = relativeMatrix.at<double>(2,1);
            ik.inverseKinematics(x,y,z,w,angles,flip);
            commandArduino(angles,10);
            locker.unlock();
            getVecs = true;
            msleep(20);
    }
}
