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
double tempAngles[7] = {0};
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
void Tricks::commandArduino(double anglez[7], int grip){
    int16_t ticks[8];
    ticks[0] = ik.getServoTick(anglez[1],0);
    ticks[1] = ik.getServoTick(pi - anglez[2],1);
    ticks[2] = ik.getServoTick(anglez[2],2);
    ticks[3] = ik.getServoTick(anglez[3],3);
    ticks[4] = ik.getServoTick((pi - anglez[4]),4);
    ticks[5] = ik.getServoTick(anglez[5],5);
    ticks[6] = ik.getServoTick((pi - anglez[6]),6);
    ticks[7] = 700 - 3.5*grip;
    sendStuff(ticks);
}

void Tricks::setArmPos(struct Pos Pos, int flip){
    double x,y,z,a,b,g;
    x = Pos.x; y = Pos.y;  z = Pos.z;
    a = Pos.alpha; b = Pos.beta; g = Pos.gamma;
    int grip = Pos.grip;
    ik.eulerMatrix(a,b,g,t);

    double anglesInternal[6] = {0,0,0,0,-1,0};

    //ik.inverseKinematicsRaw(x,y,z,t,angles,flip);
    //ik.inverseKinematicsNNRawDelta(x,y,z,t,anglesInternal,tempAngles);
    ik.inverseKinematicsNNRaw(x, y, z, t,tempAngles, flip);
    ik.convertAngles(tempAngles,angles);

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
    double anglesInternal[6] = {0,0,0,0,-1,0};

    for(int k=0; k<steps; k++){
        double t1 = (double)k/steps; /* t1 has to go from 0 to 1*/
        double x = start.x + 3*(stop.x - start.x)*pow(t1,2) - 2*(stop.x - start.x)*pow(t1,3);
        double y = start.y + 3*(stop.y - start.y)*pow(t1,2) - 2*(stop.y - start.y)*pow(t1,3);
        double z = start.z + 3*(stop.z - start.z)*pow(t1,2) - 2*(stop.z - start.z)*pow(t1,3);
        ik.eulerMatrix(start.alpha, start.beta, start.gamma,t);

        //ik.inverseKinematicsRaw(x, y, z, t,tempAngles, flip);
        //ik.inverseKinematicsNNRawDelta(x,y,z,t,anglesInternal,tempAngles);
        ik.inverseKinematicsNNRaw(x, y, z, t,tempAngles, flip);
        ik.convertAngles(tempAngles,angles);

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

    int steps = ceil(time*20); /* maybe steps should scale not with time but with path length */

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
