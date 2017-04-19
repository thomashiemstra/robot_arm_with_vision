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
#include "IK.h"
#include "Serial.h"
#include "cam.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324
#define precision 5   /* amount of steps per centimeter */
#define increase 6.0      /* I choose to lowest speed to be a factor of 4 smaller than the max speed */
#define ramp 2.0          /* distance over which to increase/decrease speed, set to 1 centimeter */

using namespace cv;
using namespace std;

void msleep(long ms);

Serial *arduino;
IK ik = IK();
cam CAM = cam(0,60);

const float arucoSquareDimension = 0.0265f; //in meters
double angles[7] = {0};
char outPut[10] = {0};
char const * portName = "\\\\.\\COM3";
double t[3][3]= {{0,1,0},    //the target rotation matrix R
                 {0,0,1},
                 {1,0,0}};

struct Pos{
    long double x; long double y; long double z;
    long double alpha; long double beta; long double gamma; /* euler angles for the target orientation */
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
void line(struct Pos start, struct Pos stop, double speed, int flip){ /* speed in cm/s */
    double j;
    double dx,dy,dz,r,x,y,z;
    double wait,current_r;
    double dalpha,dbeta,dgamma;
    double alpha,beta,gamma;
    double temp;
    double r_a; /* sum of the delta angles*/
    int dgrip;
    int steps;

    double v_max = speed/1000.0;
    double  dr = 1.0/precision; /* amount of steps per centimeter */
    double mindelay = (dr/v_max);  /* amount of ms to wait between 2 steps */
    dx = stop.x - start.x;
    dy = stop.y - start.y;
    dz = stop.z - start.z;
    r = sqrt(dx*dx+dy*dy+dz*dz); /* total path length in centimeters */
    dalpha = stop.alpha - start.alpha;
    dbeta  = stop.beta - start.beta;
    dgamma = stop.gamma - start.gamma;
    r_a = abs((dalpha + dbeta + dgamma)/pi); /* a very rough estimate for the path traveled by the wrist*/
    dgrip = stop.grip - start.grip;
    steps = r*precision;
    if(r == 0){
        steps = r_a*200;
        cout << "r_a=" << r_a << endl;
    }

    /* v = a*r²+b for current_r<=ramp, v = -c*r²+d for current_r>= r - ramp*/
    /* why not just linear? */
    double a = (1.0/(ramp*ramp) )*( (dr/mindelay)-(dr/(increase*mindelay)) );
    double b = dr/(increase*mindelay);
    double c = ( (dr/mindelay) - (dr/(increase*mindelay)) ) / (2*r*ramp - ramp*ramp);
    double d = (dr/(increase*mindelay)) + c*r*r;

    for(j = 0; j <= steps; j++){
    current_r = dr*j;
    x = start.x + ((j/steps)*dx);
    y = start.y + ((j/steps)*dy);
    z = start.z + ((j/steps)*dz);
    alpha = start.alpha + ((j/steps)*dalpha);
    beta = start.beta + ((j/steps)*dbeta);
    gamma = start.gamma + ((j/steps)*dgamma);
    ik.eulerMatrix(alpha,beta,gamma,t);
    ik.inverseKinematics(x,y,z,t,angles,flip);
    commandArduino(angles,start.grip);
    /* mess due to accelleration */
        if(r<=2*ramp){ /* path too short, v = 0.5*v_max everywhere */
            msleep(2*mindelay);
        }
        else if(r>2*ramp){  /* v = a*current_r² + b*/
            if (current_r <= ramp){
                wait = dr / ( a*current_r*current_r + b);  /* v = dr/delay so delay = dr/v */
                msleep(wait);
            }
        else if(current_r>ramp && current_r<r-ramp){
            msleep(wait);
        }
        else if(current_r >= r-ramp){ /*v = -c*current_r² + d */
            wait = dr/( -c*current_r*current_r + d);
            msleep(wait);
        }
        }
/* end of mess */
    }
    if(abs(dgrip) > 0){
        for (j=0;j<20;j++){
            temp = start.grip + (j/20)*dgrip;
            commandArduino(angles,temp);
            msleep(20);
        }
    }
}

void lineTest(struct Pos start, struct Pos stop, double speed){
    double j;
    double dx,dy,dz,r,x,y,z;
    double wait,current_r;
    double dalpha,dbeta,dgamma;
    double alpha,beta,gamma;
    double temp;
    vector<vector<double >> anglesArray;
    double r_a; /* sum of the delta angles*/
    int dgrip;
    int steps;

    double v_max = speed/1000.0;
    double  dr = 1.0/precision; /* amount of steps per centimeter */
    double mindelay = (dr/v_max);  /* amount of ms to wait between 2 steps */
    dx = stop.x - start.x;
    dy = stop.y - start.y;
    dz = stop.z - start.z;
    r = sqrt(dx*dx+dy*dy+dz*dz); /* total path length in centimeters */
    dalpha = stop.alpha - start.alpha;
    dbeta  = stop.beta - start.beta;
    dgamma = stop.gamma - start.gamma;
    //r_a = 12*(dalpha + dbeta + dgamma)/(2*pi); /* a verry rough estimate for the path traveled by the wrist*/
    dgrip = stop.grip - start.grip;
    steps = r*precision;

    anglesArray.resize(steps);
    for (int i = 0; i < steps; ++i)
        anglesArray[i].resize(7);

    double a = (1.0/(ramp*ramp) )*( (dr/mindelay)-(dr/(increase*mindelay)) );
    double b = dr/(increase*mindelay);
    double c = ( (dr/mindelay) - (dr/(increase*mindelay)) ) / (2*r*ramp - ramp*ramp);
    double d = (dr/(increase*mindelay)) + c*r*r;

    for(j=0; j<steps; j++){
        x = start.x + ((j/steps)*dx);
        y = start.y + ((j/steps)*dy);
        z = start.z + ((j/steps)*dz);
        alpha = start.alpha + ((j/steps)*dalpha);
        beta = start.beta + ((j/steps)*dbeta);
        gamma = start.gamma + ((j/steps)*dgamma);
        ik.eulerMatrix(alpha,beta,gamma,t);
        ik.inverseKinematicsTest(x,y,z,t,anglesArray[j]);
    }
    for(j=0; j<steps; j++){
        commandArduinoTest(anglesArray[j],start.grip);
            /* mess due to accelleration */
        if(r<=2*ramp){ /* path too short, v = 0.5*v_max everywhere */
            msleep(2*mindelay);
        }
        else if(r>2*ramp){  /* v = a*current_r² + b*/
            if (current_r <= ramp){
                wait = dr / ( a*current_r*current_r + b);  /* v = dr/delay so delay = dr/v */
                msleep(wait);
            }
        else if(current_r>ramp && current_r<r-ramp){
            msleep(wait);
        }
        else if(current_r >= r-ramp){ /*v = -c*current_r² + d */
            wait = dr/( -c*current_r*current_r + d);
            msleep(wait);
        }
        }
/* end of mess */
    }

}

void setPos(struct Pos* pos, double x, double y, double z, double alpha, double beta, double gamma,int grip){
    pos->x=x; pos->y=y; pos->z=z;
    pos->alpha=alpha; pos->beta=beta; pos->gamma=gamma;
    pos->grip =  grip;
}

int wait(){
    printf("press any key to continue or esc to quit \n");
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


int main(void)
{
    int counter = 0;
    double speed = 30; /* in cm/s */
    double pitchdown = 45*degtorad;
    double x,y,z,theta,temptheta;
    int flip = 1;
    int grip = 100;
    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    Mat relativeMatrix = Mat::zeros(3,3, CV_64F);
    vector<double> relPos1(3);
    CAM.getMatrixFromFile("CameraCalibration", cameraMatrix, distanceCoefficients);

    struct Pos start, stop, tempopen, tempclosed, obj, objup, objuprotated, checkPos;
    setPos(&start,-20,25,5,0,0,-pitchdown,grip);
    //setPos(&stop,20,25,15,0,0,0,grip);

    arduino = new Serial(portName);
    cout << "is connected: " << arduino->IsConnected() << std::endl;

    ik.eulerMatrix(0,0,-pitchdown,t);
    ik.inverseKinematics(-20,25,5,t,angles,flip);
    commandArduino(angles,grip);

    int looptieloop = wait();
    while(looptieloop == 1){

        CAM.findVecsCharuco(cameraMatrix, distanceCoefficients, arucoSquareDimension,relPos1,relativeMatrix,42);
        temptheta = atan2(relativeMatrix.at<double>(1,0),relativeMatrix.at<double>(0,0));
        x = 100*relPos1[0] -0.5;
        y = 100*relPos1[1] + 12;
        z = 2;
        theta = fixtheta(x,temptheta);
        cout << "x=" << x << "  y=" << y << "   theta=" <<theta << endl;
        grip = 100;
        setPos(&objup,x,y,z+10,0,0,-pitchdown,grip);
        setPos(&objuprotated,x,y,z+10,theta,0,-pitchdown,grip);

        line(start,objup,speed,flip);
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
        line(objup,start,speed,flip);

        looptieloop = wait();
    }

}
