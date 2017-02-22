#include <iostream>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <windows.h>
#include <unistd.h>
#include <stdint.h>
#include "IK.h"
#include "Serial.h"
#include "cam.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324
#define precision 4   /* amount of steps per centimeter */
#define increase 4.0      /* I choose to lowest speed to be a factor of 4 smaller than the max speed */
#define ramp 1.0          /* distance over which to increase/decrease speed, set to 1 centimeter */

using namespace cv;
using namespace std;

void msleep(long ms);

Serial *arduino;
IK ik = IK();
cam CAM = cam(0,30);


int16_t ticks[8];
double angles[7] = {0};
double t[3][3]= {{0,1,0},    //the target rotation matrix R
                 {0,0,1},
                 {1,0,0}};

struct Pos{
    long double x; long double y; long double z;
    long double alpha; long double beta; long double gamma; /* euler angles for the target orientation */
    int grip;
};

char outPut[10] = {0};
char const * portName = "\\\\.\\COM3";

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

    ticks[0] = ik.getServoTick(angles[1],0);
    ticks[1] = ik.getServoTick(angles[2],1);
    ticks[2] = ik.getServoTick(pi - angles[2],2);
    ticks[3] = ik.getServoTick(angles[3],3);
    ticks[4] = ik.getServoTick(angles[4],4);
    ticks[5] = ik.getServoTick(angles[5],5);
    ticks[6] = ik.getServoTick(pi - angles[6],6);
    ticks[7] = 400 - 1.5*grip;
    sendStuff(ticks);

}

void msleep(long ms)  /* delay function in miliseconds*/
{
    long us;
    us = 1000*ms;
    struct timespec wait;
    wait.tv_sec = us / (1000 * 1000);
    wait.tv_nsec = (us % (1000 * 1000)) * 1000;
    nanosleep(&wait, NULL);
}

/* this function is a mess and also it's cheating, please ignore */
void line(struct Pos start, struct Pos stap, double speed){ /* speed in cm/s */
    double j;
    double dx,dy,dz,r,x,y,z;
    double wait,current_r;
    double dalpha,dbeta,dgamma;
    double alpha,beta,gamma;
    double temp;
    //double r_a; /* sum of the delta angles*/
    int dgrip;
    int steps;

    double v_max = speed/1000.0;
    double  dr = 1.0/precision; /* amount of steps per centimeter */
    double mindelay = (dr/v_max);  /* amount of ms to wait between 2 steps */
    dx = stap.x - start.x;
    dy = stap.y - start.y;
    dz = stap.z - start.z;
    r = sqrt(dx*dx+dy*dy+dz*dz); /* total path length in centimeters */
    dalpha = stap.alpha - start.alpha;
    dbeta  = stap.beta - start.beta;
    dgamma = stap.gamma - start.gamma;
    //r_a = 12*(dalpha + dbeta + dgamma)/(2*pi); /* a verry rough estimate for the path traveled by the wrist*/
    dgrip = stap.grip - start.grip;
    steps = r*precision;

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
    ik.inverseKinematics(x,y,z,t,angles);
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

void setPos(struct Pos* pos, double x, double y, double z, double alpha, double beta, double gamma,int grip){
    pos->x=x; pos->y=y; pos->z=z;
    pos->alpha=alpha; pos->beta=beta; pos->gamma=gamma;
    pos->grip =  grip;
}

int main(void)
{
///* a change in orientation of the end effector has no function yet */
///* getting the relative rotation is not happening either so far*/
    double hold = 10;
    double speed = 30; /* in cm/s */
    double x,y,z;
    const float arucoSquareDimension = 0.025f; //in meters
    int looptieloop = 1;
    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    vector<double> relPos1(3), relRot1(3); /* relRot is in compact Rodrigues notation NOT EULER ANGLES!!!! */
    Mat relativeMatrix = Mat::eye(3,3, CV_64F);

    CAM.getMatrixFromFile("CameraCalibration", cameraMatrix, distanceCoefficients);

    struct Pos start;
    struct Pos tempopen;
    struct Pos tempclosed;
    struct Pos stop;
    struct Pos obj; //location of the object we want to grab and squeeze real hard yet tenderly
    struct Pos objup;
    setPos(&start,-15,15,1,0,0,-20*degtorad, 100);
    setPos(&stop,10,20,15,-pi/4,0,0,100);
    setPos(&tempopen,-15,25,10,0,0,-20*degtorad, 100);
    setPos(&tempclosed,-15,25,10,0,0,-20*degtorad,0);
    arduino = new Serial(portName);
    cout << "is connected: " << arduino->IsConnected() << std::endl;
    ik.eulerMatrix(0,0,-20*degtorad,t); /* pointed slightly downward */
    ik.inverseKinematics(-15,15,1,t,angles);
    commandArduino(angles,100);


    while(looptieloop == 1){
        looptieloop = CAM.startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension,relPos1,relRot1,49,48);
        if(looptieloop == -1)
            break;
        cout << "x=" << 100*relPos1[0] << "  y=" << 100*relPos1[1] << endl;
        x = 100*relPos1[0] -0.9;
        y = 100*relPos1[1] + 11;
        z = 1;
        setPos(&objup,x,y,10,0,0,-20*degtorad,100); /* first we move above the thingy */
        setPos(&obj,x,y,z,0,0,-20*degtorad,0); /* and now we move down and grab! */
        line(start,tempopen,speed);
        msleep(hold);
        line(tempopen,objup,speed);
        msleep(hold);
        line(objup,obj,speed/4);
        setPos(&objup,x,y,10,0,0,-20*degtorad,0); /* keep the bitch closed */
        msleep(hold);
        line(obj,objup,speed); /* back up */
        msleep(hold);
        line(objup,tempclosed,speed);
        msleep(hold);
        line(tempclosed,start,speed); /* bring it home */
    }

}
