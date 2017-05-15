#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include "IK.h"
#include "Serial.h"
#include "cam.h"
#include "tricks.h"
#include "PathPlanning.h"
#include <algorithm>
#include <chrono>
#include <math.h>

#define x_comp  0
#define y_comp  1
#define z_comp  2
#define d4 13

#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324

using namespace std;
/* I feel safer globally defining these*/
mutex mu,grabmu;
condition_variable cond;

//double objectPoints[10][1000000][3];
double ***objectPoints;

char const * portName = "\\\\.\\COM3";
Tricks tricks = Tricks();
PathPlanning pp = PathPlanning();
cam CAM = cam(0,30); /* 30 is as high as she'll go*/
IK ik = IK();
Serial *arduino;

const float arucoSquareDimension = 0.0265f; //in meters

double w[3][3]= {{0,1,0},    //the target rotation matrix R
                 {0,0,1},
                 {1,0,0}};

struct Pos{
    double x; double y; double z;
    double alpha; double beta; double gamma; /* euler angles for the target orientation */
    int grip;
};
/* points should be the amount for the largest object */
void allocateCrap(int markers, int points){
    int i,j;
    objectPoints = (double ***)malloc(markers*sizeof(double **));
    for(i=0; i<markers; i++){
        objectPoints[i] = (double **)malloc(points*sizeof(double *));
        for(j=0; j<points; j++){
            objectPoints[i][j] = (double *)malloc(3*sizeof(double));
        }
    }

}

void freeCrap(int markers, int points){
    int i,j;
    for(i=0; i<markers; i++){
        for(j=0; j<points; j++){
            free(objectPoints[i][j]);
        }
    }
     for(i=0; i<markers; i++){
        free(objectPoints[i]);
     }
     free(objectPoints);
}

int wait(){
    cout << "press any key to continue or esc to quit" << endl;
    if(getch() == 27)
        return 0;
    else
        return 1;
}

int main(void){
    arduino = new Serial(portName);
    cout << "is connected: " << arduino->IsConnected() << std::endl;

    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    Mat relativeMatrix = Mat::zeros(3,3, CV_64F);
    CAM.getMatrixFromFile("CameraCalibration720.dat", cameraMatrix, distanceCoefficients);

    vector<double> relPos1(3);

    double theta;

    struct Pos start, stop;
    tricks.setPos(&start,-20,30,10,0,0,0,10);
    tricks.setPos(&stop,20,25,25,0,0,0,10);

    int pointDensity = 10;
    int points;
    int marker = 10;
    int totalmarkers = 21;
    /* CAREFULL! make sure this is large enough (just figure it out before hand and allocate it!)*/
    int size = 1000000;
    allocateCrap(totalmarkers,size);

    pp.createPointsBox(marker,pointDensity,objectPoints, points);

    cout << objectPoints[marker][45396][0] << endl;
    cout << objectPoints[marker][45396][1] << endl;
    cout << objectPoints[marker][45396][2] << endl;
    cout << "---------------" << endl;


    CAM.findVecsCharuco(cameraMatrix, distanceCoefficients, arucoSquareDimension, relPos1, relativeMatrix, 10);
    theta = atan2(relativeMatrix.at<double>(1,0),relativeMatrix.at<double>(0,0));


    /* x,y in centimeters */
    relPos1[0] = 100*relPos1[0]; relPos1[1]  = 100*relPos1[1] + 10.5;
    pp.rotTrans(10, objectPoints, points, theta, relPos1);


    cout << objectPoints[marker][45396][0] << endl;
    cout << objectPoints[marker][45396][1] << endl;
    cout << objectPoints[marker][45396][2] << endl;
    cout << "---------------" << endl;


    pp.line(start,stop,20,0,objectPoints,marker,points);
    wait ();
    tricks.pointToPoint(stop, start, 2, 0);
    freeCrap(2,size);
    return 1;
}
