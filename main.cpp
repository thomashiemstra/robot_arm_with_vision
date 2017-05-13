#include <iostream>
#include <thread>
#include <mutex>
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


using namespace std;
/* I feel safer globally defining these*/
mutex mu,grabmu;
condition_variable cond;

//double objectPoints[10][1000000][3];
double ***objectPoints;

char const * portName = "\\\\.\\COM3";
Tricks tricks = Tricks(portName);
PathPlanning pp = PathPlanning(portName);
IK ik2 = IK();
//cam CAM1 = cam(0,30); /* 30 is as high as she'll go*/


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

int main(void){
    //double speed = 25;
    //int flip = 0;

    //tricks.showOff(speed);
    //tricks.monkeySeeMonkeyDo();
    //tricks.stacking(speed,flip);

    struct Pos start, stop;
    tricks.setPos(&start,-15,30,10,0,0,0,10);
    tricks.setPos(&stop,15,30,10,0,0,0,10);

    pp.line(start, stop, 1, 0);

//    int points;
//    int marker = 0;
//    int size = 1000000;
//    allocateCrap(2,size);
//
//
//    pp.createPointsBox(marker,10,objectPoints, points);
//    cout << points << endl;
//    cout << objectPoints[marker][points-1][x_comp] << endl;
//    double res = 10000000;
//    double temp;
//    auto begin = std::chrono::high_resolution_clock::now();
//    for(int i=0; i < points; i++){
//        temp = sqrt(pow(objectPoints[marker][i][x_comp],2) + pow(objectPoints[marker][i][y_comp],2) + pow(objectPoints[marker][i][z_comp],2)  );
//        if(temp < res){
//            res = temp;
//            //record specific point index
//        }
//    }
//    auto end = std::chrono::high_resolution_clock::now();
//    std::cout <<"took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << "ms" << std::endl;
//    freeCrap(2,size);
    return 1;
}
