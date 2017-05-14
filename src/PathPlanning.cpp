#include "PathPlanning.h"
#include "IK.h"
#include "Serial.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <windows.h>
#include <unistd.h>
#include <conio.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

#define x_comp  0
#define y_comp  1
#define z_comp  2
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324

/* parameter for the attractive force, only joints 2,4 and 6 are used since 2,3 and 4,6 share the same origin*/
double c[7] = {0,0,1,0,1,0,0.2};
/* parameter for the repulsive force*/
double n[7] = {0,0,4,0,4,0,3};
/* size of the steps in radians */
double alpha = 0.0001;
double offset = 8; /* extra distance in cm by which to inflate the object*/

double s[3][3]= {{0,1,0},    //the target rotation matrix R
                 {0,0,1},
                 {1,0,0}};

struct Pos{
    double x; double y; double z;
    double alpha; double beta; double gamma; /* euler angles for the target orientation */
    int grip;
};

PathPlanning::PathPlanning(){
    return;
}


int PathPlanning::wait(){
    cout << "press any key to continue or esc to quit" << endl;
    if(getch() == 27)
        return 0;
    else
        return 1;
}

void PathPlanning::sendStuff(int16_t *val){ //sending 7 2 byte ints over serial
	uint8_t bytes[16];
    for(int i =0; i < 8; i++){
        bytes[2*i] = (val[i] >> 8) &0xff;
        bytes[2*i+1] = val[i] & 0xff;
    }
	arduino->WriteData(bytes,16);
}

void PathPlanning::commandArduino(double angles[7], int grip){
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

void PathPlanning::rotTrans(int marker, double*** objectPoints, int points, double theta, vector<double>& trans){
    int i;
    double x,y;
    double c = cos(theta);
    double s = sin(theta);

    for(i=0; i<points; i++){
        x = objectPoints[marker][i][x_comp];
        y = objectPoints[marker][i][y_comp];

        objectPoints[marker][i][x_comp] = (c*x - s*y) + trans[x_comp] - offset/2.0;
        objectPoints[marker][i][y_comp] = (s*x + c*y) + trans[y_comp] - offset/2.0;
    }

}

void PathPlanning::createPointsBox(int marker, double density, double*** objectPoints, int& points){
    double x,y,z;
    int i,j,k;
    int current_point = 0;
    /* fill this based on the marker number*/
    double dims[3] = {7.5,21,21};
    dims[x_comp] += offset;
    dims[y_comp] += offset;
    dims[z_comp] += offset;
    int points_x = dims[x_comp]*density;
    int points_y = dims[y_comp]*density;
    int points_z = dims[z_comp]*density;
    double dl = 1.0/density;
    //cout << points_x << endl;
    /* top */
    k=0;
    for(i=0; i<= points_x; i++){
        for(j=0; j<= points_y; j++){
            x = i*dl;
            y = j*dl;
            z = dims[z_comp];
            objectPoints[marker][current_point][x_comp] = x;
            objectPoints[marker][current_point][y_comp] = y;
            objectPoints[marker][current_point][z_comp] = z;
            //cout << current_point << " x=" <<  x << "\t y=" << y << "\t z=" << z << endl;
            current_point += 1;
        }
    }
    //cout << "---------------" << endl;
    /* back and front */
    for(i=0; i<= points_x; i++){
        for(j=0; j< points_z; j++){
            for(k=0; k < 2; k++ ){
                x = i*dl;
                z = j*dl;
                y = k*dims[y_comp];
                objectPoints[marker][current_point][x_comp] = x;
                objectPoints[marker][current_point][y_comp] = y;
                objectPoints[marker][current_point][z_comp] = z;
                //cout << current_point << "\t x=" <<  x << "\t y=" << y << "\t z=" << z << endl;
                current_point += 1;
            }

        }
    }
    //cout << "---------------" << endl;
    /* sides */
    for(i=1; i< points_y; i++){
        for(j=0; j< points_z; j++){
            for(k=0; k < 2; k++ ){
                y = i*dl;
                z = j*dl;
                x = k*dims[x_comp];
                objectPoints[marker][current_point][x_comp] = x;
                objectPoints[marker][current_point][y_comp] = y;
                objectPoints[marker][current_point][z_comp] = z;
                //cout << current_point << "\t x=" <<  x << "\t y=" << y << "\t z=" << z << endl;
                current_point += 1;
            }

        }
    }
    points = current_point;
}
/* d is cutoff distance beyond which the F_world = 0*/
void PathPlanning::getRepulsiveForceWorld(double F_world[7][3], double angles_current[7], int marker, double*** objectPoints, int points, double d){
    int i,j;
    double temp;
    double currentPos[7][3];
    int point; /* index of the point on the object closest to control point i*/
    double absD; /* magnitude of the distance between control point and object*/
    bool calc;


    ik.forwardKinematics(angles_current, currentPos);


    double res = d;
    for(i=2; i<7; i +=2){
        calc = false;
        for(j=0; j<points-1; j++){
            temp = sqrt(pow(currentPos[i][x_comp] - objectPoints[marker][j][x_comp], 2) + pow(currentPos[i][y_comp] - objectPoints[marker][j][y_comp], 2) + pow(currentPos[i][z_comp] - objectPoints[marker][j][z_comp], 2));
            if(temp < res){
                res = temp;
                point = j;
                calc = true;
            }
        }
        //cout << "calc= " << calc << endl;



        if(calc){
            absD = sqrt( pow(currentPos[i][x_comp] - objectPoints[marker][point][x_comp],2) +  pow(currentPos[i][y_comp] - objectPoints[marker][point][y_comp],2) + pow(currentPos[i][z_comp] - objectPoints[marker][point][z_comp],2));
            F_world[i][x_comp] = n[i]*((1.0/res) - 1.0/d)*(1.0/pow(res,2))*(currentPos[i][x_comp] - objectPoints[marker][point][x_comp])/absD;
            F_world[i][y_comp] = n[i]*((1.0/res) - 1.0/d)*(1.0/pow(res,2))*(currentPos[i][y_comp] - objectPoints[marker][point][y_comp])/absD;
            F_world[i][z_comp] = n[i]*((1.0/res) - 1.0/d)*(1.0/pow(res,2))*(currentPos[i][z_comp] - objectPoints[marker][point][z_comp])/absD;
        }
        else{
             F_world[i][x_comp] =  F_world[i][y_comp] =  F_world[i][z_comp] = 0;
        }
        res = d;
    }

}

/* needs raw angles, d is the cutoff distance between linear and quadratic potential */
void PathPlanning::getAttractiveForceWorld(double F_world[7][3], double angles_final[7], double angles_current[7], double d){
    double currentPos[7][3];
    double goalPos[7][3];

    ik.forwardKinematics(angles_final, goalPos);
    ik.forwardKinematics(angles_current, currentPos);
    /* no quadratic term yet: d not used*/
    /* only joint 2, 4 and 6 are being evaluated since 1 does nothing, 2=3 and 4=5*/
    for(int i=2; i<7; i +=2){
        F_world[i][x_comp] = -c[i]*(currentPos[i][x_comp] - goalPos[i][x_comp]);
        F_world[i][y_comp] = -c[i]*(currentPos[i][y_comp] - goalPos[i][y_comp]);
        F_world[i][z_comp] = -c[i]*(currentPos[i][z_comp] - goalPos[i][z_comp]);
    }
}
/* marker should become an array at one point*/
void PathPlanning::line(struct Pos start, struct Pos stop, int time, int flip, double*** objectPoints, int marker, int points){
    int i;
    double stopAngles[7] = {0};
    double currentAngles[7] = {0};
    double F_world[7][3];
    double F_joints[7];
    double absF;
    double temp1,temp2;
    double finishedCondition;
    bool done = false;
    double servoAngles[7];
    int counter;

    ik.eulerMatrix(start.alpha, start.beta, start.gamma,s);
    ik.inverseKinematicsRaw(start.x, start.y, start.z, s,currentAngles, flip);
    ik.eulerMatrix(stop.alpha, stop.beta, stop.gamma,s);
    ik.inverseKinematicsRaw(stop.x, stop.y, stop.z, s,stopAngles, flip);
    /* just to be safe*/
    for(i=0; i<7; i++){
        F_joints[i] = 0;
    }

    /* get first force*/
    getAttractiveForceWorld(F_world, stopAngles, currentAngles, 1);
    cout << "fx attractive=" << F_world[6][x_comp] << endl;
    ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);
    getRepulsiveForceWorld(F_world, currentAngles, marker, objectPoints, points, 5);
    cout << "fx repulsive=" << F_world[6][x_comp] << endl;
    cout << "fy repulsive=" << F_world[6][y_comp] << endl;
    cout << "fz repulsive=" << F_world[6][z_comp] << endl;
    ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);

    ik.convertAngles(currentAngles,servoAngles);
    commandArduino(servoAngles,10);

    wait();

    auto begin = std::chrono::high_resolution_clock::now();
    while(!done){
        temp1 = temp2 = 0;
        for(i=1; i<7; i++){
            temp1 += pow(currentAngles[i-1] - stopAngles[i-1],2);
            temp2 += pow(F_joints[i],2);
        }
        absF = sqrt(temp2);
        finishedCondition = sqrt(temp1);

        //cout << finishedCondition << endl;
        //cout << "d1=" << currentAngles[5] - stopAngles[5] << endl;

        if(counter > 100){
            ik.convertAngles(currentAngles,servoAngles);
            commandArduino(servoAngles,10);
            counter = 0;
        }

        //std::this_thread::sleep_for(std::chrono::microseconds(1));
        //cout << "d4=" << currentAngles[4] - stopAngles[4] << endl;

        if(finishedCondition < 0.4){
            done = true;
        }
        //cout << "\r" << " finished=" << finishedCondition  <<"                   " << flush;
        for(i=1; i<7; i++){
            currentAngles[i] += alpha*F_joints[i]/absF;
        }

        for(i=0; i<7; i++){
            F_joints[i] = 0;
        }
        getAttractiveForceWorld(F_world,stopAngles, currentAngles , 1);
        ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);
        getRepulsiveForceWorld(F_world, currentAngles, marker, objectPoints, points, 5);
        ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);
        counter++;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "this shit took " <<std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << "ms" << std::endl;
    for(i=1; i<7; i++){
        cout << currentAngles[i] << endl;
    }
    cout << "done" << endl;
}
