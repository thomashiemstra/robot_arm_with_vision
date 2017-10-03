#include "IK_NN.h"
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include<vector>
#include <chrono>
#include "doublefann.h"

#define x_comp  0
#define y_comp  1
#define z_comp  2

#define sx_comp     3
#define sy_comp     4
#define sz_comp     5

#define ax_comp     6
#define ay_comp     7
#define az_comp     8

#define d1  12.5   //ground to q1
#define d6  12 //gripper to wrist
#define a2 15.0    //q1 to q2
#define d4 19.2  //q2 to wrist
#define pi  3.141592653589793238462
#define half_pi 1.570796326794897
#define degtorad 0.0174532925199
#define radtodeg 57.295779513082

IK_NN::IK_NN(){
    ann = fann_create_from_file("nn/ik_float_20_20_20_20_20_20_20_20.net");
    ann_orientation = fann_create_from_file("nn/ik_float_orientation_20_20_20_20.net");
    ann_position = fann_create_from_file("nn/ik_float_position_20_20_20_20.net");
    ann_full = fann_create_from_file("nn/ik_float_delta_full_50_50.net");
    ann_full1 = fann_create_from_file("nn/ik_float_delta_full_51_50.net");

    calc = (double *)malloc(sizeof(double)*6);
    calcPos = (double *)malloc(sizeof(double)*6);
    calcOrientation = (double *)malloc(sizeof(double)*6);

	return;
}

IK_NN::~IK_NN(){
    fann_destroy(ann); fann_destroy(ann_orientation); fann_destroy(ann_position); free(ann_full);
    //free(calc); free(calcPos); free(calcOrientation);
}

void IK_NN::inverseKinematicsNNRaw(double x,double y,double z,double t[3][3],double angles[7], int flip){
    fann_type *calc;
    fann_type pos[9];
    /* scale the input to {-1,1} */
    double temp = a2 + d1 + d4 + d6;
    pos[x_comp] = x*(1.0/temp);
    pos[y_comp] = y*(1.0/temp);
    pos[z_comp] = z*(1.0/temp);
    pos[sx_comp] = t[0][1];
    pos[sy_comp] = t[1][1];
    pos[sz_comp] = t[2][1];
    pos[ax_comp] = t[0][2];
    pos[ay_comp] = t[1][2];
    pos[az_comp] = t[2][2];

    struct fann *ann = fann_create_from_file("nn/ik_float_40_40.net");

    calc = fann_run(ann, pos);
    /* scale angles back from {-1,1} to their respective range */
                                        /* range: */
    angles[1] = (calc[0] + 1)*pi/2;       /* [0,PI] */
    angles[2] = (calc[1] + 1)*pi/2;       /* [0,PI] */
    angles[3] = -calc[2]*pi/2.0;        /* [PI/2,-PI/2] */
    angles[4] = calc[3]*pi;             /* [-PI,PI] */
    angles[5] = (calc[4] + 1)*pi/2.0;   /* [0,PI] flip is hard coded to 0 for now*/
    angles[6] = calc[5]*pi;             /* [-PI,PI] */

    fann_destroy(ann);
}

void IK_NN::forwardKinematics(double *angles, double *pos){
    double q1 =  (angles[0] + 1)*half_pi;           /* [0,pi] */
    double q2 =  (angles[1] + 1)*half_pi;           /* [0,pi] */
    double q3 =  -(angles[2] + 1)*half_pi + half_pi;/* [pi/2,-pi/2] */
    double q4 =  angles[3]*pi;                      /* [-pi,pi] */
    double q5 = (angles[4] + 1)*half_pi;            /* [0,pi] */
    double q6 = angles[5]*pi;                       /* [-pi,pi] */

    double temp  = a2 + d1 + d6 + d4;

    double sx = cos(q6)*(cos(q4)*sin(q1) - cos(q1)*cos(q2 + q3)*sin(q4)) - (cos(q5)*sin(q1)*sin(q4) + cos(q1)*(cos(q2 + q3)*cos(q4)*cos(q5) - sin(q2 + q3)*sin(q5)))*sin(q6);
    double sy = cos(q1)*(-cos(q4)*cos(q6) + cos(q5)*sin(q4)*sin(q6)) - sin(q1)*(-sin(q2 + q3)*sin(q5)*sin(q6) + cos(q2 + q3)*(cos(q6)*sin(q4) + cos(q4)*sin(q5)*sin(q6)));
    double sz = -cos(q6)*sin(q2 + q3)*sin(q4) - (cos(q4)*cos(q5)*sin(q2 + q3) + cos(q2 + q3)*sin(q5))*sin(q6);
    double ax = sin(q1)*sin(q4)*sin(q5) + cos(q1)*(cos(q5)*sin(q2 + q3) + cos(q2 + q3)*cos(q4)*sin(q5));
    double ay = cos(q5)*sin(q1)*sin(q2 + q3) + (cos(q2 + q3)*cos(q4)*sin(q1) - cos(q1)*sin(q4))*sin(q5);
    double az = -cos(q2 + q3)*cos(q5) + cos(q4)*sin(q2 + q3)*sin(q5);

    pos[x_comp] = (1.0/temp)*(d6*sin(q1)*sin(q4)*sin(q5) + cos(q1)*(a2*cos(q2) + (d4 + d6*cos(q5))*sin(q2 + q3) + d6*cos(q2 + q3)*cos(q4)*sin(q5)));
    pos[y_comp] = (1.0/temp)*(cos(q3)*(d4 + d6*cos(q5))*sin(q1)*sin(q2) - d6*(cos(q4)*sin(q1)*sin(q2)*sin(q3) + cos(q1)*sin(q4))*sin(q5) + cos(q2)*sin(q1)*(a2 + (d4 + d6*cos(q5))*sin(q3) + d6*cos(q3)*cos(q4)*sin(q5)));
    pos[z_comp] = (1.0/temp)*(d1 - cos(q2 + q3)*(d4 + d6*cos(q5)) + a2*sin(q2) + d6*cos(q4)*sin(q2 + q3)*sin(q5));

    pos[sx_comp] = sx;
    pos[sy_comp] = sy;
    pos[sz_comp] = sz;
    pos[ax_comp] = ax;
    pos[ay_comp] = ay;
    pos[az_comp] = az;
}
/* anglesInternal has range (-1,1) angles is as normal, this algorithm needs the current angles of the robot as input */
void IK_NN::inverseKinematicsNNRawDelta(double x,double y,double z,double t[3][3], double anglesInternal[6] ,double angles[7]){

    double pos[15];
    double tempPos[9];
    double posError[3];
    double posInput[9];
    double orientationError[6];
    double orientationInput[12];
    double totalPosError[9];
    double totalPosInput[15];

    /* scale the input to {-1,1} */
    double temp = a2 + d1 + d4 + d6;
    pos[x_comp] = x*(1.0/temp);
    pos[y_comp] = y*(1.0/temp);
    pos[z_comp] = z*(1.0/temp);
    pos[sx_comp] = t[0][1];
    pos[sy_comp] = t[1][1];
    pos[sz_comp] = t[2][1];
    pos[ax_comp] = t[0][2];
    pos[ay_comp] = t[1][2];
    pos[az_comp] = t[2][2];

    /* run the network a few times to (hopefully) improve the results */
    for(int j=0; j<1; j++){
        memcpy(pos+9, anglesInternal, sizeof(double)*6);
        calc = fann_run(ann, pos);
        /* rescale the output to (-2,2) */
        for(int i = 0; i < 6; i++)
            anglesInternal[i] += calc[i]*2;
    }
    /* run the network on the position and orientation error */
    for(int j=0; j<10; j++){
        /* update angles using the position error */
        forwardKinematics(anglesInternal, tempPos);
        for(int i=0; i<3; i++)
            posError[i] = pos[i] - tempPos[i];
        memcpy(posInput,posError,sizeof(double)*3);
        memcpy(posInput + 3,anglesInternal,sizeof(double)*6);

        calcPos = fann_run(ann_position, posInput);
        for(int i = 0; i < 6; i++)
            anglesInternal[i] += calcPos[i]/4.0;

        /* update angles using the orientation error */
        forwardKinematics(anglesInternal, tempPos);

        for(int i=0; i<6; i++)
            orientationError[i] = pos[i+3] - tempPos[i+3];

        memcpy(orientationInput,orientationError,sizeof(double)*6);
        memcpy(orientationInput + 6,anglesInternal,sizeof(double)*6);

        calcOrientation = fann_run(ann_orientation, orientationInput);
        for(int i = 0; i < 6; i++)
            anglesInternal[i] += calcOrientation[i]/4.0;
    }

    for(int j=0; j<10; j++){
       /* update angles using the position + orientation error */
        forwardKinematics(anglesInternal, tempPos);

        for(int i=0; i<9; i++)
            totalPosError[i] = pos[i] - tempPos[i];

        memcpy(totalPosInput,totalPosError,sizeof(double)*9);
        memcpy(totalPosInput + 9,anglesInternal,sizeof(double)*6);

        calcTotal = fann_run(ann_full, totalPosInput);
        for(int i = 0; i < 6; i++)
            anglesInternal[i] += calcTotal[i]/20.0;
    }

    for(int j=0; j<5; j++){
       /* update angles using the position + orientation error */
        forwardKinematics(anglesInternal, tempPos);

        for(int i=0; i<9; i++)
            totalPosError[i] = pos[i] - tempPos[i];

        memcpy(totalPosInput,totalPosError,sizeof(double)*9);
        memcpy(totalPosInput + 9,anglesInternal,sizeof(double)*6);

        calcTotal = fann_run(ann_full1, totalPosInput);
        for(int i = 0; i < 6; i++)
            anglesInternal[i] += calcTotal[i]/50.0;
    }

    angles[1] = (anglesInternal[0] + 1)*half_pi;
    angles[2] = (anglesInternal[1] + 1)*half_pi;
    angles[3] = -anglesInternal[2]*half_pi;
    angles[4] = anglesInternal[3]*pi;
    angles[5] = (anglesInternal[4] + 1)*half_pi;
    angles[6] = anglesInternal[5]*pi;
}
