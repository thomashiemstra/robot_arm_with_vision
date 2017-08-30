#include "IK.h"
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
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324

using namespace::std;

double epsilon = 0.05; /*use to avoid wrist singularities */
/* arrays for the multimap function which makes the servos a bit more linear */
double inangles[5]      =  {0,pi/4.0, pi/2.0, (3.0/4.0)*pi, pi};
double servovals[7][5] =   	{	{225,350,480,610,750}, //0
								{250,360,490,630,765}, //1
								{245,365,490,630,766}, //2
								{270,380,510,660,795}, //3
								{240,400,585,775,930}, //4
								{190,370,565,780,970}, //5 goes from 0 to 270 aka -135 to +135
								{245,390,545,715,875} };//6

IK::IK(void){
	return;
}
/* roll pitch yaw matrix with the columns permutated z->y, y->x x->z */
/* so that by default the gripper is in the y_0 direction and the  slider is in the x_0 direction */
void IK::eulerMatrix(double alpha, double beta, double gamma, double m[3][3]){
    double ca,cb,cy,sa,sb,sy;
    ca = cos(alpha); cb = cos(beta); cy = cos(gamma);
    sa = sin(alpha); sb = sin(beta); sy = sin(gamma);
    m[0][0] = ca*sb*cy + sa*sy;
    m[1][0] = sa*sb*cy - ca*sy;
    m[2][0] = cb*cy;

    m[0][1] = ca*cb;
    m[1][1] = sa*cb;
    m[2][1] = -sb;

    m[0][2] = ca*sb*sy - sa*cy;
    m[1][2] = sa*sb*sy + ca*cy;
    m[2][2] = cb*sy;
}

short IK::getServoTick(double val, int servoNumber){ /* a multimap from angle to servo signal */
  int size = 5;
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= inangles[0]) return servovals[servoNumber][0];
  if (val >= inangles[size-1]) return servovals[servoNumber][size-1];
  // search right interval
  int pos = 1;  // _in[0] allready tested
  while(val > inangles[pos]) pos++;
  // this will handle all exact "points" in the _in array
  if (val == inangles[pos]) return servovals[servoNumber][pos];
  // interpolate in the right segment for the rest
  return round((val - inangles[pos-1]) * (servovals[servoNumber][pos] - servovals[servoNumber][pos-1]) / (inangles[pos] - inangles[pos-1]) + servovals[servoNumber][pos-1]);
}

void IK::inverseKinematics(double x,double y,double z,double t[3][3],double angles[7], int flip){ /*results are stored in the angles array */
    /* the method here follows the calculations from the book:
    Modelling and Control of Robot Manipulators, Lorenzo Sciavicco and Bruno Siciliano chapter 2 (specifically 2.12.2)*/
    double xc,yc,zc;
    double ay,ax,az,sz,nz;
    double r11,r12,r13,r21,r22,r23,r31,r32,r33;
    double q1,q2,q3;
    double D,k1,k2;
    /* compute the wrist position */
    xc = x - d6*t[0][2];
    yc = y - d6*t[1][2];
    zc = z - d6*t[2][2];
    /* solve inverse kinematics for the first 3 angles found in: */
    /* http://www.hessmer.org/uploads/RobotArm/Inverse%2520Kinematics%2520for%2520Robot%2520Arm.pdf */
    angles[1] = atan2l(yc,xc);

    D = ( pow(xc,2) + pow(yc,2) + pow((zc-d1),2) - pow(a2,2) - pow(d4,2) )/(2*a2*d4);
    angles[3] = atan2l(-sqrt(1 - pow(D,2)),D );

    k1 = a2+d4*cosl(angles[3]);
    k2 = d4*sinl(angles[3]);
    angles[2] = atan2l( (zc-d1), sqrt(pow(xc,2) + pow(yc,2)) ) - atan2l(k2,k1) ;
    /* the DH frame is rotated 90 degrees compared to the calculated value see 2.9.7 from the book*/
    angles[3] += pi/2;
    /* for my own sanity */
    q1=angles[1]; q2=angles[2]; q3=angles[3];
    r11=t[0][0];r12=t[0][1];r13=t[0][2]; r21=t[1][0];r22=t[1][1];r23=t[1][2]; r31=t[2][0];r32=t[2][1];r33=t[2][2];
    /* solve inverse kinematics for the final 3 angles (2.12.5 from the book) */
    ax = r13*cosl(q1)*cosl(q2 + q3) + r23*cosl(q2 + q3)*sinl(q1) + r33*sinl(q2 + q3);
    ay = -r23*cosl(q1) + r13*sinl(q1);
    az = -r33*cosl(q2 + q3) + r13*cosl(q1)*sinl(q2 + q3) + r23*sinl(q1)*sinl(q2 + q3);
    sz = -r32*cosl(q2 + q3) + r12*cosl(q1)*sinl(q2 + q3) + r22*sinl(q1)*sinl(q2 + q3);
    nz = -r31*cosl(q2 + q3) + r11*cosl(q1)*sinl(q2 + q3) + r21*sinl(q1)*sinl(q2 + q3);
    /* getting the angles from the matrix only works if ax != 0 and ay != 0 */
    if(ax == 0)
        ax += epsilon;
    if(ay == 0)
        ay += epsilon;

	if(flip){
		angles[4] = atan2(-ay,-ax);
		angles[5] = atan2(-sqrt(ax*ax+ay*ay),az);
		angles[6] = atan2(-sz,nz);
	}
	else{
		angles[4] = atan2(ay,ax);
		angles[5] = atan2(sqrt(ax*ax+ay*ay),az);
		angles[6] = atan2(sz,-nz);
	}
    /* all that follows now is fixing the angles because some of the servo orientations */
    /* do no align with the DH frames and servo's can only move 180 degrees*/
	angles[1] = angles[1];
    angles[2] = angles[2];
    angles[3] = -(angles[3] - pi/2);
    angles[4] = (0.5*angles[4] + pi/2); /* 2:1 gear ration for this one*/
    angles[5] = (pi/2.0) - (2.0/3.0)*angles[5] ; /* a 270 degree servo goes from 135 to -135 degrees */
	angles[6] = (pi/2.0) + 0.5*angles[6]; /* again 2:1*/
}
/* don't adjust angles for motors*/
void IK::inverseKinematicsRaw(double x,double y,double z,double t[3][3],double angles[7], int flip){ /*results are stored in the angles array */
    /* the method here follows the calculations from the book:
    Modelling and Control of Robot Manipulators, Lorenzo Sciavicco and Bruno Siciliano chapter 2 (specifically 2.12.2)*/
    double xc,yc,zc;
    double ay,ax,az,sz,nz;
    double r11,r12,r13,r21,r22,r23,r31,r32,r33;
    double q1,q2,q3;
    double D,k1,k2;
    /* compute the wrist position */
    xc = x - d6*t[0][2];
    yc = y - d6*t[1][2];
    zc = z - d6*t[2][2];
    /* solve inverse kinematics for the first 3 angles found in: */
    /* http://www.hessmer.org/uploads/RobotArm/Inverse%2520Kinematics%2520for%2520Robot%2520Arm.pdf */
    angles[1] = atan2l(yc,xc);

    D = ( pow(xc,2) + pow(yc,2) + pow((zc-d1),2) - pow(a2,2) - pow(d4,2) )/(2*a2*d4);
    angles[3] = atan2l(-sqrt(1 - pow(D,2)),D );

    k1 = a2+d4*cosl(angles[3]);
    k2 = d4*sinl(angles[3]);
    angles[2] = atan2l( (zc-d1), sqrt(pow(xc,2) + pow(yc,2)) ) - atan2l(k2,k1) ;
    /* the DH frame is rotated 90 degrees compared to the calculated value see 2.9.7 from the book*/
    angles[3] += pi/2;
    /* for my own sanity */
    q1=angles[1]; q2=angles[2]; q3=angles[3];
    r11=t[0][0];r12=t[0][1];r13=t[0][2]; r21=t[1][0];r22=t[1][1];r23=t[1][2]; r31=t[2][0];r32=t[2][1];r33=t[2][2];
    /* solve inverse kinematics for the final 3 angles (2.12.5 from the book) */
    ax = r13*cosl(q1)*cosl(q2 + q3) + r23*cosl(q2 + q3)*sinl(q1) + r33*sinl(q2 + q3);
    ay = -r23*cosl(q1) + r13*sinl(q1);
    az = -r33*cosl(q2 + q3) + r13*cosl(q1)*sinl(q2 + q3) + r23*sinl(q1)*sinl(q2 + q3);
    sz = -r32*cosl(q2 + q3) + r12*cosl(q1)*sinl(q2 + q3) + r22*sinl(q1)*sinl(q2 + q3);
    nz = -r31*cosl(q2 + q3) + r11*cosl(q1)*sinl(q2 + q3) + r21*sinl(q1)*sinl(q2 + q3);
    /* getting the angles from the matrix only works if ax != 0 and ay != 0 */
    if(ax == 0)
        ax += epsilon;
    if(ay == 0)
        ay += epsilon;

	if(flip){
		angles[4] = atan2(-ay,-ax);
		angles[5] = atan2(-sqrt(ax*ax+ay*ay),az);
		angles[6] = atan2(-sz,nz);
		angles[0] = angles[5];
	}
	else{
		angles[4] = atan2(ay,ax);
		angles[5] = atan2(sqrt(ax*ax+ay*ay),az);
		angles[6] = atan2(sz,-nz);
		angles[0] = angles[5];
	}
}

void IK::inverseKinematicsNNRaw(double x,double y,double z,double t[3][3],double angles[7], int flip){
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

    struct fann *ann = fann_create_from_file("ik_double_30_30.net");;

    calc = fann_run(ann, pos);
    /* scale angles back from {-1,1} to their respective range */
                                        /* range: */
    angles[1] = (calc[0] + 1)*pi/2;       /* [0,2PI] */
    angles[2] = (calc[1] + 1)*pi/2;       /* [0,2PI] */
    angles[3] = -calc[2]*pi/2.0;        /* [PI/2,-PI/2] */
    angles[4] = calc[3]*pi;             /* [-PI,PI] */
    angles[5] = (calc[4] + 1)*pi/2.0;   /* [0,PI] flip is hard coded to 0 for now*/
    angles[6] = calc[5]*pi;             /* [-PI,PI] */
}


void IK::convertAngles(double inangles[7], double outangles[7]){
    /* all that follows now is fixing the angles because some of the servo orientations */
    /* do no align with the DH frames and servo's can only move 180 degrees*/
	outangles[1] = inangles[1];
    outangles[2] = inangles[2];
    outangles[3] = -(inangles[3] - pi/2);
    outangles[4] = (0.5*inangles[4] + pi/2); /* 2:1 gear ration for this one*/
    outangles[5] = (pi/2.0) - (2.0/3.0)*inangles[5] ; /* a 270 degree servo goes from 135 to -135 degrees */
	outangles[6] = (pi/2.0) + 0.5*inangles[6]; /* again 2:1*/
}
/* only gives the position of all the joins not rotation, "joints"  3 and 5 are center of masses */
void IK::forwardKinematics(double angles[7], double jointPos[7][3]){
    double q1,q2,q3,q4,q5;
    q1=angles[1]; q2=angles[2]; q3=angles[3]; q4=angles[4]; q5=angles[5];

    jointPos[1][x_comp] = 0;
    jointPos[1][y_comp] = 0;
    jointPos[1][z_comp] = d1;

    jointPos[2][x_comp]  = a2*cos(q1)*cos(q2);
    jointPos[2][y_comp]  = a2*cos(q2)*sin(q1);
    jointPos[2][z_comp]  = d1+a2*sin(q2);

    jointPos[3][x_comp] = cos(q1)*(a2*cos(q2) + (d4/2.0)*sin(q2+q3));
    jointPos[3][y_comp] = sin(q1)*(a2*cos(q2) + (d4/2.0)*sin(q2+q3));
    jointPos[3][z_comp] = d1 - (d4/2.0)*cos(q2+q3)+a2*sin(q2);

    jointPos[4][x_comp] =  cos(q1)*(a2*cos(q2) + d4*sin(q2+q3));
    jointPos[4][y_comp] =  sin(q1)*(a2*cos(q2) + d4*sin(q2+q3));
    jointPos[4][z_comp] =  d1 - d4*cos(q2+q3) + a2*sin(q2);

    jointPos[5][x_comp] = (d6/2.0)*sin(q1)*sin(q4)*sin(q5) + cos(q1)*(a2*cos(q2) + (d4 + (d6/2.0)*cos(q5))*sin(q2 + q3) + (d6/2.0)*cos(q2 + q3)*cos(q4)*sin(q5));
    jointPos[5][y_comp] = cos(q3)*(d4 + (d6/2.0)*cos(q5))*sin(q1)*sin(q2) - (d6/2.0)*(cos(q4)*sin(q1)*sin(q2)*sin(q3) + cos(q1)*sin(q4))*sin(q5) + cos(q2)*sin(q1)*(a2 + (d4 + (d6/2.0)*cos(q5))*sin(q3) + (d6/2.0)*cos(q3)*cos(q4)*sin(q5));
    jointPos[5][z_comp] = d1 - cos(q2 + q3)*(d4 + (d6/2.0)*cos(q5)) + a2*sin(q2) + (d6/2.0)*cos(q4)*sin(q2 + q3)*sin(q5);

    jointPos[6][x_comp] = d6*sin(q1)*sin(q4)*sin(q5) + cos(q1)*(a2*cos(q2) + (d4 + d6*cos(q5))*sin(q2 + q3) + d6*cos(q2 + q3)*cos(q4)*sin(q5));
    jointPos[6][y_comp] = cos(q3)*(d4 + d6*cos(q5))*sin(q1)*sin(q2) - d6*(cos(q4)*sin(q1)*sin(q2)*sin(q3) + cos(q1)*sin(q4))*sin(q5) + cos(q2)*sin(q1)*(a2 + (d4 + d6*cos(q5))*sin(q3) + d6*cos(q3)*cos(q4)*sin(q5));
    jointPos[6][z_comp] =  d1 - cos(q2 + q3)*(d4 + d6*cos(q5)) + a2*sin(q2) + d6*cos(q4)*sin(q2 + q3)*sin(q5);
}

/* this function will be called in a loop going over all the world forces on all the joints */
void IK::jacobianTransposeOnF(double F_world[7][3], double F_joint[7], double angles[7]){
    double q1,q2,q3,q4,q5,fx,fy,fz;
    q1=angles[1]; q2=angles[2]; q3=angles[3]; q4=angles[4]; q5=angles[5];
    /* joint 2 origin of DH frame*/
    fx = F_world[2][x_comp]; fy = F_world[2][y_comp]; fz = F_world[2][z_comp];
    F_joint[1] += a2*cos(q2)*(fy*cos(q1) - fx*sin(q1));
    F_joint[2] += a2*(fz*cos(q1) - (fx*cos(q1) + fy*sin(q1))*sin(q2));
    /* "joint" 3 is the middle between q2 and wrist */
    fx = F_world[3][x_comp]; fy = F_world[3][y_comp]; fz = F_world[3][z_comp];
    F_joint[1] += (fy*cos(q1) - fx*sin(q1))*(a2*cos(q2) + (d4/2.0)*sin(q2 + q3));
    F_joint[2] += a2*fz*cos(q2) + (fx*cos(q2) + fy*sin(q1))*((d4/2.0)*cos(q2 + q3) - a2*sin(q2)) + (d4/2.0)*fz*sin(q2 + q3);
    F_joint[3] +=   (d4/2.0)*cos(q2 + q3)*(fx*cos(q1) + fy*sin(q1)) + (d4/2.0)*fz*sin(q2 + q3);
    /* joint 4 origin of DH frame */
    fx = F_world[4][x_comp]; fy = F_world[4][y_comp]; fz = F_world[4][z_comp];
    F_joint[1] += (fy*cos(q1) - fx*sin(q1))*(a2*cos(q2) + d4*sin(q2 + q3));
    F_joint[2] += a2*fz*cos(q2) + (fx*cos(q2) + fy*sin(q1))*(d4*cos(q2 + q3) - a2*sin(q2)) + d4*fz*sin(q2 + q3);
    F_joint[3] +=   d4*cos(q2 + q3)*(fx*cos(q1) + fy*sin(q1)) + d4*fz*sin(q2 + q3);
    /* "joint5" is the middle between the wrist and the gripper end */
    fx = F_world[5][x_comp]; fy = F_world[5][y_comp]; fz = F_world[5][z_comp];
    F_joint[1] += (fy*cos(q1) - fx*sin(q1))*(a2*cos(q2) + (d4 + (d6/2.0)*cos(q5))*sin(q2 + q3)) + (d6/2.0)*(cos(q2 + q3)*cos(q4)*(fy*cos(q1) - fx*sin(q1)) + (fx*cos(q1) + fy*sin(q1))*sin(q4))*sin(q5);
    F_joint[2] += a2*fz*cos(q2) + cos(q2 + q3)*(d4 + (d6/2.0)*cos(q5))*(fx*cos(q1) + fy*sin(q1)) + fz*(d4 + (d6/2.0)*cos(q5))*sin(q2 + q3) + (d6/2.0)*fz*cos(q2 + q3)*cos(q4)*sin(q5) - (fx*cos(q1) + fy*sin(q1))*(a2*sin(q2) + (d6/2.0)*cos(q4)*sin(q2 + q3)*sin(q5));
    F_joint[3] += (d4 + (d6/2.0)*cos(q5))*(cos(q2 + q3)*(fx*cos(q1) + fy*sin(q1)) + fz*sin(q2 + q3)) + (d6/2.0)*cos(q4)*(fz*cos(q2 + q3) - (fx*cos(q1) + fy*sin(q1))*sin(q2 + q3))*sin(q5);
    F_joint[4] += -d6*(-fx*cos(q4)*sin(q1) + (fy*cos(q2 + q3)*sin(q1) + fz*sin(q2 + q3))*sin(q4) + cos(q1)*(fy*cos(q4) + fx*cos(q2 + q3)*sin(q4)))*sin(q5);
    F_joint[5] += (d6/2.0)*cos(q5)*(cos(q4)*(cos(q2 + q3)*(fx*cos(q1) + fy*sin(q1)) + fz*sin(q2 + q3)) + (-fy*cos(q1) + fx*sin(q1))*sin(q4)) + d6*(fz*cos(q2 + q3) - (fx*cos(q1) + fy*sin(q1))*sin(q2 + q3))*sin(q5);
    /* joint 6 origin of DH frame*/
    fx = F_world[6][x_comp]; fy = F_world[6][y_comp]; fz = F_world[6][z_comp];
    F_joint[1] += (fy*cos(q1) - fx*sin(q1))*(a2*cos(q2) + (d4 + d6*cos(q5))*sin(q2 + q3)) + d6*(cos(q2 + q3)*cos(q4)*(fy*cos(q1) - fx*sin(q1)) + (fx*cos(q1) + fy*sin(q1))*sin(q4))*sin(q5);
    F_joint[2] += a2*fz*cos(q2) + cos(q2 + q3)*(d4 + d6*cos(q5))*(fx*cos(q1) + fy*sin(q1)) + fz*(d4 + d6*cos(q5))*sin(q2 + q3) + d6*fz*cos(q2 + q3)*cos(q4)*sin(q5) - (fx*cos(q1) + fy*sin(q1))*(a2*sin(q2) + d6*cos(q4)*sin(q2 + q3)*sin(q5));
    F_joint[3] += (d4 + d6*cos(q5))*(cos(q2 + q3)*(fx*cos(q1) + fy*sin(q1)) + fz*sin(q2 + q3)) + d6*cos(q4)*(fz*cos(q2 + q3) - (fx*cos(q1) + fy*sin(q1))*sin(q2 + q3))*sin(q5);
    F_joint[4] += -d6*(-fx*cos(q4)*sin(q1) + (fy*cos(q2 + q3)*sin(q1) + fz*sin(q2 + q3))*sin(q4) + cos(q1)*(fy*cos(q4) + fx*cos(q2 + q3)*sin(q4)))*sin(q5);
    F_joint[5] += d6*cos(q5)*(cos(q4)*(cos(q2 + q3)*(fx*cos(q1) + fy*sin(q1)) + fz*sin(q2 + q3)) + (-fy*cos(q1) + fx*sin(q1))*sin(q4)) + d6*(fz*cos(q2 + q3) - (fx*cos(q1) + fy*sin(q1))*sin(q2 + q3))*sin(q5);
}
