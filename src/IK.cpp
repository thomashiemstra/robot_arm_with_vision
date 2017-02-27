#include "IK.h"
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <time.h>

#define d1  12.5   //ground to q1
#define d6  12.5 //gripper to wrist
#define a_2 15    //q1 to q2
#define a_3 14.3  //q2 to wrist
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324


using namespace::std;

double epsilon = 0.05; /*use to avoid wrist singularities */
/* arrays for the multimap function which makes the servos a bit more linear */
double inangles[5]      =  {0,pi/4.0, pi/2.0, (3.0/4.0)*pi, pi};
double servovals[7][5] =   	{	{130,240,350,450,570}, //0
								{135,250,350,450,565}, //1
								{110,230,330,445,560}, //2
								{175,280,380,500,620}, //3
								{135,250,355,450,580}, //4
								{150,245,340,445,555}, //5
								{155,250,350,450,555} };//6

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

short IK::getServoTick(double val, int servoNumber){ /* a multimap from angles to servo signals */
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

void IK::inverseKinematics(double x,double y,double z,double t[3][3],double angles[7]) /*results are stored in angles array */
{
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
    D = ( pow(xc,2) + pow(yc,2) + pow((zc-d1),2) - pow(a_2,2) - pow(a_3,2) )/(2*a_2*a_3);
    angles[1] = atan2l(yc,xc);
    angles[3] = atan2l( -sqrt( 1 - pow(D,2) ),D );
    k1 = a_2+a_3*cosl(angles[3]);
    k2 = a_3*sinl(angles[3]);
    angles[2] = atan2l( (zc-d1), sqrt(pow(xc,2) + pow(yc,2)) ) - atan2l(k2,k1) ;
    /* the DH frame is rotated 90 degrees compared to the calculated value see 2.9.7 from the book*/
    angles[3] = angles[3] + pi/2;

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

    angles[4] = atan2l(-ay,-ax);
    angles[5] = atan2l(-sqrt(ax*ax+ay*ay),az);
    angles[6] = atan2l(-sz,nz);

    /* all that follows now is fixing the angles because some of the servo orientations */
    /* do no align with the DH frames and servo's can only move 180 degrees*/
    if (angles[4] > pi/2){
		angles[4] -= pi;
		angles[5] = -angles[5];
		angles[6] += pi ;
    }
    if (angles[4] < -pi/2){
		angles[4] += pi;
		angles[5] = -angles[5];
		angles[6] -= pi ;
    }
	if( angles[6] < -pi)
		angles[6] += 2*pi;
	if( angles[6] > pi)
		angles[6] -= 2*pi;

	angles[1] = angles[1];
    angles[2] = angles[2];
    angles[3] = -(angles[3] - pi/2);
    angles[4] = (angles[4] + pi/2);
    angles[5] = (pi/2.0) - angles[5];
	angles[6] = (pi/2.0) + 0.5*angles[6]; /* 0.5 because of the 2:1 gearing on the last joint*/

}
