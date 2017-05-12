/*
	Inverse kinematics for a 6 DOF robot arm
*/

#ifndef IK_h
#define IK_h
#include<vector>

using namespace::std;

class IK
{
	public:
		IK(void);
		void inverseKinematics(double x,double y,double z,double t[3][3],double angles[7], int flip);
		void inverseKinematicsRaw(double x,double y,double z,double t[3][3],double angles[7], int flip);
        short getServoTick(double val, int servoNumber);
        void eulerMatrix(double alpha, double beta, double gamma, double m[3][3]);
        void convertAngles(double angles[7]);
	private:

};

#endif
