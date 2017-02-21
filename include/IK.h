/*
	Inverse kinematics for a 6 DOF robto arm
*/

#ifndef IK_h
#define IK_h

class IK
{
	public:
		IK(void);
		void inverseKinematics(double x,double y,double z,double t[3][3],double angles[7]);
        short getServoTick(double val, int servoNumber);
        void eulerMatrix(double alpha, double beta, double gamma, double m[3][3]);
	private:

};

#endif
