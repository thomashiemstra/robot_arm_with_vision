#ifndef IK_NN_H
#define IK_NN_H

using namespace::std;

class IK_NN
{
    public:
    IK_NN();
    ~IK_NN();
    void inverseKinematicsRaw(double x,double y,double z,double t[3][3],double angles[7], int flip);
    void inverseKinematicsNNRaw(double x,double y,double z,double t[3][3],double angles[7], int flip);
    void inverseKinematicsNNRawDelta(double x,double y,double z,double t[3][3], double anglesInternal[6] ,double angles[7]);

    private:
    void forwardKinematics(double *angles, double *pos);

    struct fann *ann;
    struct fann *ann_orientation;
    struct fann *ann_position;
    struct fann *ann_full;
    struct fann *ann_full1;
    double *calc;
    double *calcTotal;
    double *calcPos;
    double *calcOrientation;

};

#endif // IK_NN_H
