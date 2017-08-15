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
#define r_comp      0
#define theta_comp  1
#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.01745329251994329576923690768488612713
#define radtodeg 57.2957795130823208767981548141051703324

double repD = 4;
double attD = 2;
/* parameter for the attractive force*/
double c[7] = {0,0,2,2,6,2,4};
/* parameter for the repulsive force*/
double n[7] = {0,0,5,5,10,10,10};
/* control points are modeled as spheres (every object is approximately a sphere, even you) */
double controlPointSize[7] = {0,0,5,5,7,4,4};
/* maximum size of the steps in radians */
double alpha = 0.001;
int delay = 2;
double offset = 2; /* extra distance in cm by which to inflate the object*/
double scaling = 1.2;
double density = 5; /* points per cm for the objects*/
const float arucoSquareDimension = 0.0265f; //in meters

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

void PathPlanning::rotTrans(int marker, vector<vector<vector<double > > >& objectPoints, double theta, vector<double>& trans){
    int i;
    double x,y;
    double c = cos(theta);
    double s = sin(theta);
    int points = objectPoints[marker].size();

    for(i=0; i<points; i++){
        x = objectPoints[marker][i][x_comp] - 1.5; /* the origin fame of the marker is not at a corner but shifted by about 1.5 cm (center of the marker)*/
        y = objectPoints[marker][i][y_comp] - 1.5;

        objectPoints[marker][i][x_comp] = (c*x - s*y) + trans[x_comp];
        objectPoints[marker][i][y_comp] = (s*x + c*y) + trans[y_comp];
    }
}
/* f(0) = scaling - 1 f(z_max) = 0, makes the side of the box at a slight angle so the robot arm gets pushed up (I hope)*/
double PathPlanning::factor(double z, double z_max, double scaling){
    double a = (1.0 - scaling)/z_max;
    double b = scaling - 1;
    return a*z+b;
}
/* represent the cylinder as a bunch of points on it's surface */
void PathPlanning::createPointsCylinder(int marker, double dims[3], vector<vector<vector<double > > >& objectPoints){
    int i,j;
    dims[r_comp] += offset/2;
    dims[z_comp] += offset/2;
    int points_theta = ceil(2*pi*dims[r_comp]*density);
    int points_z = dims[z_comp]*density;
    double dl = 1.0/density;
    vector<double > pos(4);
    /* the centre point of the cylinder and the distance from the centre to the corners is stored at index 0*/
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = dims[z_comp]/2;
    pos[3] = sqrt(pow(pos[2],2) + pow(dims[r_comp],2)); /* radius of the sphere encapsulating the cylinder*/
    objectPoints[marker].push_back(pos);
    pos.resize(3); /* no need shoving in a 4th component anymore */
    double temp; /* amount of distance to add in the r direction to make the sides of the cylinder are at an angle so the arm get's pushed up*/
    /* top */
    for(j=0; j<points_theta; j++){
        double theta = ((double)j/points_theta)*2*pi;
        pos[z_comp] = dims[z_comp];
        pos[x_comp] = dims[r_comp]*cos(theta);
        pos[y_comp] = dims[r_comp]*sin(theta);
        objectPoints[marker].push_back(pos);
    }
    /* sides */
    for(i=0; i<points_z; i++){
        for(j=0; j<points_theta; j++){
            double theta = ((double)j/points_theta)*2*pi;
            pos[z_comp] = dl*i;
            temp = dims[z_comp]*factor(pos[z_comp],dims[z_comp] ,scaling);
            double r = dims[r_comp] + temp;

            pos[x_comp] = r*cos(theta);
            pos[y_comp] = r*sin(theta);
            objectPoints[marker].push_back(pos);
        }
    }
}
/* represent the box as a bunch of points on it's surface */
void PathPlanning::createPointsBox(int marker, double dims[3], vector<vector<vector<double > > >& objectPoints){
    int i,j,k;
    dims[x_comp] += offset;
    dims[y_comp] += offset;
    dims[z_comp] += offset/2.0;
    int points_x = dims[x_comp]*density;
    int points_y = dims[y_comp]*density;
    int points_z = dims[z_comp]*density;
    double dl = 1.0/density;
    double temp; /* amount of distance to add in x or y direction to make the sides of the box be at an angle so the arm get's pushed up*/
    vector<double > pos(4);

    /* the centre point of the box and the distance from the centre to the corners is stored at index 0*/
    pos[0] = (dims[x_comp] - offset/2.0)/2.0;
    pos[1] = (dims[y_comp] - offset/2.0)/2.0;
    pos[2] =  dims[z_comp]/2;
    pos[3] = sqrt(pow(pos[0],2) + pow(pos[1],2) + pow(pos[2],2)); /* radius of the sphere encapsulating the box*/
    objectPoints[marker].push_back(pos);

    pos.resize(3); /* no need shoving in a 4th component anymore */
    /* top */
    for(i=0; i<= points_x; i++){
        for(j=0; j<= points_y; j++){
            pos[0] = i*dl - offset/2.0; /* x */
            pos[1] = j*dl - offset/2.0; /* y */
            pos[2] = dims[z_comp];      /* z */
            objectPoints[marker].push_back(pos);
        }
    }
    /* back and front */
    for(i=0; i<= points_x; i++){
        for(j=0; j< points_z; j++){
            for(k=0; k < 2; k++ ){
                pos[2] = j*dl;  /* z */
                temp = dims[z_comp]*factor(pos[2],dims[z_comp] ,scaling); /* should go from scaling to 1*/

                pos[0] = i*dl - offset/2.0; /* x */
                if (pos[0]<=dims[x_comp]/2.0)
                    pos[0] -= temp;
                else if(pos[0]>dims[x_comp]/2.0)
                    pos[0] += temp;

                pos[1] = k*dims[y_comp] - offset/2.0; /* y */
                if(pos[1]<=dims[y_comp]/2.0)
                    pos[1] -= temp;
                else if(pos[1]>dims[y_comp]/2.0)
                    pos[1] += temp;

                objectPoints[marker].push_back(pos);
            }
        }
    }
    /* sides */
    for(i=1; i< points_y; i++){
        for(j=0; j< points_z; j++){
            for(k=0; k < 2; k++ ){
                pos[2] = j*dl;
                temp = dims[z_comp]*factor(pos[2],dims[z_comp] ,scaling);

                pos[1] = i*dl - offset/2.0; /* y */
                if(pos[1]<=dims[y_comp]/2.0)
                    pos[1] -= temp;
                else if(pos[1]>dims[y_comp]/2.0)
                    pos[1] += temp;

                pos[0] = k*dims[x_comp] - offset/2.0; /* x */
                if (pos[0]<=dims[x_comp]/2.0)
                    pos[0] -=temp;
                else if(pos[0]>dims[x_comp]/2.0)
                    pos[0] +=temp;

                objectPoints[marker].push_back(pos);
            }

        }
    }
}
/* d is cutoff distance beyond which the F_world = 0*/
void PathPlanning::getRepulsiveForceWorld(double F_world[7][3], double angles_current[7], int marker, vector<vector<vector<double > > >& objectPoints){
    int point;
    double currentPos[7][3];
    int points = objectPoints[marker].size();
    ik.forwardKinematics(angles_current, currentPos);

    double res = repD;
    for(int i=2; i<7; i ++){
        bool calc = false;
        for(int j=0; j<points-1; j++){
            double temp = sqrt(pow(currentPos[i][x_comp] - objectPoints[marker][j][x_comp], 2) + pow(currentPos[i][y_comp] - objectPoints[marker][j][y_comp], 2) + pow(currentPos[i][z_comp] - objectPoints[marker][j][z_comp], 2));
            temp -= controlPointSize[i];

            if(j==0 && temp > objectPoints[marker][j][3]){
                cout << "break" << endl;
                break; /* controlpoint outside the sphere encapsulating the object so no need to check any further points*/
            }

            if(temp < res){
                res = temp;
                point = j;
                calc = true;
            }
        }
        //cout << calc << endl;
        if(calc){
            double absD = sqrt( pow(currentPos[i][x_comp] - objectPoints[marker][point][x_comp],2) +  pow(currentPos[i][y_comp] - objectPoints[marker][point][y_comp],2) + pow(currentPos[i][z_comp] - objectPoints[marker][point][z_comp],2));
            F_world[i][x_comp] = n[i]*((1.0/res) - 1.0/repD)*(1.0/pow(res,2))*(currentPos[i][x_comp] - objectPoints[marker][point][x_comp])/absD;
            F_world[i][y_comp] = n[i]*((1.0/res) - 1.0/repD)*(1.0/pow(res,2))*(currentPos[i][y_comp] - objectPoints[marker][point][y_comp])/absD;
            F_world[i][z_comp] = n[i]*((1.0/res) - 1.0/repD)*(1.0/pow(res,2))*(currentPos[i][z_comp] - objectPoints[marker][point][z_comp])/absD;
        }
        else
            F_world[i][x_comp] =  F_world[i][y_comp] =  F_world[i][z_comp] = 0;
        res = repD;
    }

}
/* needs raw angles, d is the cutoff distance between linear and quadratic potential */
void PathPlanning::getAttractiveForceWorld(double F_world[7][3], double angles_final[7], double angles_current[7]){
    double currentPos[7][3];
    double goalPos[7][3];
    double distance[7];
    double d = attD;

    ik.forwardKinematics(angles_final, goalPos);
    ik.forwardKinematics(angles_current, currentPos);
    /* only joint 2, 4 and 6 are being evaluated since 1 does nothing, 2=3 and 4=5*/
    for(int i=2; i<7; i ++){
        distance[i] = sqrt(pow(currentPos[i][x_comp] - goalPos[i][x_comp],2) + pow(currentPos[i][y_comp] - goalPos[i][y_comp],2) + pow(currentPos[i][z_comp] - goalPos[i][z_comp],2));
        if(distance[i] > d ){
            F_world[i][x_comp] = -d*c[i]*(currentPos[i][x_comp] - goalPos[i][x_comp])/distance[i];
            F_world[i][y_comp] = -d*c[i]*(currentPos[i][y_comp] - goalPos[i][y_comp])/distance[i];
            F_world[i][z_comp] = -d*c[i]*(currentPos[i][z_comp] - goalPos[i][z_comp])/distance[i];
        }
        else if(distance[i] < d){
            F_world[i][x_comp] = -c[i]*(currentPos[i][x_comp] - goalPos[i][x_comp]);
            F_world[i][y_comp] = -c[i]*(currentPos[i][y_comp] - goalPos[i][y_comp]);
            F_world[i][z_comp] = -c[i]*(currentPos[i][z_comp] - goalPos[i][z_comp]);
        }
    }
}

void PathPlanning::createPoints(int marker, vector<vector<vector<double > > >& objectPoints){
    double dims[3];
    switch(marker){
    case 10:
        dims[x_comp] = 7.5; dims[y_comp] = 20.5; dims[z_comp] = 21;
        createPointsBox(marker,dims,objectPoints);
        break;
    case 11:
        dims[x_comp] = 7.5; dims[y_comp] = 23; dims[z_comp] = 21;
        createPointsBox(marker,dims,objectPoints);
        break;
    case 12:
        dims[x_comp] = 20; dims[y_comp] = 20.5; dims[z_comp] = 7.5;
        createPointsBox(marker,dims,objectPoints);
        break;
    case 14:
        dims[r_comp] = 4.5; dims[z_comp] = 33;
        createPointsCylinder(marker,dims,objectPoints);
        break;
    case 15:
        dims[r_comp] = 10.5; dims[z_comp] = 7;
        createPointsCylinder(marker,dims,objectPoints);
        break;
    }
}

void PathPlanning::lineOO(struct Pos start, struct Pos stop, int flip){
    Mat cameraMatrix = Mat::eye(3,3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(5,1, CV_64F);
    CAM.getMatrixFromFile("CameraCalibration720.dat", cameraMatrix, distanceCoefficients);

    vector<vector<vector<double > > > objectPoints;
    vector<int> objectMarkers = {10,11,12,13,14,15,16,17,18,19}; /* all the possible markers of obstacles */
    vector<int > foundMarkers;
    int totalMarkers = 50; /* the marker library has 50 distinct markers, so let's make some room first */
    objectPoints.resize(totalMarkers);

    vector<vector<double > > relPos;
    relPos.resize(totalMarkers);
    for(int i=0; i<totalMarkers; i++)
        relPos[i].resize(3);

    vector<Mat > relativeMatrix;
    relativeMatrix.resize(totalMarkers);
    for(int i=0; i<totalMarkers; i++)
        relativeMatrix[i].push_back(Mat::zeros(3,3, CV_64F));

    /* find the obstacles */
    CAM.findVecsCharuco(cameraMatrix, distanceCoefficients, arucoSquareDimension,relPos,relativeMatrix,objectMarkers,foundMarkers);
    for(unsigned int k=0; k<foundMarkers.size(); k++){
        int marker  = foundMarkers[k];
        double theta = atan2(relativeMatrix[marker].at<double>(1,0),relativeMatrix[marker].at<double>(0,0));
        /* convert x and y to centimeters */
        relPos[marker][0] *= 100; relPos[marker][1]  = 100*relPos[marker][1] + 10;
        createPoints(marker,objectPoints);
        rotTrans(marker, objectPoints, theta, relPos[marker]);
        cout << "marker:" << marker << endl;
        cout << "x=" << relPos[marker][0] << "\ty=" << relPos[marker][1] << endl;
        cout << "x_c=" << objectPoints[marker][0][0] << "    y_c=" <<  objectPoints[marker][0][1] << endl;
    }
    if(foundMarkers.size() == 0){
        cout << "no obstacles" << endl;
        tricks.setArmPos(start, flip);
        tricks.wait();
        tricks.pointToPoint(start, stop, 2, 0);
        return;
    }
    cout << "done looking" << endl;
    tricks.wait();
    line(start,stop,delay,flip,objectPoints,foundMarkers);
}

void PathPlanning::line(struct Pos start, struct Pos stop, int time, int flip, vector<vector<vector<double > > >& objectPoints, vector<int > foundMarkers){
    int i;
    unsigned int ui;
    double stopAngles[7] = {0};
    double currentAngles[7] = {0};
    double F_world[7][3];
    double F_joints[7];
    double currentPos[7][3];
    double goalPos[7][3];
    double absF;
    double finishedCondition;
    bool done = false;
    double servoAngles[7];
    int counter = 0;

    ik.eulerMatrix(start.alpha, start.beta, start.gamma,s);
    ik.inverseKinematicsRaw(start.x, start.y, start.z, s,currentAngles, flip);
    ik.eulerMatrix(stop.alpha, stop.beta, stop.gamma,s);
    ik.inverseKinematicsRaw(stop.x, stop.y, stop.z, s,stopAngles, flip);
    /* just to be safe*/
    for(i=0; i<7; i++)
        F_joints[i] = 0;
    /* get initial force*/
    getAttractiveForceWorld(F_world, stopAngles, currentAngles);
    ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);

    for(ui = 0; ui<foundMarkers.size(); ui++){
        getRepulsiveForceWorld(F_world, currentAngles, foundMarkers[ui], objectPoints);
        ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);
    }
    ik.convertAngles(currentAngles,servoAngles);
    tricks.commandArduino(servoAngles,10);

    while(!done){
        auto begin = std::chrono::high_resolution_clock::now();
        finishedCondition = 0;

        for(i=0; i<7; i++)
            F_joints[i] = 0;

        getAttractiveForceWorld(F_world,stopAngles, currentAngles);
        ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);
        for(ui = 0; ui<foundMarkers.size(); ui++){
            getRepulsiveForceWorld(F_world, currentAngles, foundMarkers[ui], objectPoints);
            ik.jacobianTransposeOnF(F_world, F_joints, currentAngles);
        }
        ik.forwardKinematics(stopAngles, goalPos);
        ik.forwardKinematics(currentAngles, currentPos);

        for(i=2; i<7; i+=2)
            finishedCondition += pow(currentPos[i][x_comp] - goalPos[i][x_comp],2) + pow(currentPos[i][y_comp] - goalPos[i][y_comp],2) + pow(currentPos[i][z_comp] - goalPos[i][z_comp],2);
        finishedCondition = sqrt(finishedCondition);
        if(finishedCondition < 1.5)
            done = true;

        auto temp = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = temp - begin;
        double test = fp_ms.count(); /* time elapsed so far*/
        int hold = (int)(time - test); /* wait at least 10 ms seconds else the arduino get's confused*/
        if(hold<0)
            hold = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(hold));

        double absFres = 0;

        for(i=1; i<7; i++)
            absFres += pow(F_joints[i],2);
        absF = sqrt(absFres);

        for(i=1; i<7; i++)
            currentAngles[i] += alpha*F_joints[i]/absF;

        if(counter > 5){
            ik.convertAngles(currentAngles,servoAngles);
            tricks.commandArduino(servoAngles,10);
            counter = 0;
        }
        counter++;
    }
    double finalAngles[7];
    ik.convertAngles(stopAngles,finalAngles);
    tricks.anglesToAngles(servoAngles,finalAngles,1,flip,stop.grip);
    cout << "done" << endl;
}
