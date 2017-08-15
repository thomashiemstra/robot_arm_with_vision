#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include "IK.h"
#include "Serial.h"
#include "cam.h"
#include "tricks.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

extern condition_variable cond;
extern mutex mu,grabmu;
using namespace std;
extern Serial *arduino;
extern cam CAM;
extern IK ik;
extern Tricks tricks;

class PathPlanning
{
    public:
        PathPlanning();
        void lineOO(struct Pos start, struct Pos stop, int flip);
        void prepareOOThread(vector<vector<vector<double > > >& objectPoints, vector<int >& foundMarkers);
        void line(struct Pos start, struct Pos stop, int flip, vector<vector<vector<double > > >& objectPoints, vector<int > foundMarkers);
    private:
        void getAttractiveForceWorld(double F_world[7][3], double angles_final[7], double angles_current[7]);
        void getRepulsiveForceWorld(double F_world[7][3], double angles_current[7], int marker, vector<vector<vector<double > > >& objectPoints);
        void createPointsBox(int marker, double dims[3], vector<vector<vector<double > > >& objectPoints);
        void createPointsCylinder(int marker, double dims[3], vector<vector<vector<double > > >& objectPoints);
        void rotTrans(int marker, vector<vector<vector<double > > >& objectPoints, double theta, vector<double>& trans);
        double factor(double z, double z_max, double scaling);
        void createPoints(int marker, vector<vector<vector<double > > >& objectPoints);
};

#endif // PATHPLANNING_H
