#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include "IK.h"
#include "Serial.h"
#include "cam.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

extern condition_variable cond;
extern mutex mu,grabmu;
using namespace std;

class PathPlanning
{
    public:
        PathPlanning(char const * portName);
        void createPointsBox(int marker, double density, double*** objectPoints, int& points);
        void line(struct Pos start, struct Pos stop, double time, int flip);
    private:
        void getAttractiveForceWorld(double F_world[7][3], double angles_final[7], double angles_initial[7], int d);
        int wait();
        void commandArduino(double angles[7], int grip);
        void sendStuff(int16_t *val);

};

#endif // PATHPLANNING_H
