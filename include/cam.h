#ifndef CAM_H
#define CAM_H

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <windows.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <list>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>


using namespace cv;
using namespace std;
extern condition_variable cond;
extern mutex mu;


class cam
{
    public:
        cam(uint8_t cameraNum, int framesPerSecond);
        bool getMatrixFromFile(string name, Mat cameraMatrix, Mat distanceCoefficients);
        int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension, vector<double>& relPos, Mat& relativeRotMatrix, int& toFindMarker,bool &getVecs, int& condition);
        bool calibrateRoutine(int cameraNumber, Mat cameraMatrix, Mat distanceCoefficients);
        int findVecsCharuco(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension, vector<double>& relPos1, Mat& relativeRotMatrix, int toFindMarker);
    private:
        void findRelativeVector(int basePos, int Pos, vector<Vec3d>& translationVectors, vector<Vec3d>& rotationVectors, vector<double>& posRes);
        void findRotMatrix(int basePos, int Pos, vector<Vec3d>& translationVectors, vector<Vec3d>& rotationVectors, Mat&  relativeRotMatrix);
        void findRotMatrixCharuco(Vec3d& baseRotation, Vec3d& posRotation, Mat&  relativeRotMatrix);
        int findIndex(vector<int>& vec, int val);
        void findRelativeVectorCharuco(Vec3d& baseRotation, Vec3d& baseTranslation, Vec3d& posTranslation,  vector<double>& posRes);
        uint8_t camera;
        int fps;
};

#endif // CAM_H
