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
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>


using namespace cv;
using namespace std;

class cam
{
    public:
        cam(uint8_t cameraNum, int framesPerSecond);
        bool getMatrixFromFile(string name, Mat cameraMatrix, Mat distanceCoefficients);
        int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension, vector<double>& relPos1, vector<double>& relRot1, int baseMarker, int toFindMarker );
        bool calibrateRoutine(int cameraNumber, Mat cameraMatrix, Mat distanceCoefficients);
    private:
        void findRelativeVectors(int basePos, int Pos, vector<Vec3d>& translationVectors, vector<Vec3d>& rotationVectors, vector<double>& posRes, vector<double>& Rotres );
        int findIndex(vector<int>& vec, int val);
        uint8_t camera;
        int fps;
};

#endif // CAM_H
