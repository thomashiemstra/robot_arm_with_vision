#include "cam.h"

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

cam::cam(uint8_t cameraNum, int framesPerSecond){
    camera = cameraNum;
    fps = framesPerSecond;
}

int cam::findIndex(vector<int>& vec, int val){
    int res;
    uint16_t length = vec.size();
    res = find(vec.begin(), vec.end(), val) - vec.begin();
        if (res >= length){
            res = -1;
        }
    return res;
}

void cam::findRotMatrix(int basePos, int Pos, vector<Vec3d>& translationVectors, vector<Vec3d>& rotationVectors, Mat&  relativeRotMatrix){
    int i,j,k;
    Mat baseRotMatrix = Mat::zeros(3,3, CV_64F);
    Mat baseRotMatrixTranspose = Mat::zeros(3,3, CV_64F);
    Mat objectRotMatrix = Mat::zeros(3,3, CV_64F);

    Rodrigues(rotationVectors[basePos],baseRotMatrix);
    Rodrigues(rotationVectors[Pos],objectRotMatrix);

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            relativeRotMatrix.at<double>(i,j) = 0;
            baseRotMatrixTranspose.at<double>(j,i) = baseRotMatrix.at<double>(i,j);
        }
    }

    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            for(k = 0; k < 3; k++)
            {
                relativeRotMatrix.at<double>(i,j) += baseRotMatrixTranspose.at<double>(i,k) * objectRotMatrix.at<double>(k,j);
            }
        }
    }

}

void cam::findRotMatrixCharuco(Vec3d& baseRotation, Vec3d& posRotation, Mat&  relativeRotMatrix){
    int i,j,k;
    Mat baseRotMatrix = Mat::zeros(3,3, CV_64F);
    Mat baseRotMatrixTranspose = Mat::zeros(3,3, CV_64F);
    Mat objectRotMatrix = Mat::zeros(3,3, CV_64F);

    Rodrigues(baseRotation,baseRotMatrix);
    Rodrigues(posRotation,objectRotMatrix);

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            relativeRotMatrix.at<double>(i,j) = 0;
            baseRotMatrixTranspose.at<double>(j,i) = baseRotMatrix.at<double>(i,j);
        }
    }

    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            for(k = 0; k < 3; k++)
            {
                relativeRotMatrix.at<double>(i,j) += baseRotMatrixTranspose.at<double>(i,k) * objectRotMatrix.at<double>(k,j);
            }
        }
    }

}

void cam::findRelativeVector(int basePos, int Pos, vector<Vec3d>& translationVectors, vector<Vec3d>& rotationVectors, vector<double>& posRes){
    int i,j;
    vector<double> R(3);
    Mat baseRotMatrix = Mat::zeros(3,3, CV_64F);

    /* posRes is the vector from the world coordinate system to object 1 expressed in world base vectors*/
    /* R is the vector from object to base in expressed in the camera frame*/
    for(i = 0; i < 3; i++)
        R[i] = translationVectors[Pos][i] -  translationVectors[basePos][i];

    /* R is still expressed with respect to the camera frame, to fix this we must multiply R by the transpose of the rotation matrix between the world and camera frame */
    Rodrigues(rotationVectors[basePos],baseRotMatrix);

    for(i = 0; i < 3; i++){
        posRes[i] = 0;
        for(j = 0; j < 3; j++){
            posRes[i] += baseRotMatrix.at<double>(j,i)*R[j];
        }
    }
}

int cam::startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension, vector<double>& relPos1, Mat& relativeRotMatrix, int baseMarker, int toFindMarker){
    vector<double> tempRelPos1(3);
    vector<vector<double> > tempArrayRelPos1;

    double new_y,old_y = 0;
    double tempx,tempy;
   // double counter = 0;
    Mat frame;
    vector<int> markerIds(2);
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;
    vector<Vec3d> rotationVectors, translationVectors;
    Ptr< aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_50);
    int loopCondition = 0;

    VideoCapture vid(0);

    if(!vid.isOpened()){
        return -1;
        cout << "no dice" << endl;
    }
    namedWindow("Webcam",CV_WINDOW_AUTOSIZE);
    while(loopCondition == 0){
            /* turns out the largest found y (and accompanying x) are the correct ones */
            for(int i = 0; i<30; i++){
                if(!vid.read(frame))
                    break;
                aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
                aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

                int max = markerIds.size();
                int basePos = findIndex(markerIds, baseMarker);
                int Pos1 = findIndex(markerIds, toFindMarker);
                for(int i = 0; i < max; i++){
                    aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.08f);
                }
                imshow("Webcam", frame);
                if(Pos1 != -1 && basePos != -1){
                    findRelativeVector(basePos, Pos1, translationVectors, rotationVectors, relPos1);
                        new_y = relPos1[1];
                        if(new_y > old_y){
                            tempx = relPos1[0];
                            tempy = new_y;
                            old_y = new_y;
                            findRotMatrix(basePos, Pos1, translationVectors, rotationVectors, relativeRotMatrix);
                        }
                        else if(new_y < old_y){
                            tempy = old_y;
                        }
                        //cout << "x=" << relPos1[0]*100 << "  y=" << 100*relPos1[1] << endl;
                }
                markerIds.resize(2); markerIds[0] = -1;  markerIds[1] = -1;
                waitKey(1000/fps);
            }
            relPos1[0] = tempx;
            relPos1[1] = tempy;
            loopCondition = 1;
    }
    return 1;
}

bool cam::getMatrixFromFile(string name, Mat cameraMatrix, Mat distanceCoefficients){
    ifstream inStream(name);
    if(inStream){

        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        for(int r = 0; r < rows; r++){
            for(int c = 0; c < columns; c++){
                inStream >> cameraMatrix.at<double>(r,c);
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;


        for(int r = 0; r < rows; r++){
            for(int c = 0; c < columns; c++){
                inStream >> distanceCoefficients.at<double>(r,c);
            }
        }

        inStream.close();
        return true;
    }
    return false;
}

void findRelativeVectorCharuco(Vec3d& baseRotation, Vec3d& baseTranslation, Vec3d& posTranslation,  vector<double>& posRes){
    int i,j;
    vector<double> R(3);
    Mat baseRotMatrix = Mat::eye(3,3, CV_64F);

    /* posRes is the vector from the world coordinate system to object 1 expressed in world base vectors*/
    /* R is the vector from object to base in expressed in the camera frame*/
    for(i = 0; i < 3; i++)
        R[i] = posTranslation[i] -  baseTranslation[i];
    //cout << "\r" << "Rx=" << 100*R[0] << " Ry=" << 100*R[1] << " Rz=" << 100*R[2] << "                   " << flush;
    //cout << "\r" << "POSRx=" << 100*posTranslation[0] << " Ry=" << 100*posTranslation[1] << " Rz=" << 100*posTranslation[2] << "                   " << flush;
    //cout << "\r" << "BASERx=" << 100*baseTranslation[0] << " Ry=" << 100*baseTranslation[1] << " Rz=" << 100*baseTranslation[2] << "                   " << flush;
    /* R is still expressed with respect to the camera frame, to fix this we must multiply R by the transpose of the rotation matrix between the world and camera frame */
    Rodrigues(baseRotation,baseRotMatrix);

    for(i = 0; i < 3; i++){
        posRes[i] = 0;
        for(j = 0; j < 3; j++){
            posRes[i] += baseRotMatrix.at<double>(j,i)*R[j];
        }
    }
}

int cam::findVecsCharuco(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension, vector<double>& relPos, Mat& relativeRotMatrix, int toFindMarker){
    Mat frame;
    double new_y,old_y = 0;
    double tempx,tempy;
    float axisLength = 0.5f * ((float)min(5, 3) * (0.0265f));
    bool validPose;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_7X7_50);
    Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 3, 0.0265f, 0.0198f, markerDictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    VideoCapture vid(0);
    if(!vid.isOpened()){
        return -1;
    }
    namedWindow("Webcam",CV_WINDOW_AUTOSIZE);

    for(int i = 0; i<30; i++){
        if(!vid.read(frame))
            break;
        vector<Vec3d> rotationVectors, translationVectors;
        Vec3d rvec, tvec;
        vector< Point2f > charucoCorners;
        vector< vector< Point2f > > markerCorners, rejectedMarkers;
        vector<int> markerIds, charucoIds;

        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
        if(markerIds.size()>0)
                aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, charucoboard, charucoCorners, charucoIds, cameraMatrix, distanceCoefficients);
        validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, cameraMatrix, distanceCoefficients, rvec, tvec);
        int Pos1 = findIndex(markerIds, toFindMarker);

        if(Pos1 != -1)
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[Pos1], translationVectors[Pos1], 0.08f);
        if(validPose)
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rvec, tvec, axisLength);

        if(Pos1 != -1){
            findRelativeVectorCharuco(rvec, tvec, translationVectors[Pos1], relPos);
        new_y = relPos[1];
            if(new_y > old_y){
                tempx = relPos[0];
                tempy = new_y;
                old_y = new_y;
                findRotMatrixCharuco(rvec, rotationVectors[Pos1], relativeRotMatrix);
            }
            else if(new_y < old_y){
                tempy = old_y;
            }
            cout << "x=" << relPos[0]*100 << "  y=" << 100*relPos[1] << endl;
        }
    imshow("Webcam", frame);
    waitKey(1000/fps);
    }
    relPos[0] = tempx;
    relPos[1] = tempy;
    return 1;
}
