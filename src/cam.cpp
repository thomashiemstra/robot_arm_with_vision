#include "cam.h"
#include <chrono>
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

cam::cam(uint8_t cameraNum, int framesPerSecond){
    camera = cameraNum;
    fps = framesPerSecond;
}

/* find the index of a specific marker in the array of all found markers */
int cam::findIndex(vector<int>& vec, int val){
    int res;
    uint16_t length = vec.size();
    res = find(vec.begin(), vec.end(), val) - vec.begin();
        if (res >= length){
            res = -1;
        }
    return res;
}

/* find the rotation of the marker expressed in the coordinate frame of the Charucoboard */
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

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            for(k = 0; k < 3; k++){
                relativeRotMatrix.at<double>(i,j) += baseRotMatrixTranspose.at<double>(i,k) * objectRotMatrix.at<double>(k,j);
            }
        }
    }
}

/* find the coordinates of the marker expressed in the frame of the Charucoboard */
void cam::findRelativeVectorCharuco(Vec3d& baseRotation, Vec3d& baseTranslation, Vec3d& posTranslation,  vector<double>& posRes){
    int i,j;
    vector<double> R(3);
    Mat baseRotMatrix = Mat::eye(3,3, CV_64F);

    /* posRes is the vector from the Charucoboard to the marker expressed in the coordinate frame of the board*/
    /* R is the vector from object to base in expressed in the camera frame*/
    for(i = 0; i < 3; i++)
        R[i] = posTranslation[i] -  baseTranslation[i];

    /* R is still expressed with respect to the camera frame, to fix this we must multiply R by the transpose of the rotation matrix between the world and camera frame */
    Rodrigues(baseRotation,baseRotMatrix);
    for(i = 0; i < 3; i++){
        posRes[i] = 0;
        for(j = 0; j < 3; j++){
            posRes[i] += baseRotMatrix.at<double>(j,i)*R[j];
        }
    }
}

/* constantly outputs the camera feed and calculate the position of toFindMarker when asked, used in multi threading  */
int cam::startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension,
                               vector<double>& relPos, Mat& relativeRotMatrix, int& toFindMarker,bool &getVecs, int& condition){
    Mat frame;
    /* dictionaries for all the markers I want to use */
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 3, 0.0265f, 0.0198f, markerDictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    /* vars needed to pick the best candidate from different measurements */
    int counter = 0;
    double new_y,old_y = 0;
    double tempx = 0,tempy = 0;
    VideoCapture vid(0);

    if(!vid.isOpened())
        return -1;

    vid.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') ); //MJPG drastically improves frame read times
    vid.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    vid.set(CV_CAP_PROP_FRAME_HEIGHT,720);

    namedWindow("Webcam",CV_WINDOW_AUTOSIZE);
    while(true){
        auto begin = std::chrono::high_resolution_clock::now();
        if(!vid.read(frame))
            break;
        Vec3d rvec, tvec;
        vector<Vec3d> rotationVectors, translationVectors;
        vector< Point2f > charucoCorners;
        vector< vector< Point2f > > markerCorners, rejectedMarkers;
        vector<int> markerIds, charucoIds;

        /* detect all markers, I have to detect every single markers and compute their Pose before I can pick the Pose of the specific marker I'm looking for*/
        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
        if(markerIds.size()>0)
                aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, charucoboard, charucoCorners, charucoIds, cameraMatrix, distanceCoefficients);
        int Pos1 = findIndex(markerIds, toFindMarker); /* the index of the marker I'm after*/
        if(Pos1 != -1)
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[Pos1], translationVectors[Pos1], 0.08f);

        /* detecting all markers wasn't totally useless, that info is used now to find the charucoboard*/
        bool validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, cameraMatrix, distanceCoefficients, rvec, tvec);
        if(validPose)
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rvec, tvec, 0.12f);

        imshow("Webcam", frame);

        /* find the position and rotation of the toFindMarker expressed in the coordinate frame of the Charucoboard*/
        /* the computed position with the largest y value tends to be the best one so I pick that one from all the measurements */
        int measurements = 5;
        if(Pos1 != -1 && getVecs){
            unique_lock<mutex> locker(mu); /* I need a lock on relPos and relativeRotMatrix until they are properly calculated*/
            /* relPos is the position of the marker expressed in the coordinate frame of the Charucoboard*/
            findRelativeVectorCharuco(rvec, tvec, translationVectors[Pos1], relPos);
            new_y = relPos[1];

            if(new_y > old_y){
                tempx = relPos[0];
                tempy = new_y;
                old_y = new_y;
                /* only when this measurements is better than the previous do I calculate the rotation of the marker*/
                findRotMatrixCharuco(rvec, rotationVectors[Pos1], relativeRotMatrix);
            }
            else if(new_y < old_y)
                tempy = old_y;

            counter ++;
            if(counter == measurements){ /* return the best found relative Pose*/
                counter = 0;
                getVecs = false;
                relPos[0] = tempx;
                relPos[1] = tempy;
                old_y=0;
                locker.unlock();
                cond.notify_one();
            }
        }

        /*make sure we exactly hit the target fps, waitKey(1000/fps) isn't accurate enough for my taste*/
        waitKey(1); /* without this imshow shows nothing*/
        auto temp = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = temp - begin;
        double test = fp_ms.count(); /* time elapsed so far*/
        int wait = (int)((1000.0/fps) - test); /* true fps time*/
        if(wait<0)
            wait = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(wait));
        if(condition == 0) break;
    }
    destroyWindow("Webcam");
    return 1;
}

/* computes the relative position and rotation between 2 charuco boards */
int cam::copyMovement(const Mat& cameraMatrix, const Mat& distanceCoefficients,
                      vector<double>& relPos, Mat& relativeRotMatrix, bool &getVecs, int& condition){
    Mat frame;
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 5, 0.0265f, 0.0198f, markerDictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    Ptr<aruco::Dictionary> markerDictionary1 = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
    Ptr<aruco::CharucoBoard> charucoboard1 = aruco::CharucoBoard::create(6, 4, 0.0265f, 0.0198f, markerDictionary1);
    Ptr<aruco::Board> board1 = charucoboard1.staticCast<aruco::Board>();

    bool validPose;
    bool validPose1;

    VideoCapture vid(0);

    if(!vid.isOpened()){
        return -1;
    }
    vid.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );//MJPG drastically improves frame read times
    vid.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    vid.set(CV_CAP_PROP_FRAME_HEIGHT,720);

    namedWindow("Webcam",CV_WINDOW_AUTOSIZE);
    while(true){
        auto begin = std::chrono::high_resolution_clock::now();
        if(!vid.read(frame))
            break;
        //auto end = std::chrono::high_resolution_clock::now();
        //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << "ms" << std::endl;

        Vec3d rvecP, tvecP;
        Vec3d rvec, tvec;
        vector< Point2f > charucoCorners;
        vector< Point2f > charucoCorners1;
        vector< vector< Point2f > > markerCorners, rejectedMarkers;
        vector< vector< Point2f > > markerCorners1, rejectedMarkers1;
        vector<int> markerIds, charucoIds;
        vector<int> markerIds1, charucoIds1;
        /* detect base board*/
        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        if(markerIds.size()>0)
            aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, charucoboard, charucoCorners, charucoIds, cameraMatrix, distanceCoefficients);
        validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, cameraMatrix, distanceCoefficients, rvec, tvec);

        /* detect position board */
        aruco::detectMarkers(frame, markerDictionary1, markerCorners1, markerIds1);
        if(markerIds1.size()>0)
            aruco::interpolateCornersCharuco(markerCorners1, markerIds1, frame, charucoboard1, charucoCorners1, charucoIds1, cameraMatrix, distanceCoefficients);
        validPose1 = aruco::estimatePoseCharucoBoard(charucoCorners1, charucoIds1, charucoboard1, cameraMatrix, distanceCoefficients, rvecP, tvecP);

        if( validPose && validPose1){
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rvec, tvec, 0.12f);
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rvecP, tvecP, 0.12f);
        }

        /* find x,y and theta(rotation around the z-axis)*/
        if(validPose && validPose1 && getVecs){
            unique_lock<mutex> locker(mu);
            findRelativeVectorCharuco(rvec, tvec, tvecP, relPos);
            findRotMatrixCharuco(rvec, rvecP, relativeRotMatrix);
            //cout << "\r" << " dx=" << 100*relPos1[0] << " dy=" << 100*relPos1[1] << "  dz=" << 100*relPos1[2] <<"                   " << flush;
            getVecs = false;
            locker.unlock();
            cond.notify_one();
        }
        imshow("Webcam", frame);
        /*make sure we wait exactly 1000/fps milliseconds */
        if(waitKey(1) == 27); //wait for 'esc' key press. If 'esc' key is pressed, break loop
        auto temp = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = temp - begin;
        double test = fp_ms.count(); /* time elapsed so far*/
        //cout << "\r" << " time=" << test << "ms" <<"                   " << flush;
        int wait = (int)((1000.0/fps) - test); /* true fps time*/
        if(wait<0)
            wait = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(wait));
        if(condition == 0) break;
    }
    destroyWindow("Webcam");
    return 1;
}

/* load the calibration matrix */
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

/* Instead of taking as input a single marker it takes in an array of markers and return the Pose for all of them at once, very similar to startWebcamMonitoring()*/
int cam::findVecsCharuco(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension,
                         vector<vector<double > >& relPos, vector<Mat >& relativeRotMatrix, vector<int >& toFindMarkers, vector<int >& foundMarkers){
    Mat frame;
    vector<double > new_y(relPos.size()), old_y(relPos.size()), tempx(relPos.size()), tempy(relPos.size());

    for(unsigned int i=0; i<relPos.size(); i++)
        old_y[i] = 0;

    float axisLength = 0.5f * ((float)min(5, 3) * (0.0265f));
    bool validPose;
    int index;
    int marker;
    int inFoundMarkers;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(5, 3, 0.0265f, 0.0198f, markerDictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    VideoCapture vid(0);
    if(!vid.isOpened()){
        return -1;
    }
    vid.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );/* MJPG drastically improves frame read times */
    vid.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    vid.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    namedWindow("Webcam",CV_WINDOW_AUTOSIZE);

    /* try to find each vector 10 times and take the position with the largest y (which is usually the most accurate reading)*/
    for(int j = 0; j<10; j++){
        auto begin = std::chrono::high_resolution_clock::now();
        if(!vid.read(frame))
            break;

        vector<Vec3d> rotationVectors, translationVectors;
        Vec3d rvec, tvec; /* for the charucoboard */
        vector< Point2f > charucoCorners;
        vector< vector< Point2f > > markerCorners, rejectedMarkers;
        vector<int> markerIds, charucoIds;

        aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
        if(markerIds.size()>0)
                aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, charucoboard, charucoCorners, charucoIds, cameraMatrix, distanceCoefficients);
        validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, cameraMatrix, distanceCoefficients, rvec, tvec);
        if(validPose)
            aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rvec, tvec, axisLength);

        for(unsigned int i=0; i<toFindMarkers.size(); i++){
            marker = toFindMarkers[i];
            index = findIndex(markerIds, marker);
            if(index != -1){

                inFoundMarkers = findIndex(foundMarkers, marker);  /* is the marker already marked as found? */
                if(inFoundMarkers == -1) /* if not, add it to the list. We only want to add it once.*/
                    foundMarkers.push_back(marker);

                aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[index], translationVectors[index], 0.08f);
                findRelativeVectorCharuco(rvec, tvec, translationVectors[index], relPos[marker]);
                new_y[marker] = relPos[marker][1];

                if(new_y[marker] > old_y[marker]){
                    tempx[marker] = relPos[marker][0];
                    tempy[marker] = relPos[marker][1];
                    old_y[marker] = new_y[marker];
                    findRotMatrixCharuco(rvec, rotationVectors[index], relativeRotMatrix[i]);
                }
                else if(new_y[marker] < old_y[marker]){
                    tempy[marker] = old_y[marker];
                }
            }
        }
    waitKey(1);
    imshow("Webcam", frame);

    auto temp = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = temp - begin;
    double test = fp_ms.count();
    int wait = (int)((1000.0/fps) - test);
    if(wait<0)
        wait = 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(wait));
    //auto end = std::chrono::high_resolution_clock::now();
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << "ms" << std::endl;
    }
    for(unsigned int i=0; i<foundMarkers.size(); i++){
        marker = foundMarkers[i];
        relPos[marker][0] = tempx[marker];
        relPos[marker][1] = tempy[marker];
    }
    destroyWindow("Webcam");
    return 1;
}
