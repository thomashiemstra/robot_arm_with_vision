#include "tricks.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

using namespace cv;
using namespace std;

Tricks::Tricks()
{
    return;
}

//void Tricks::sendStuff(int16_t *val){ //sending 7 2 byte ints over serial
//	uint8_t bytes[16];
//    for(int i =0; i < 8; i++){
//        bytes[2*i] = (val[i] >> 8) &0xff;
//        bytes[2*i+1] = val[i] & 0xff;
//    }
//	arduino->WriteData(bytes,16);
//}
