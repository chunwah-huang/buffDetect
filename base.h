#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
//标定XML路径
#define XML_PATH "./cameraParams_infantry4.xml"
//buff-model尺寸
#define BULLET_SPEED 1//子弹射速
#define BUFF_BOTTOM_H 519//buff最底装甲板距离地面高度
#define ROBOT_H 400//枪口高度
#define BUFF_ROBOT_Z 7200//枪口和buff的直线距离

#endif // BASE_H
