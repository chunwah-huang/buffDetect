#ifndef BUFF_DETECT_H
#define BUFF_DETECT_H

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef enum{UNKOWN,INACTION,ACTION}ObjectType;
class Object
{
public:
    Object(){}
    ~Object(){}
    void smallUpdateOrder(); // 更新能量机关装甲板的绝对位置
    void bigUpdateOrder();
    void KnowYourself(Mat &img);    //判断能量机关扇叶的状态（激活　未激活）

    RotatedRect small_rect_;    // 能量机关扇叶内轮廓
    RotatedRect big_rect_;  // 能量机关扇叶外轮廓
    vector<Point2f> points_2d_; // ｐｎｐ角度解算的四个点
    vector<Point2f> big_points_2d_;
    float angle_ = 0;
    float diff_angle = 0;
    int type_ = UNKOWN;
};

class BuffDetector
{
public:
    BuffDetector() {}
    ~BuffDetector(){}
    int BuffDetectTask(Mat & img);
private:
    bool ImageProcess(Mat & img);
    bool FindCenterR(Mat & bin_img, Mat &img);
    int getState();

private:
     vector<Point2f> points_2d;
     vector<Point2f> big_points_2d;
     Point2f target_center;
     Point2f roi_center;
     Point2f circle_center;
     Point2f pre_center;
     Mat binImg;


private:
    const int color_ = 1;
    const int pre_angle = 20;

private://能量机关顺逆时针判断
    const float r = 0.1;
    float buff_angle_ = 0;
    float diff_angle_ = 0;
    float last_angle = 0;
    float d_angle_ = 0;
    int find_cnt = 0;
    int direction_tmp = 0;
};

#endif // BUFF_DETECT_H
double point_Distance(Point2f & p1, Point2f & p2);
int GetRectIntensity(const Mat &img, Rect rect);
