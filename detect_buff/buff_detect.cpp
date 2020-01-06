#include "buff_detect.h"


bool BuffDetector::ImageProcess(Mat & img){

    Mat gaussImg;
    GaussianBlur(img, gaussImg, Size(3, 3), 0);
    points_2d.clear();
    vector<Mat> bgr;

    split(gaussImg, bgr);
    if(color_ == 0){
        subtract(bgr[2], bgr[1], gaussImg);
    }
    else{
        subtract(bgr[0], bgr[2], gaussImg);
    }
    double th = threshold(gaussImg, binImg, 35, 255,  0);
//    if(th-10 > 0)
//        threshold(gaussImg, binImg, th-15, 255,  0);

    dilate(binImg, binImg, getStructuringElement(MORPH_RECT, Size(3,3)));
    //imshow("bin", binImg);
    //cout << "th:" << th << endl;
    vector<Object> vec_target;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(binImg, contours, hierarchy, 2, CHAIN_APPROX_NONE);
    for(int i = 0; i < (int)contours.size(); ++i){
        // 用于寻找小轮廓，没有父轮廓的跳过, 以及不满足6点拟合椭圆
        if(hierarchy[i][3]<0 || contours[i].size() < 6 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;

        // 小轮廓面积条件
        double small_rect_area = contourArea(contours[i]);
        double small_rect_length = arcLength(contours[i],true);
        if(small_rect_length < 10)
            continue;
        if(small_rect_area < 200 || small_rect_area > 2000)
            continue;
        // 大轮廓面积条件
        double big_rect_area = contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
        double big_rect_length = arcLength(contours[static_cast<uint>(hierarchy[i][3])],true);
        if(big_rect_area < 300 || big_rect_area > 1e4)
            continue;
        if(big_rect_length < 50)
            continue;

        Object object;

        object.small_rect_=fitEllipse(contours[i]);
        object.big_rect_ = fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);

        Point2f small_point_tmp[4];
        object.small_rect_.points(small_point_tmp);
        Point2f big_point_tmp[4];
        object.big_rect_.points(big_point_tmp);

        //组合符合条件的装甲板和叶片
        object.diff_angle =fabsf(object.big_rect_.angle-object.small_rect_.angle);
        if(object.diff_angle<100 && object.diff_angle>80){
            float small_rect_size_ratio;
            if(object.small_rect_.size.width > object.small_rect_.size.height){
                small_rect_size_ratio = object.small_rect_.size.width/object.small_rect_.size.height;
            }else {
                small_rect_size_ratio = object.small_rect_.size.height/object.small_rect_.size.width;
            }
            //cout << big_rect_area << "  " << small_rect_area << endl;
            double area_ratio = (double)object.small_rect_.boundingRect().area()/(double)object.big_rect_.boundingRect().area();
            //cout << area_ratio << endl;
            object.type_ = UNKOWN;
            //再次清洗目标找出叶片
            if(small_rect_size_ratio>1 && small_rect_size_ratio<3 && area_ratio>0.08f && area_ratio<0.25f){
                //                for(int k=0;k<4;k++){
                //                    line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 255), 1);
                //                    line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                //                }
                object.type_ = ACTION;
                //直接找出未激活状态目标
                if(/*small_rect_area*9>big_rect_area && small_rect_area*3<big_rect_area*/0){
                    object.type_ = INACTION;
                    object.smallUpdateOrder();
                    //object.bigUpdateOrder();
                    for(int k=0;k<4;k++){
                        line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 0, 255), 3);
                        //line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                    }
                }
                else {//未能找出未激活目标,将识别的激活目标逐个筛选出
                    object.smallUpdateOrder();
                    object.KnowYourself(binImg);
                    if(object.type_ == INACTION){//若筛选出
                        for(int k=0;k<4;k++){
                            line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 0), 3);
                            //line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
                        }
                        object.bigUpdateOrder();
                    }
                    else {//未能筛选出未激活状态目标
//                        for(int k=0;k<4;k++){
//                            line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 255), 1);
//                            line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 1);
//                        }
                    }
                }
                //所有识别目标装进容器,无论是否是激活状态
                vec_target.push_back(object);
            }
        }
    }
    //cout << vec_target.size() << endl;
    Object final_target;
    bool is_target = false;
    for(int i2 = 0; i2 < (int)vec_target.size(); i2++){
        Object object_tmp = vec_target.at(i2);
        if(object_tmp.type_ == INACTION){//清洗容器得出未激活的目标
            is_target = true;
            final_target = vec_target.at(i2);
            buff_angle_ = final_target.angle_;
            points_2d = final_target.points_2d_;
            big_points_2d = final_target.big_points_2d_;
            target_center = final_target.small_rect_.center;
            //circle(img, big_points_2d[1], 2, Scalar(0,255,255), 2, 8, 0);
            Point2f big_vector = final_target.small_rect_.center - final_target.big_rect_.center;
            roi_center = (big_points_2d[1]+big_points_2d[2])/2-big_vector;
            //circle(img, roi_center, 2, Scalar(0,0,255), 2, 8, 0);
        }
    }
    //imshow("src", img);
    return is_target;
}

bool BuffDetector::FindCenterR(Mat &bin_img, Mat &img){
    bool is_circle = false;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(bin_img, contours, hierarchy, 0, CHAIN_APPROX_NONE);
    for(int j = 0; j < (int)contours.size(); ++j){
        double circle_area = contourArea(contours[j]);

        if(circle_area > 250 || circle_area < 30)
            continue;
        RotatedRect circle_rect = fitEllipse(contours[j]);
        double rect_ratio;
        if((double)circle_rect.boundingRect().width > (double)circle_rect.boundingRect().height){
            rect_ratio = (double)circle_rect.boundingRect().width/(double)circle_rect.boundingRect().height;
        }
        else {
            rect_ratio = (double)circle_rect.boundingRect().height/(double)circle_rect.boundingRect().width;
        }

        if(rect_ratio > 0.9f && rect_ratio < 1.1f){
            circle_center = circle_rect.center;
            is_circle = true;
            //drawContours(img, contours, j, Scalar(0,255,0),2);
            ellipse(img,circle_rect,Scalar(0,255,233),2,8);
        }
    }
    return is_circle;
}

int BuffDetector::BuffDetectTask(Mat &img){
    bool is_target = ImageProcess(img);
    int common = 0;
    Mat result_img;
    Mat roi_img;
    binImg.copyTo(result_img);
    img.copyTo(roi_img);

    if(is_target){//可找到未激活目标
        RotatedRect roi_R(roi_center, Size(90,90), 0);
        Rect roi = roi_R.boundingRect();

        if(roi.tl().x < 0 || roi.tl().y < 0|| roi.br().x > img.cols || roi.br().y > img.rows){
            Point TL = roi.tl();
            Point BR = roi.br();
            if(roi.tl().x < 0 || roi.tl().y < 0){
                if(roi.tl().x<0){
                    TL.x = 0;
                }
                else {
                    TL.y = 0;
                }
            }
            else {
                if(roi.br().x>img.cols){
                    BR.x = img.cols;
                }
                else {
                    BR.y = img.rows;
                }
            }
            roi = Rect(TL, BR);
        }

        binImg(roi).copyTo(result_img);
        img(roi).copyTo(roi_img);

        rectangle(img,roi,Scalar(0,255,200),2,8,0);

        bool is_circle = FindCenterR(result_img, roi_img);
        if(is_circle == true){
            double theta = atan(double(target_center.y - circle_center.y) / (target_center.x - circle_center.x));
            double total = (pre_angle+theta)*CV_PI/180;
            double sin_calcu = sin(total);
            double cos_calcu = cos(total);
            Point2f round_center(circle_center.x+roi.tl().x, circle_center.y+roi.tl().y);
            pre_center.x = (target_center.x-round_center.x)*cos_calcu-(target_center.y-round_center.y)*sin_calcu+round_center.x;
            pre_center.y = (target_center.x-round_center.x)*sin_calcu+(target_center.y-round_center.y)*cos_calcu+round_center.y;
            double radio = point_Distance(round_center, pre_center);
            circle(img, round_center, radio, Scalar(0,255,125),2,8,0);
            circle(img, pre_center, 3, Scalar(255,0,0),3,8,0);
        }
        else{

        }

        ++find_cnt;
        if(find_cnt%2 == 0){//隔帧读数据
            direction_tmp = getState();
            if(find_cnt == 10)
                find_cnt = 0;
        }
         if(direction_tmp == 1){//good

        }
        else if(direction_tmp == -1){//poor

        }
        else {//stay

        }
    }
    //imshow("roi", result_img);
    //imshow("roi_img", roi_img);
    imshow("bin", binImg);
    imshow("img", img);
    //cout << direction_tmp << endl;
    return common;
}

int BuffDetector::getState(){
    diff_angle_ = buff_angle_ - last_angle;
    last_angle = buff_angle_;
    if(fabs(diff_angle_)<10 && fabs(diff_angle_)>1e-6){
        d_angle_ = (1 - r) * d_angle_ + r * diff_angle_;
    }

    if(d_angle_ > 1.5f)
        return 1;
    else if(d_angle_ < -1.5f)
        return -1;
    else
        return 0;
}

void Object::smallUpdateOrder(){
    points_2d_.clear();
    Point2f points[4];
    small_rect_.points(points);
    Point2f point_up_center = (points[0] + points[1])/2;
    Point2f point_down_center = (points[2] + points[3])/2;
    double up_distance = point_Distance(point_up_center, big_rect_.center);
    double down_distance = point_Distance(point_down_center, big_rect_.center);
    //cout << big_rect_.angle << endl;

    if(up_distance > down_distance){
        angle_ = small_rect_.angle;
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
    }
    else {
        angle_ = small_rect_.angle+180;
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
    }

    //cout << angle_ << endl;
}

void Object::bigUpdateOrder(){
    big_points_2d_.clear();
    Point2f big_points[4];
    big_rect_.points(big_points);
    Point2f point_up_center = (big_points[1] + big_points[2])/2;
    Point2f point_down_center = (big_points[0] + big_points[3])/2;
    double up_distance = point_Distance(point_up_center, small_rect_.center);
    double down_distance = point_Distance(point_down_center, small_rect_.center);
    if(up_distance > down_distance){
        big_points_2d_.push_back(big_points[0]);big_points_2d_.push_back(big_points[1]);
        big_points_2d_.push_back(big_points[2]);big_points_2d_.push_back(big_points[3]);
    }
    else {
        big_points_2d_.push_back(big_points[2]);big_points_2d_.push_back(big_points[3]);
        big_points_2d_.push_back(big_points[0]);big_points_2d_.push_back(big_points[1]);
    }
}

void Object::KnowYourself(Mat &img){
    Point2f vector_height = points_2d_.at(0) - points_2d_.at(3);

    Point left_center = points_2d_.at(3) - vector_height;
    Point right_center = points_2d_.at(2) - vector_height;

    int width = 5;
    int height = 5;

    Point left1 = Point(left_center.x - width, left_center.y - height);
    Point left2 = Point(left_center.x + width, left_center.y + height);

    Point right1 = Point(right_center.x - width, right_center.y - height);
    Point right2 = Point(right_center.x + width, right_center.y + height);

    Rect left_rect(left1, left2);
    Rect right_rect(right1, right2);

    int left_intensity = GetRectIntensity(img, left_rect);
    int right_intensity = GetRectIntensity(img, right_rect);
    //cout << left_intensity << "  " << right_intensity << endl;
    if(left_intensity <= 20 && right_intensity <= 20){
        type_ = INACTION;
    }
    else {
        type_ = ACTION;
    }
}

double point_Distance(Point2f & p1, Point2f & p2){
    double Dis=pow(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2),0.5);
    return Dis;
}

int GetRectIntensity(const Mat &img, Rect rect){
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
    //imshow("roi ", roi);
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}


