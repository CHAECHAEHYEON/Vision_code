#ifndef __VISION_FUNC_H__
#define __VISION_FUNC_H__


#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

class WeAreVision
{
private:
    /* data */
public:
    // Vision_func(/* args */);
    Mat warp_matrix_inv;
    Mat warp_matrix;
    Mat img_color;
    Mat frame1, frame2;

    Mat imgUndistort, imgUnwarp;
    Mat imgHLS_L;
    Mat imgLAB_B;
    Mat combineOut;


    // vector<Point> sliding_window(Mat img);
    // vector<double> polyFit(vector<Point> px, int i, int degree);
    // Scalar RGB_mean(Mat img, int X, int width, int Y, int height);

    Point2f warp_SRC_ROI[4];
    Point2f warp_DST_ROI[4];
    Point ptOld1;
    int isStop = 100;
    int H, S, V;
    bool callback = false;	
    int first_run = 1; // KEY for compute speed!!!

    Mat make_zeros(Mat img);
    Scalar IMG_mean(Mat img, int X, int width, int Y, int height);
    Mat make_ones(Mat img);
    Mat filterImg(Mat imgUnwarp, int toColorChannel, int mode);
    Mat combine_both_img(Mat hls, Mat lab);
    Mat normalize_HLS_L(Mat unWarp);
    Mat normalize_LAB_B(Mat unWarp);

    Mat bird_eyes_view(Mat img);
    Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh);
    Mat STOP_preprocessing(Mat img, Mat img_warp);

    void DISPLAY_meter(Mat final, int count);
    void on_mouse(int event, int x, int y, int flags, void *);
    void mouse_callback(int event, int x, int y, int flags, void *param);

    // ~Vision_func();
};

// Vision_func::Vision_func(/* args */)
// {
// }

// Vision_func::~Vision_func()
// {
// }



#endif