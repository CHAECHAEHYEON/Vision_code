#include "opencv2/opencv.hpp"
#include <numeric>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"

using namespace cv;
using namespace std;

cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);
bool detect_stopline = false;
Mat bird_eyes_view(Mat img);
int max_arr(int arr[], int arr_len);
vector<Point2f> slidingWindow(Mat image, Rect window);
Mat warp_matrix, warp_matrix_inv;
float ekf_fuction(float point, int number);
int point_history[2];
int point_history_right[2];
bool first_run = 1;
bool first_run_right = 1;

bool first_init = 1;
bool first_index = 1;

bool first_init_right = 1;
bool first_index_right = 1;

bool get2ndOrderRegression(std::vector<double> *srcX, std::vector<double> *srcY, double *b0, double *b1, double *b2);
bool RANSAC123(std::vector<double> srcX, std::vector<double> srcY, double *b0, double *b1, double *b2);
// Mat mask_filter(Mat img, int w, int h, int thresh);
int isStop = 10;
float difference_dist = 0, difference_dist1 = 0, difference_dist2 = 0, difference_dist3 = 0;
float difference_dist4 = 0, difference_dist5 = 0, difference_dist6 = 0;
char *pbuffer, *pbuffer1, *pbuffer2, *pbuffer3;
char *pbuffer4, *pbuffer5, *pbuffer6, *pbuffer7;
char buffer[100], buffer1[100], buffer2[100], buffer3[100];
char buffer4[100], buffer5[100], buffer6[100], buffer7[100];

Point pt01d;
void on_mouse(int event, int x, int y, int flags, void*);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 10);
    image_transport::Publisher pub1 = it.advertise("camera/image_final", 10);
    ros::Publisher lane_detection_pub = nh.advertise<std_msgs::Float32MultiArray>("LaneDetection", 10);
    ros::Publisher stopline_pub = nh.advertise<std_msgs::Int32>("StopLineDistance", 10);

    ros::Rate loop_rate(50);
    sensor_msgs::ImagePtr msg;
    sensor_msgs::ImagePtr msg1; 
    std_msgs::Float32MultiArray lane_msg;
    std_msgs::Int32 stopline_msg;
    int index[3];
    int index2[3];
    int index_right[3];
    int index2_right[3];

    // VideoCapture cap("/home/usera/catkin_ws/src/stopline_2021/src/line_corn2.mp4");
    // VideoCapture cap("/home/usera/catkin_ws/src/line_detection/src/test.mp4");
    // VideoCapture cap("/home/usera/catkin_ws/src/line_detection/src/산학좌회전.mp4");
    VideoCapture cap("/home/kim1999/catkin_ws/src/test_1/src/video/front_stopline.mp4");
    //    VideoCapture cap(0);

    if (!cap.isOpened())
    {
        cerr << "video load fail!!\n"
             << endl;
    }

    cout << "Frame width: " << cvRound(cap.get(CAP_PROP_FRAME_WIDTH)) << endl;
    cout << "Frame height: " << cvRound(cap.get(CAP_PROP_FRAME_HEIGHT)) << endl;
    cout << "Frame count: " << cvRound(cap.get(CAP_PROP_FRAME_COUNT)) << endl;
    double fps = cap.get(CAP_PROP_FPS);
    cout << "FPS: " << fps << endl;
    // int delay = cvRound(1000 / fps);
    Mat src;

    Point poly[1][4];
    Point poly_under[1][4];
    Point poly_top[1][4];

    Point poly_right[1][4];
    Point poly_under_right[1][4];
    Point poly_top_right[1][4];
    Mat HLS_image, HSV_image;
    Mat HLS_image_right, HSV_image_right;

    Scalar lower_red = Scalar(0, 0, 135); //red색 차선 (HSV)
    Scalar upper_red = Scalar(255, 100, 255);
    Scalar lower_yellow = Scalar(15, 0, 110); //노란색 차선 (HSV)
    Scalar upper_yellow = Scalar(120, 80, 210);
    Scalar lower_white = Scalar(0, 0, 150);
    Scalar upper_white = Scalar(255, 50, 255);
    // Scalar lower_white_ = Scalar(0, 100, 0);
    // Scalar upper_white_ = Scalar(255, 130, 255); // 160~180
    // Scalar lower_white__ = Scalar(0, 180, 0);
    // Scalar upper_white__ = Scalar(255, 230, 255);
    Mat warp_view;
    Mat yellow_image, white_image, red_image, white_image_, white_image__;
    Mat White_Plus_Yellow__;
    
    Mat White_Plus_Yellow_;
    Mat White_Plus_Yellow;
    Mat White_Plus_Yellow_minus_red;
    Mat poly_roi_right;
    Mat poly_roi_under_right;
    Mat poly_roi_top_right;
    Mat poly_roi;
    Mat poly_roi_under;
    Mat poly_roi_top;


    Mat marph_open;
    Mat marph_close;

    int sum_index = 0;
    int sum_index2 = 0;

    int sum_index_right = 0;
    int sum_index2_right = 0;

    while (ros::ok())
    {
        //연산속도 측정을 위한 것(1)
        int64 time01 = getTickCount();
        //원본
        cap >> src;
        resize(src, src, Size(640, 480));

        if (src.empty())
        {
            printf("finish!!\n");
            break;
        }

        if (waitKey(1) == 27)
        {
            break;
            printf("강제 종료\n");
        }
        imshow("view", src);
        setMouseCallback("view", on_mouse);
        if (first_run == 1)
        {
            warp_matrix, warp_matrix_inv = bird_eyes_view(src);
            first_run = 0;
        }
        // imshow("src",src);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        pub.publish(msg);
        // 시점변환

        cv::warpPerspective(src, warp_view, warp_matrix, cv::Size()); //버드아이뷰 전환
        imshow("src",warp_view);

        Mat gray_img(warp_view.rows, warp_view.cols, CV_8UC1);

        for (int y = 0; y < warp_view.rows; y++)
        {
            for (int x = 0; x < warp_view.cols; x++)
            {
                int avg = (warp_view.at<Vec3b>(y, x)[0] + warp_view.at<Vec3b>(y, x)[1] + warp_view.at<Vec3b>(y, x)[2]) / 3;
                gray_img.at<uchar>(y, x) = avg;
            }
        }
        imshow("aaaaaa", gray_img);

        Mat dx, dy;
        Sobel(gray_img, dx, CV_32FC1, 1, 0);
        Sobel(gray_img, dy, CV_32FC1, 0, 1);

        Mat fmag, mag;
        magnitude(dx, dy, fmag);
        fmag.convertTo(mag, CV_8UC1);
        Mat edge = mag >50;
        imshow("edge", edge);
        cvtColor(warp_view, HLS_image, CV_BGR2HLS);
        cvtColor(warp_view, HSV_image, CV_BGR2HSV);
        

        inRange(warp_view, lower_red, upper_red, red_image);
        inRange(HSV_image, lower_yellow, upper_yellow, yellow_image);
        inRange(HSV_image, lower_white, upper_white, white_image);
        // inRange(HLS_image, lower_white_, upper_white_, white_image_);
        // inRange(HLS_image, lower_white__, upper_white__, white_image__);
        
        // imshow("노란색 검출", yellow_image);
        // imshow("흰색 검출", white_image);
        // imshow("HLS검출", white_image__);

        // imshow("red 검출", red_image);
        // imshow("red 팽창 검출", red_image);

        // Mat White_Plus_Yellow = yellow_image + white_image_;
        // White_Plus_Yellow__ = white_image - white_image_;     
        // White_Plus_Yellow_ = White_Plus_Yellow__ + white_image__;
        // White_Plus_Yellow = White_Plus_Yellow_ + yellow_image ;
        // White_Plus_Yellow_minus_red = White_Plus_Yellow - red_image;
        White_Plus_Yellow = yellow_image + white_image;
        White_Plus_Yellow_minus_red = White_Plus_Yellow - red_image;
        dilate(White_Plus_Yellow_minus_red, White_Plus_Yellow_minus_red, cv::Mat());
        dilate(White_Plus_Yellow_minus_red, White_Plus_Yellow_minus_red, cv::Mat());

        imshow("~White_Plus_Yellow_minus_red", ~White_Plus_Yellow_minus_red);
        //이진 영상 닫기        
        // erode(edge, marph_close, cv::Mat());
        // erode(marph_close, marph_close, cv::Mat());
        dilate(edge, marph_close, cv::Mat());
        marph_close = edge - (~White_Plus_Yellow_minus_red);
        // imshow("open-1",White_Plus_Yellow_minus_red);
        imshow("open", marph_close);
        // waitKey();



        // imshow("open", marph_close);
        //histogram ROI아랫부분
        sum_index = 0;
        sum_index2 = 0;

        sum_index_right = 0;
        sum_index2_right = 0;

        for (int j = marph_close.rows * 3 / 4; j < marph_close.rows; j++)
        {
            for (int i = marph_close.cols/2; i > 0; i--)
            {
                if (marph_close.at<uchar>(j, i) != 0)
                {
                    index[0] = i;
                    break;
                }
            }
            for (int i = marph_close.cols/2; i < marph_close.cols; i++)
            {
                if (marph_close.at<uchar>(j, i) != 0)
                {
                    index_right[0] = i;
                    break;
                }
            }

            if(first_index == 0)
            {
                if((double)abs(index[0]-index[1])/index[0]*100 > 20)
                {
                    index[0] = index[1];
                }
            }
            if(first_index_right == 0)
            {
                if((double)abs(index_right[0]-index_right[1])/index_right[0]*100 > 20  )
                {
                    index_right[0] = index_right[1];
                }        
            }
            // cout << "index_right[0] : " << index_right[0] << endl;
            sum_index = sum_index + index[0];
            sum_index_right = sum_index_right + index_right[0];
        }
        index[0] = sum_index/(marph_close.rows / 4);
        index_right[0] = sum_index_right/(marph_close.rows / 4);

        // cout << "index[0]" << index[0] << endl;
        //histogram ROI윗부분[]


        for (int j = marph_close.rows * 0; j < marph_close.rows / 4; j++)
        {
            for (int i = marph_close.cols/2; i > 0; i--)
            {
                if (marph_close.at<uchar>(j, i) != 0)
                {
                    index2[0] = i;
                    break;
                }
            }
            for (int i = marph_close.cols/2; i < marph_close.cols; i++)
            {
                if (marph_close.at<uchar>(j, i) != 0)
                {
                    index2_right[0] = i;
                    break;
                }
            }

            if(first_index == 0)
            {
                if((double)abs(index2[0]-index2[1])/index2[0]*100 > 20)
                {
                    index2[0] = index2[1];
                }
            }
            if(first_index_right == 0)
            {
                if((double)abs(index2_right[0]-index2_right[1])/index2_right[0]*100 > 20)
                {
                    index2_right[0] = index2_right[1];
                }        
            }
            sum_index2 = sum_index2 + index2[0];
            sum_index2_right = sum_index2_right + index2_right[0];
        }
        index2[0] = sum_index2/(marph_close.rows / 4);
        index2_right[0] = sum_index2_right/(marph_close.rows / 4);
        first_index = 0;
        first_index_right = 0;
        

        //ROI 설정 (좌)

        Mat poly_mask = Mat::zeros(marph_close.size(), marph_close.type());
        Mat poly_mask_under = Mat::zeros(marph_close.size(), marph_close.type());
        Mat poly_mask_top = Mat::zeros(marph_close.size(), marph_close.type());


        Mat poly_mask_right = Mat::zeros(marph_close.size(), marph_close.type());
        Mat poly_mask_under_right = Mat::zeros(marph_close.size(), marph_close.type());
        Mat poly_mask_top_right = Mat::zeros(marph_close.size(), marph_close.type());

        if (first_init == 1)
        {
            index[1] = index[0];
            index2[1] = index2[0];

            index_right[1] = index_right[0];
            index2_right[1] = index2_right[0];
            first_init = 0;
        }

        if ((double)abs(index[0]-index[1])/index[0]*100 > 30)
        {
            if ((index[0] - index[1]) * (index[1] - index[2]) < 0)
            {
                // printf("노이즈 측정됨!\n");
                index[0] = index[1];
            }
            else
            {
                // printf("갑자기 점프11!!\n");
                index[0] = index[1] - 5;
            }
        }

        if ((double)abs(index2[0]-index2[1])/index2[0]*100 > 30)
        {
            if ((index2[0] - index2[1]) * (index2[1] - index2[2]) < 0)
            {
                // printf("노이즈 측정됨!\n");
                index2[0] = index2[1];
            }
            else
            {
                // printf("갑자기 점프11!!\n");
                index2[0] = index2[1] - 5;
            }
        }


        poly[0][0] = Point(index[0] - 50, marph_close.rows);
        poly[0][1] = Point(index[0] + 40, marph_close.rows);
        poly[0][2] = Point(index2[0] + 40, 0);
        poly[0][3] = Point(index2[0] - 50, 0);

        index[2] = index[1];
        index[1] = index[0];
        index2[2] = index2[1];
        index2[1] = index2[0];


        if ((double)abs(index_right[0]-index_right[1])/index_right[0]*100 > 30)
        {
            if ((index_right[0] - index_right[1]) * (index_right[1] - index_right[2]) < 0)
            {
                // printf("노이즈 측정됨!\n");
                index_right[0] = index_right[1];
            }
            else
            {
                // printf("갑자기 점프11!!\n");
                index_right[0] = index_right[1] + 5;
            }
        }
        if ((double)abs(index2_right[0]-index2_right[1])/index2_right[0]*100 > 30)
        {
            if ((index2_right[0] - index2_right[1]) * (index2_right[1] - index2_right[2]) < 0)
            {
                // printf("노이즈 측정됨!\n");
                index2_right[0] = index2_right[1];
            }
            else
            {
                // printf("갑자기 점프11!!\n");
                index2_right[0] = index2_right[1] + 5;
            }
        }

        poly_right[0][0] = Point(index_right[0] - 40, marph_close.rows);
        poly_right[0][1] = Point(index_right[0] + 50, marph_close.rows);
        poly_right[0][2] = Point(index2_right[0] + 50, 0);
        poly_right[0][3] = Point(index2_right[0] - 40, 0);

        index_right[2] = index_right[1];
        index_right[1] = index_right[0];
        index2_right[2] = index2_right[1];
        index2_right[1] = index2_right[0];


        poly_under[0][0] = Point(index[0] - 50, marph_close.rows);
        poly_under[0][1] = Point(index[0] + 40, marph_close.rows);
        poly_under[0][2] = Point(index[0] + 40, 0);
        poly_under[0][3] = Point(index[0] - 50, 0);
        
        poly_top[0][0] = Point(index2[0] - 50, marph_close.rows);
        poly_top[0][1] = Point(index2[0] + 40, marph_close.rows);
        poly_top[0][2] = Point(index2[0] + 40, 0);
        poly_top[0][3] = Point(index2[0] - 50, 0);

        poly_under_right[0][0] = Point(index_right[0] - 40, marph_close.rows);
        poly_under_right[0][1] = Point(index_right[0] + 50, marph_close.rows);
        poly_under_right[0][2] = Point(index_right[0] + 50, 0);
        poly_under_right[0][3] = Point(index_right[0] - 40, 0);
        
        poly_top_right[0][0] = Point(index2_right[0] - 40, marph_close.rows);
        poly_top_right[0][1] = Point(index2_right[0] + 50, marph_close.rows);
        poly_top_right[0][2] = Point(index2_right[0] + 50, 0);
        poly_top_right[0][3] = Point(index2_right[0] - 40, 0);


        const Point *ppt[1] = {poly[0]};
        const Point *ppt_under[1] = {poly_under[0]};
        const Point *ppt_top[1] = {poly_top[0]};

        const Point *ppt_right[1] = {poly_right[0]};
        const Point *ppt_under_right[1] = {poly_under_right[0]};
        const Point *ppt_top_right[1] = {poly_top_right[0]};
        int npt[] = {4};


        fillPoly(poly_mask, ppt, npt, 1, Scalar(255, 255, 255), 8);
        fillPoly(poly_mask_under, ppt_under, npt, 1, Scalar(255, 255, 255), 8);
        fillPoly(poly_mask_top, ppt_top, npt, 1, Scalar(255, 255, 255), 8);
        bitwise_and(marph_close, poly_mask, poly_roi);
        bitwise_and(marph_close, poly_mask_under, poly_roi_under);
        bitwise_and(marph_close, poly_mask_top, poly_roi_top);

        bitwise_or(poly_roi, poly_roi_under, poly_roi);
        bitwise_or(poly_roi, poly_roi_top, poly_roi);

        fillPoly(poly_mask_right, ppt_right, npt, 1, Scalar(255, 255, 255), 8);
        fillPoly(poly_mask_under_right, ppt_under_right, npt, 1, Scalar(255, 255, 255), 8);
        fillPoly(poly_mask_top_right, ppt_top_right, npt, 1, Scalar(255, 255, 255), 8);
        bitwise_and(marph_close, poly_mask_right, poly_roi_right);
        bitwise_and(marph_close, poly_mask_under_right, poly_roi_under_right);
        bitwise_and(marph_close, poly_mask_top_right, poly_roi_top_right);

        bitwise_or(poly_roi_right, poly_roi_under_right, poly_roi_right);
        bitwise_or(poly_roi_right, poly_roi_top_right, poly_roi_right);

        // ROI영역 시각화
        Mat poly_roi_clone = poly_roi.clone();
        line(poly_roi_clone, poly[0][0], poly[0][3], Scalar(255, 255, 255), 3);
        line(poly_roi_clone, poly[0][1], poly[0][2], Scalar(255, 255, 255), 3);
        line(poly_roi_clone, poly_under[0][0], poly_under[0][3], Scalar(255, 255, 255), 3);
        line(poly_roi_clone, poly_under[0][1], poly_under[0][2], Scalar(255, 255, 255), 3);
        line(poly_roi_clone, poly_top[0][0], poly_top[0][3], Scalar(255, 255, 255), 3);
        line(poly_roi_clone, poly_top[0][1], poly_top[0][2], Scalar(255, 255, 255), 3);
        // imshow("poly_roi_clone", poly_roi_clone);

        Mat poly_roi_clone_right = poly_roi_right.clone();
        line(poly_roi_clone_right, poly_right[0][0], poly_right[0][3], Scalar(255, 255, 255), 3);
        line(poly_roi_clone_right, poly_right[0][1], poly_right[0][2], Scalar(255, 255, 255), 3);
        line(poly_roi_clone_right, poly_under_right[0][0], poly_under_right[0][3], Scalar(255, 255, 255), 3);
        line(poly_roi_clone_right, poly_under_right[0][1], poly_under_right[0][2], Scalar(255, 255, 255), 3);
        line(poly_roi_clone_right, poly_top_right[0][0], poly_top_right[0][3], Scalar(255, 255, 255), 3);
        line(poly_roi_clone_right, poly_top_right[0][1], poly_top_right[0][2], Scalar(255, 255, 255), 3);
        // imshow("poly_roi_clone_right", poly_roi_clone_right);

        //원본중 총 ROI 영역
        marph_close = poly_roi + poly_roi_right;
        // imshow("left",poly_roi_left);
        // imshow("right",poly_roi_right);
        // imshow("marph_close", marph_close);

        // 슬라이딩 윈도우 <-- 연산량 많이 먹음
        // if (index[1] > 1100 || index[0] > 1100)
        // {
        //     index[1] = 1100;
        //     index[0] = 1100;
        // }
        // else if (index[0] < 100 || index[1] < 100)              
        // {
        //     index[0] = 100;
        //     index[1] = 100;
        // }
        Rect window1 = Rect(index[0] - 40, marph_close.rows - 60, 80, 60);
        Rect window2 = Rect(index_right[0] - 40, marph_close.rows - 60, 80, 60);
        Mat clone1 = marph_close.clone();
        Mat clone2 = marph_close.clone();
        vector<Point2f> all_left_points = slidingWindow(clone1, window1);
        vector<Point2f> all_right_points = slidingWindow(clone2, window2); 
        imshow("window1",clone1);
        imshow("window2",clone2);
        //2차항 회귀 or RANSAC
        Mat semi_final = marph_close.clone();
        Mat semi_final_ransac = marph_close.clone();
        cvtColor(semi_final, semi_final, CV_GRAY2BGR);
        cvtColor(semi_final_ransac, semi_final_ransac, CV_GRAY2BGR);
        float coor[all_left_points.size()] = {0};
        std::vector<double> srcX, srcY;
        std::vector<double> srcX1, srcY1;
        for (int i = 0; i < all_left_points.size(); i++)
        {
            srcX.push_back(all_left_points[i].x);
            srcY.push_back(all_left_points[i].y);
            srcX1.push_back(all_right_points[i].x);
            srcY1.push_back(all_right_points[i].y);
        }
        vector<Point2f> all_left_points_2nd;
        vector<Point2f> all_right_points_2nd;
        //--------------------------------------------------for 2차항 회귀
        double b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0;
        get2ndOrderRegression(&srcY, &srcX, &b0, &b1, &b2);
        get2ndOrderRegression(&srcY1, &srcX1, &b3, &b4, &b5);
        // cout << "b0 : "<< b0<< "b1 : "<< b1<< "b2 : "<< b2<< "b3 : "<< b3<< "b4 : "<< b4<< "b5 : "<< b5 << endl;
        for (int i = 0; i < all_left_points.size(); i++)
        {
            double X = all_left_points[i].y;
            double Y = b0 + b1 * X + b2 * X * X;
            double X1 = all_right_points[i].y;
            double Y1 = b3 + b4 * X1 + b5 * X1 * X1;
            all_left_points_2nd.push_back(Point2d(Y, X));
            all_right_points_2nd.push_back(Point2d(Y1, X1));
        }
        //보기 편하게 그림그리기
        for (int er = 0; er < all_left_points.size() - 1; er++)
        {
            line(semi_final, all_left_points_2nd[er], all_left_points_2nd[er + 1], Scalar(255, 255, 0), 10);//왼쪽 차선
            line(semi_final, all_right_points_2nd[er], all_right_points_2nd[er + 1], Scalar(255, 255, 0), 10);//오른쪽 차선
            coor[er] = ekf_fuction((all_right_points_2nd[er].x + all_left_points_2nd[er].x) / 2, er);
        }
        for (int i = 0; i < all_right_points.size() - 1; i++)
        {
            line(semi_final, all_left_points_2nd[i], all_right_points_2nd[i], Scalar(0, 0, 255), 5);//좌우 빨간색
            circle(semi_final, Point(coor[i], (all_right_points_2nd[i].y + all_right_points_2nd[i].y) / 2), 10, Scalar(0, 255, 255), 5);//중앙경로
            line(semi_final, Point2d(semi_final.cols / 2, semi_final.rows), Point2d(semi_final.cols / 2, (all_right_points_2nd[3].y + all_right_points_2nd[3].y) / 2), Scalar(255, 0, 255), 5);//중앙 보라색선
            difference_dist = coor[0] - semi_final.cols / 2;
            difference_dist1 = coor[1] - semi_final.cols / 2;
            difference_dist2 = coor[2] - semi_final.cols / 2;
            difference_dist3 = coor[3] - semi_final.cols / 2;
            difference_dist4 = coor[4] - semi_final.cols / 2;
            difference_dist5 = coor[5] - semi_final.cols / 2;
            difference_dist6 = coor[6] - semi_final.cols / 2;
            lane_msg.data = {difference_dist, difference_dist1, difference_dist2, difference_dist3, difference_dist4, difference_dist5, difference_dist6};
            pbuffer = gcvt(difference_dist, 4, buffer);
            pbuffer1 = gcvt(difference_dist1, 4, buffer1);
            pbuffer2 = gcvt(difference_dist2, 4, buffer2);
            pbuffer3 = gcvt(difference_dist3, 4, buffer3);
            pbuffer4 = gcvt(difference_dist4, 4, buffer4);
            pbuffer5 = gcvt(difference_dist5, 4, buffer5);
            pbuffer6 = gcvt(difference_dist6, 4, buffer6);
            // cout << "lane_msg.data[0] : " << lane_msg.data[0] << "  lane_msg.data[1] : " << lane_msg.data[1] << endl;
            // cout<<  "lane_msg.data[2] : " << lane_msg.data[2] << "  lane_msg.data[3] : " << lane_msg.data[3] << endl;
            // cout << "lane_msg.data[4] : " << lane_msg.data[4] << "  lane_msg.data[5] : " << lane_msg.data[5] << endl;
            // cout<<  "lane_msg.data[6] : " << lane_msg.data[6] << endl;
        }
        for (int j = 0; j < 7; j++)
        {
            line(semi_final, Point(coor[j], (all_right_points_2nd[j].y + all_right_points_2nd[j].y) / 2), Point2d(semi_final.cols / 2, (all_right_points_2nd[j].y + all_right_points_2nd[j].y) / 2), Scalar(0, 255, 100), 5);
        }
        // imshow("semi_final", semi_final);

        //--------------------------------------------------for RANSAC // 2차항회귀랑 거의 같아서 의미 없음
        // all_left_points_2nd.clear();
        // all_right_points_2nd.clear();
        // double b6 = 0, b7 = 0, b8 = 0, b9 = 0, b10 = 0, b11 = 0;
        // RANSAC123(srcY, srcX, &b6, &b7, &b8);
        // RANSAC123(srcY1, srcX1, &b9, &b10, &b11);
        // // cout << "b6 : "<< b6<< "b7 : "<< b7<< "b8 : "<< b8<< "b9 : "<< b9<< "b10 : "<< b10<< "b11 : "<< b11 << "\n" <<endl;

        // for (int i = 0; i < all_left_points.size(); i++)
        // {
        //     double X = all_left_points[i].y;
        //     double Y = b6 + b7 * X + b8 * X * X;
        //     double X1 = all_right_points[i].y;
        //     double Y1 = b9 + b10 * X1 + b11 * X1 * X1;
        //     all_left_points_2nd.push_back(Point2d(Y, X));
        //     all_right_points_2nd.push_back(Point2d(Y1, X1));
        // }

        //보기 편하게 그림그리기
        for (int er = 0; er < all_left_points.size() - 1; er++)
        {
            line(semi_final_ransac, all_left_points_2nd[er], all_left_points_2nd[er + 1], Scalar(255, 255, 0), 10);
            line(semi_final_ransac, all_right_points_2nd[er], all_right_points_2nd[er + 1], Scalar(255, 255, 0), 10);
            coor[er] = ekf_fuction((all_right_points_2nd[er].x + all_left_points_2nd[er].x) / 2, er);
        }

        for (int i = 0; i < all_right_points.size() - 1; i++)
        {
            line(semi_final_ransac, all_left_points_2nd[i], all_right_points_2nd[i], Scalar(0, 0, 255), 5);
            circle(semi_final_ransac, Point(coor[i], (all_right_points_2nd[i].y + all_right_points_2nd[i].y) / 2), 10, Scalar(0, 255, 255), 5);
            line(semi_final_ransac, Point2d(semi_final_ransac.cols / 2, semi_final_ransac.rows), Point2d(semi_final_ransac.cols / 2, semi_final_ransac.rows - 50), Scalar(100, 255, 255), 5);
            putText(semi_final_ransac, pbuffer, Point(500, 600), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0));
            difference_dist = coor[0] - semi_final_ransac.cols / 2;
            pbuffer = gcvt(difference_dist, 4, buffer);
            // lane_msg.data = difference_dist;

        }
        // imshow("semi_final_ransac", semi_final_ransac);
        //----------------------------------------------------


        // 원본에 대입
        Mat final;
        cv::warpPerspective(semi_final_ransac, final, warp_matrix_inv, cv::Size(semi_final.cols, semi_final.rows)); //버드아이뷰 역전환
        // cv::warpPerspective(semi_final_ransac, final, warp_matrix_inv, cv::Size(semi_final.cols, semi_final.rows)); //버드아이뷰 역전환
        // imshow("final",final);

        // Mat final11; //정지선과 차선인식 합친 것
        // final11 = final2 + final;
        // // imshow("final11",final11);

        Mat final4;
        addWeighted(src, 1, final, 1, 0, final4);
        imshow("final4",final4);
    

        waitKey();
        ros::spinOnce();
        loop_rate.sleep();

        if (waitKey(1) == 27)
		{
			break;
			printf("end");
		}
    }
    return 0;
}


Mat bird_eyes_view(Mat img)
{
    int width = img.cols;
    int height = img.rows;

    width = img.cols;
    height = img.rows;

    Point2f warp_src_point[4];
    Point2f warp_dst_point[4];
   
    //원본의 좌표(좌하단, 우하단, 좌상단, 우상단)
    

    warp_src_point[0].x = 0; // fix
	warp_src_point[0].y = 440; // fix
	warp_src_point[1].x = width - warp_src_point[0].x; // fix
	warp_src_point[1].y = warp_src_point[0].y; // fix
	warp_src_point[2].x = 140; 
	warp_src_point[2].y = 325;
	warp_src_point[3].x = width - warp_src_point[2].x;
	warp_src_point[3].y = warp_src_point[2].y;


    // -----------------------------------------------------------------------------------------------
	//목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단) _ modified
	warp_dst_point[0].x = 0;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - warp_dst_point[0].x;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 0;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - warp_dst_point[2].x;
	warp_dst_point[3].y = 0;

    warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
    invert(warp_matrix, warp_matrix_inv);
    // cout<<"warp_matrix_inv : "<<warp_matrix_inv<<endl;
    Mat dst;
    cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height)); //버드아이뷰 전환
    // imshow("dst111", dst);
    Mat dst1;
    cv::warpPerspective(dst, dst1, warp_matrix_inv, cv::Size()); //버드아이뷰 역전환
    // imshow("dst", dst1);

    return warp_matrix, warp_matrix_inv;
}

vector<Point2f> slidingWindow(Mat image, Rect window)
{
    vector<Point2f> points;
    const Size imgSize = image.size();
    bool shouldBreak = false;
    float currentX;
    double history[4] = {0};
    // circle(image,Point(window.x + window.width * 0.5f,image.rows - 30),3,Scalar(255,255,255),4);
    float diff = 0;
    int count = 1;
    if (window.x + window.width * 0.5f == 0)
    {
        currentX = image.cols * 0.8;
    }
    
    while (true)
    {
        if (window.x < 0)
            window.x = 0;
        if (window.x + window.width >= imgSize.width)
            window.x = imgSize.width - window.width - 1;

        if (shouldBreak)
            break;

        currentX = window.x + window.width * 0.5f;
        Mat roi = image(window); //Extract region of interest
        vector<Point> locations;
        findNonZero(roi, locations); //Get all non-black pixels. All are white in our case
        float avgX = 0.0f;
        for (int i = 0; i < locations.size(); ++i) //Calculate average X position
        {
            float x = locations[i].x;
            avgX += window.x + x;
        }

        if (locations.empty())
        {
            avgX = currentX + diff;
            // printf("diff : %lf\n", diff);
            if (avgX < 0)
            {
                avgX = 0;
            }
            else if (avgX > image.cols)
            {
                avgX = image.cols;
            }
        }
        else if (detect_stopline == true)
        {
            avgX = currentX;
        }
        else
        {
            avgX = avgX / locations.size();
            history[0] = history[1];
            history[1] = history[2];
            history[2] = history[3];
            history[3] = avgX;
            if (abs(history[3] - history[1]) < 100)
            {
                diff = history[3] - history[1];
            }
            rectangle(image, Rect(avgX - window.width / 2 * 0.8, window.y + window.height * 0.1, window.width * 0.8, window.height * 0.8), Scalar(255, 255, 255), 1);
        }
        Point point(avgX, window.y + window.height * 0.5f);
        rectangle(image, Rect(avgX - window.width / 2, window.y, window.width, window.height), Scalar(255, 255, 255), 1);
        points.push_back(point);
        //Move the window up
        window.y -= window.height;

        //For the uppermost position
        if (window.y < 0)
        {
            window.y = 0;
            shouldBreak = true;
        }

        //Move x position
        window.x += (point.x - currentX);

        //Make sure the window doesn't overflow, we get an error if we try to get data outside the matrix

    }
    // imshow("Sliding_Window", image);
    return points;
}

float ekf_fuction(float point, int number)
{
    static float A, H, Q, R, x[12] = {0}, P[12] = {0}, first_run = 1;
    if (first_run == 1)
    {
        A = 1;
        H = 1;
        Q = 1;
        R = 1; //이부분만지기

        x[number] = point;
        P[number] = 9;
        first_run = 0;
    }
    float xp = A * x[number];
    float Pp = A * P[number] * A + Q;

    float K = Pp * H * (1 / (Pp + R));

    x[number] = xp + K * (point - H * xp);
    P[number] = Pp - K * H * Pp;
    return x[number];
}

double square(double init, double x)
{
    return init + x * x;
}

double cubic(double init, double x)
{
    return init + x * x * x;
}

double forth_power(double init, double x)
{
    return init + x * x * x * x;
}

// Yi = b0 + b1*Xi + b2*Xi^2 + ei 으로, sum(e_i)를 최소화시키게끔

// regression 된 변수를 찾도록 정규방정식을 편미분해서 0 이 되는

// 각각의 bi를 구하면 다음과 같다.

bool get2ndOrderRegression(std::vector<double> *srcX, std::vector<double> *srcY, double *b0, double *b1, double *b2)
{
    double Y = std::accumulate(srcY->begin(), srcY->end(), 0.0);
    double X = std::accumulate(srcX->begin(), srcX->end(), 0.0);
    double X2 = std::accumulate(srcX->begin(), srcX->end(), 0.0, square);
    double X3 = std::accumulate(srcX->begin(), srcX->end(), 0.0, cubic);
    double X4 = std::accumulate(srcX->begin(), srcX->end(), 0.0, forth_power);
    double K = 0.0;
    double L = 0.0;
    int i = 0;
    int n = (int)srcX->size();
    for (i = 0; i < n; i++)
    {
        K += ((*srcY)[i] * (*srcX)[i] * (*srcX)[i]);
        L += ((*srcY)[i] * (*srcX)[i]);
    }
    double denominator = -n * X4 * X2 + X4 * X * X + X2 * X2 * X2 + X3 * X3 * n - 2 * X3 * X * X2;
    double b0p = -(Y * X4 * X2 - Y * X3 * X3 - X * L * X4 + X * X3 * K - X2 * X2 * K + X2 * X3 * L);
    double b1p = X * Y * X4 - X * K * X2 - L * n * X4 + X3 * n * K - Y * X2 * X3 + X2 * X2 * L;
    double b2p = -(K * n * X2 - K * X * X - X2 * X2 * Y - X3 * n * L + X3 * X * Y + X * X2 * L);
    *b0 = b0p / denominator;
    *b1 = b1p / denominator;
    *b2 = b2p / denominator;
    return true;
}


bool RANSAC123(std::vector<double> srcX, std::vector<double> srcY, double *b0, double *b1, double *b2)
{
    int n = (int)srcX.size();
    double king_b0[n] = {0};
    double king_b1[n] = {0};
    double king_b2[n] = {0};
    double count_ransac[n] = {0};
    int arr_size = 4;
    int num[arr_size] = {0};
    std::vector<double> srcXX;
    std::vector<double> srcYY;
    srand(time(0));
    for (int cnt = 0; cnt < n; cnt++)
    {
        srcXX.clear();
        srcYY.clear();
        double distance_value = 0;
        do
        {
            num[0] = rand() % n;
            num[1] = rand() % n;
            num[2] = rand() % n;
            num[3] = rand() % n;
        } while (num[0] == num[1] || num[1] == num[2] || num[2] == num[3] || num[3] == num[0] || num[0] == num[2] || num[1] == num[3]);
        for (int k = 0; k < arr_size; k++)
        {
            srcXX.push_back(srcX[num[k]]);
            srcYY.push_back(srcY[num[k]]);
        }
        get2ndOrderRegression(&srcXX, &srcYY, b0, b1, b2);
        for (int k = 0; k < srcX.size(); k++)
        {
            double X = srcX[k];
            double Y = *b0 + *b1 * X + *b2 * X * X;
            distance_value += pow(Y - srcY[k], 2);
        }
        count_ransac[cnt] = distance_value;
        king_b0[cnt] = *b0;
        king_b1[cnt] = *b1;
        king_b2[cnt] = *b2;
    }

    int min = count_ransac[0];
    int minIndex = 0;
    for (int i = 0; i < n; i++)
    {
        if (count_ransac[i] < min)
        {
            min = count_ransac[i];
            minIndex = i;
        }
    }

    int max = count_ransac[0];
    int maxIndex = 0;
    for (int i = 0; i < n; i++)
    {
        if (count_ransac[i] > max)
        {
            max = count_ransac[i];
            maxIndex = i;
        }
    }
    // printf("최소 오차값 : %lf\n", count_ransac[minIndex]);
    *b0 = king_b0[minIndex];
    *b1 = king_b1[minIndex];
    *b2 = king_b2[minIndex];
    return true;
}

void on_mouse(int event, int x, int y, int flags, void*)
{
    if (event == EVENT_LBUTTONDOWN){

        pt01d = Point(x,y);
        cout << "EVENT_LBUTTONDOWN: " << x << " , " << y << endl;
    }
}
