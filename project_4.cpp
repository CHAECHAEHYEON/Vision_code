#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h" // Int32형 메시지 주고받을 수 있게 함
#include <std_msgs/Float64.h> //Float64형 메시지 주고방르 수 있게 함
#include <sstream> //C++ 표준 라이브러리 일종
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h> // ros에서 opencv사용할 수 있게 함

using namespace cv;
using namespace std;

Mat bird_eyes_view(Mat img);
Mat warp_matrix;
Mat warp_matrix_inv;

Mat mask_filter(Mat img, int w, int h, int thresh);
vector<Point> sliding_window(Mat img);
vector<double> polyFitfit(vector<Point> px, int i, int degree);
void on_mouse(int event, int x, int y, int flags, void *userdata);
Point ptOld1;
int isStop = 100;
void mouse_callback(int event, int x, int y, int flags, void *param);
Mat img_color;
Mat img_hsv;
Mat OutputImage;
int H, S, V;
Mat frame1, frame2, left, left_gray;
bool callback = false;
//void imageCallback(const sensor_msgs::ImagesConstPtr &msg); //??????????????
Scalar RGB_mean(Mat img, int X, int width, int Y, int height);
int first_run = 1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stopline_publisher");
    ros::NodeHandle nh1;
    ros::Publisher stopline_pub = nh1.advertise<std_msgs::Float64>("stopline", 100); //float 메시지로 stopline이란 이름으로 ros전송
    image_transport::ImageTransport it(nh1);
    image_transport::Publisher image_raw_pub = it.advertise("camera/stopline/image_raw", 100);
    sensor_msgs::ImagePtr msg1; //카메라에서 얻은 정보를 송신하려면 이걸 작성해야 함.
    ros::Rate loop_rate(50);
    int count = 100; //초기 거리는 100으로 설정해 멈추지 않도록 한다.
    //VideoCapture cap1(0); //전방 정면캠
    VideoCapture cap1(2);

    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    while(ros::ok())
    {
        waitKey(1);
        //if (waitKey(cvRound(cap1.get(CAP_PROP_FPS) / 1000)) == 27) cap1 >> frame1;
        cap1 >> frame1;
        msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
        image_raw_pub.publish(msg1);
        if (frame1.empty())
        {
            cerr << "finish!\n" << endl;
        }
        imshow("publish_img", frame1);
        Mat img_resize;
        cv::resize(frame1, img_resize, cv::Size(640, 480), 0, 0);

        img_color = img_resize.clone();

        Mat img_warp, img_warp_clone;

        if (first_run == 1)
        {
            img_warp = bird_eyes_view(img_resize);
            first_run = 0;
        }
        else
        {
            cv::warpPerspective(img_resize, img_warp, warp_matrix, cv::Size());
        }
        img_warp_clone = img_warp.clone();

        rectangle(img_warp_clone, Rect(Point(img_warp.cols * 2/9, img_warp.rows * 2/3), Point(img_warp.cols * 5/18, img_warp.rows * 5/6)), Scalar(0, 0, 255), 1, 8, 0); // 색 평균 영역 설정
        rectangle(img_warp_clone, Rect(Point(img_warp.cols * 2/3, img_warp.rows * 2/3), Point(img_warp.cols * 13/18, img_warp.rows * 5/6)), Scalar(0, 0, 255), 1, 8, 0); // 두 번째 색 평균 영역 설정
        imshow("img_warp_color_mean", img_warp_clone);

        Scalar mean_color1 = RGB_mean(img_warp, img_warp.cols * 2/9, img_warp.cols * 1/18, img_warp.rows * 2/3, img_warp.rows * 1/6);
        Scalar mean_color2 = RGB_mean(img_warp, img_warp.cols * 2/3, img_warp.cols * 1/18, img_warp.rows * 2/3, img_warp.rows * 1/6);

        Mat img_binary;
        cv::inRange(img_warp, (mean_color1 + mean_color2) / 2 + cv::Scalar(10, 10, 10), cv::Scalar(255, 255, 255), img_binary);
        imshow("img_binary", img_binary);

        Mat img_integral;
        cv::integral(img_binary, img_integral); //x, y값 1씩 증가

        Mat img_mask;
        img_mask = mask_filter(img_integral, 5, 5, 173); // thresh값 확인 ################################################################

        Mat warp_inv;
        cv::warpPerspective(img_mask, warp_inv, warp_matrix_inv, cv::Size());

        Mat img_resize1;
        cv::resize(img_color, img_resize1, cv::Size(641, 481), 0, 0);

        Mat final;
        addWeighted(warp_inv, 1, img_resize1, 1, 0, final);

        if( isStop == 12)
        {
            count = 12;
            putText(final, "12M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 10)
        {
            count = 10;
            putText(final, "10M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 9)
        {
            count = 9;
            putText(final, "9M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 8)
        {
            count = 8;
            putText(final, "8M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 7)
        {
            count = 7;
            putText(final, "7M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 6)
        {
            count = 6;
            putText(final, "6M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 5)
        {
            count = 5;
            putText(final, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 4)
        {
            count = 4;
            putText(final, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 3)
        {
            count = 3;
            putText(final, "3M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }
        else if(isStop == 2)
        {
            count = 2;
            putText(final, "2M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
            //isStop = 100; //이번 과제에서는 마지막에 멈추기만 하면 도기 때문에 isStop을 100으로 초기화할 필요가 없음.
        }
        else if(isStop == 100)
        {
            count = 100;
            putText(final, "GO", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
        }

        imshow("final", final);
        std_msgs::Float64 msg;

        setMouseCallback("final", on_mouse);
        setMouseCallback("img_warp_color_mean", on_mouse);

        msg.data = count;
        ROS_INFO("%f", msg.data); //메시지를 화면에 표시
        stopline_pub.publish(msg); // msg데이터 전송 (이름 = stopline)
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

Scalar RGB_mean(Mat img, int X, int width, int Y, int height)
{
    Mat img_roi = img(Rect(Point(X, Y), Point(X + width, Y + height)));

    Scalar average = mean(img_roi);
    return average;
}

Mat bird_eyes_view(Mat img)
{
    int width = img.cols;
    int height = img.rows;

    width = img.cols;
    height = img.rows;
    Point2f warp_src_point[4];
    Point2f warp_dst_point[4];

    // //원본의 좌표
    // warp_src_point[0].x = 5;
    // warp_src_point[0].y = height;
    // warp_src_point[1].x = width - warp_src_point[0].x;
    // warp_src_point[1].y = warp_src_point[0].y;
    // warp_src_point[2].x = 280;
    // warp_src_point[2].y = 80;
    // warp_src_point[3].x = width - warp_src_point[2].x;
    // warp_src_point[3].y = warp_src_point[2].y;

    warp_src_point[0].x = 47;
    warp_src_point[0].y = height;
    warp_src_point[1].x = width - warp_src_point[0].x;
    warp_src_point[1].y = warp_src_point[0].y;
    warp_src_point[2].x = 266;
    warp_src_point[2].y = 280;
    warp_src_point[3].x = width - warp_src_point[2].x;
    warp_src_point[3].y = warp_src_point[2].y;

    // //목표 좌표
    // warp_dst_point[0].x = 150;
    // warp_dst_point[0].y = height;
    // warp_dst_point[1].x = width - warp_dst_point[0].x;
    // warp_dst_point[1].y = height;
    // warp_dst_point[2].x = 150;
    // warp_dst_point[2].y = 0;
    // warp_dst_point[3].x = width - warp_dst_point[2].x;
    // warp_dst_point[2].y = 0;

    warp_dst_point[0].x = 50;
    warp_dst_point[0].y = height;
    warp_dst_point[1].x = width - warp_dst_point[0].x;
    warp_dst_point[1].y = height;
    warp_dst_point[2].x = 50;
    warp_dst_point[2].y = 0;
    warp_dst_point[3].x = width - warp_dst_point[2].x;
    warp_dst_point[2].y = 0;

    warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
    invert(warp_matrix, warp_matrix_inv); //전치행렬 생성

    Mat dst;
    cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));

    return dst;
}

Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh)
{
    int height = img.rows;
    int width = img.cols;
    Mat img_maskfilter;
    img_maskfilter = Mat::zeros(height, width, CV_8UC1);
    Mat img_stop;
    img_stop = Mat::zeros(height, width, CV_8UC3);
    float mask[3];
    
    isStop = 100;
    uint *image = (uint *)img.data;
    uchar *score_data = (uchar *)img_maskfilter.data;
    int mask_w = _mask_w, mask_h = _mask_h;

    //초기값 설정
    int sy = 0;

    int roi_w = 130; //mask_filter를 실행하는 x값
    int histo = 0;

    for (int y = 20; y < height - 10; y++)
    {
        histo = 0;
        for ( int x = int (width / 2) - roi_w; x <= (width / 2) + roi_w; x++)
        {
            for (int i = 0; i < 3; i++)
            {
                sy = y + (2 * mask_h + 1) * ( i - 1);
                int ax, bx, cx, dx;
                int ay, by, cy, dy;
                dy = sy + mask_h;
                dx = x + mask_w;
                cy = sy - mask_h - 1;
                cx = x + mask_w;
                by = sy + mask_h;
                bx = x - mask_w - 1;
                ay = sy - mask_h - 1;
                ax = x - mask_h - 1;

                mask[i] = image[(dy)*width +dx] - image[(cy)*width + cx] - image[(by)*width + bx] +image[(ay)*width + ax];
            }

            float sum = ((mask[1] - mask[0]) + (mask[1] - mask[2])) / 2;

            if(sum > 6000)
            {
                score_data[width * y + x] = 255;
                histo++;
            }
        }

        line(img_stop, Point(int(width / 2) + roi_w, 20), Point(int(width / 2) + roi_w, height), Scalar(255, 255, 0), 5);
        line(img_stop,  Point(int(width / 2) - roi_w, 20), Point(int(width / 2) - roi_w, height), Scalar(255, 255, 0), 5);

        if(histo > thresh)
        {
            line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30); //정지선을 긋는다.

            if(y < 170)
            {
                cout << "stop Line distance : 10M\n" << endl;
                isStop = 10;
            }
            else if(y < 180)
            {
                cout << "stop Line distance : 9M\n" << endl;
                isStop = 9;
            }
            else if(y < 195)
            {
                cout << "stop Line distance : 8M\n" << endl;
                isStop = 8;
            }
            else if(y < 235)
            {
                cout << "stop Line distance : 7M\n" << endl;
                isStop = 7;
            }
            else if(y < 270)
            {
                cout << "stop Line distance : 6M\n" << endl;
                isStop = 6;
            }
            else if(y < 300)
            {
                cout << "stop Line distance : 5M\n" << endl;
                isStop = 5;
            }
            else if(y < 330)
            {
                cout << "stop Line distance : 4M\n" << endl;
                isStop = 4;
            }
            else if(y < 370)
            {
                cout << "stop Line distance : 3M\n" << endl;
                isStop = 3;
            }
            else if(y < 410)
            {
                cout << "stop Line distance : 2M\n" << endl;
                isStop = 2;
            }

            break;
        }
    }

    imshow("img_stop", img_stop);
    imshow("img_maskfilter", img_maskfilter);
    return img_stop;
}

void on_mouse(int event, int x, int y, int flags, void *)
{
    switch (event)
    {
        case EVENT_LBUTTONDOWN:
            ptOld1 = Point(x, y);
            cout << "EVENT_LBUTTONDOWN : " << x << ", " << y << endl;
            break;
        case EVENT_LBUTTONUP:
            cout << "EVENT_LBUTTONUP : " << x << ", " << y << endl;
            break;
    }
}

void mouse_callback(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDBLCLK)
    {
        Vec3b color_pixel = img_color.at<Vec3b>(y, x);

        Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel);

        Mat hsv_color;
        cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV);

        H = hsv_color.at<Vec3b>(0,0)[0];
        S = hsv_color.at<Vec3b>(0,0)[1];
        V = hsv_color.at<Vec3b>(0,0)[2];

        cout << "H= " << H << endl;
        cout << "S= " << S << endl;
        cout << "V= " << V << "\n" <<endl;

        H = H - 200;
        S = S - 50;
        V = V - 50;

        if(H < 0) H = 0;
        if(S < 0) S = 0;
        if(V < 0) V = 0;
    }
}
