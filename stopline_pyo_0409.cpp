// K-BUB team, PyoSH retouched
// preprocessing 
// 0409 update : 1. RGB_mean -> IMG_mean


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

// -----------VARIABLES--------------------------------------------------------------
Mat warp_matrix_inv;
Mat warp_matrix;
Mat img_color;
Mat frame1, frame2;

Mat imgUndistort, imgUnwarp;
Mat imgHLS_L;
Mat imgLAB_B;
Mat combineOut;


vector<Point> sliding_window(Mat img);
vector<double> polyFit(vector<Point> px, int i, int degree);
Scalar RGB_mean(Mat img, int X, int width, int Y, int height);

Point2f warp_SRC_ROI[4];
Point2f warp_DST_ROI[4];
Point ptOld1;
int isStop = 100;
int H, S, V;
bool callback = false;	
int first_run = 1; // KEY for compute speed!!!

// -----------FUNCTIONS--------------------------------------------------------------
Mat make_zeros(Mat img)
{
    return Mat::zeros(img.rows, img.cols, img.type());
}

Scalar IMG_mean(Mat img, int X, int width, int Y, int height)
{
	Mat img_roi = img(Rect(Point(Y, X), Point(Y + height, X + width)));
	// imshow("img_roi",img_roi);
	Scalar average = mean(img_roi);
	// std::cout << average << std::endl;
	return average;
}

Mat make_ones(Mat img)
{
    return Mat::ones(img.rows, img.cols, img.type());
}

Mat filterImg(Mat imgUnwarp, int toColorChannel, int mode)
{
    /*
    channel mode definition.
        0 : Hue
        1 : Lightness
        2 : Saturation
        
        hue max : 179, l and s max : 255
    */
    Mat imgConverted;
    // Mat imgOUT = Mat(720, 1280, CV_8UC3);
    // Mat imgOUT = Mat::zeros(720, 1280, CV_8UC3);
    Mat imgOUT = Mat::zeros(480, 640, CV_8UC1);

    /* 1. convert color channel from BGR to HLS or LAB. */
    if (toColorChannel == 0)
    {
        cvtColor(imgUnwarp, imgConverted, COLOR_BGR2HLS);
        // imshow("Hls", imgConverted);
    }
    else if (toColorChannel == 1)
    {
        cvtColor(imgUnwarp, imgConverted, COLOR_BGR2Lab);
        // imshow("Lab", imgConverted);
    }

    /* 2. pixel pointer variable setting.
    help -->  https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
    */
    uint8_t *pixelPtr = (uint8_t *)imgConverted.data;
    int cn = imgConverted.channels();

    switch (mode)
    {
    case 0:
        // set H space Only  // set L space Only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 0];
            }
        }
        break;
    case 1:
        // set L space Only  // set A space Only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 1];
            }
        }
        break;

    case 2:
        // set S space Only  // set B space Only
        for (int i = 0; i < imgConverted.rows; i++)
        {
            for (int j = 0; j < imgConverted.cols; j++)
            {
                imgOUT.at<uint8_t>(i, j) = pixelPtr[i * imgConverted.cols * cn + j * cn + 2];
            }
        }
        break;

    default:
        break;
    }

    /* 
    COLORMAP for grayscale.     
    ref : https://learnopencv.com/applycolormap-for-pseudocoloring-in-opencv-c-python/
     applyColorMap(imgOUT, imgOUT, COLORMAP_BONE);
     컬러맵을 쓰니깐 채널이 1개에서 3개가 되버렸다. 이 메쏘드는 안쓰는게 맞는듯.
     컬러맵은 그레이 스케일에 대해서, 색상을 역으로 부여하는 꼴이 되기 때문에 채널이 3개로 늘어난 듯.
    */

    return imgOUT;
}

Mat combine_both_img(Mat hls, Mat lab)
{
    Mat imgOut;

    imgOut = make_zeros(hls);
    for (int i = 0; i < imgOut.rows; i++)
    {
        for (int j = 0; j < imgOut.cols; j++)
        {
            if (hls.at<uint8_t>(i, j) == 1 || lab.at<uint8_t>(i, j) == 1)
            {
                imgOut.at<uint8_t>(i, j) = 1;
            }
        }
    }
    return imgOut;
}

Mat normalize_HLS_L(Mat unWarp)
{
    /* normalizing L color channel pixel from hls img. */
    Mat imgHLS_L, imgNormal;
    double minVal, maxVal;
    Point minLoc, maxLoc;

    Scalar mean_color1 = IMG_mean(unWarp, unWarp.rows * 3 / 5, unWarp.rows * 1/ 5, unWarp.cols * 1 / 7, unWarp.cols / 7);
	Scalar mean_color2 = IMG_mean(unWarp, unWarp.rows * 3 / 5, unWarp.rows * 1/ 5, unWarp.cols * 4 / 7, unWarp.cols * 1 / 7);
    Scalar mean_color3 = IMG_mean(unWarp, unWarp.rows * 1 / 7 ,unWarp.rows * 1/ 7 , unWarp.cols * 1/ 7, unWarp.cols * 1/7);

    int mean_thres = (mean_color1[1] + mean_color2[1]+ mean_color3[1]) /3 +10;
    
    int lowThres; // origin : 200

    if (mean_thres < 157){
        lowThres = 165; 
    }
    else{
        lowThres = mean_thres;
    }

    printf("%d",lowThres);
    
    // get a single channel img(filtered one.)
    imgHLS_L = filterImg(unWarp, 0, 1);
    // imshow("filterimg",imgHLS_L);
    // get max, min value of the matrix.
    minMaxLoc(imgHLS_L, &minVal, &maxVal, &minLoc, &maxLoc);

    // make normalized img.
    imgNormal = (255 / maxVal)* imgHLS_L;
    
    // imshow("normalimg",imgNormal);
    // apply threshold for L channel.
    Mat imgOut = make_zeros(imgNormal);
    // Mat imgOut2 = make_zeros(imgNormal);

    threshold(imgNormal, imgOut, lowThres, 255, THRESH_BINARY);
    // threshold(imgNormal, imgOut2,0,255,THRESH_BINARY|THRESH_TRIANGLE);
    // 적응형 이진화
    
    return imgOut;
}

Mat normalize_LAB_B(Mat unWarp)
{
    /* normalizing B color channel pixel from LAB img. */
    Mat imgLAB_B, imgNormal;
    double minVal, maxVal;
    Point minLoc, maxLoc;
    int yellowCrit = 190;
    int lowThres = 10; // origin: 190

    // get a single channel img(filtered one.)
    imgLAB_B = filterImg(unWarp, 1, 2);

    // get max, min value of the matrix.
    minMaxLoc(imgLAB_B, &minVal, &maxVal, &minLoc, &maxLoc);

    // (conditional) make normalized img.
    // B channel means a range from blue(0) to yellow(255).
    // So, the bigger values, it becomes close to yellow color.(yellow lane)
    if (maxVal > yellowCrit)
    {
        imgNormal = (255 / maxVal) * imgLAB_B;
    }
    else
    {
        imgNormal = imgLAB_B;
    }
    // imshow("imgNormal", imgNormal);

    // apply threshold for L channel.
    Mat imgOut = Mat::zeros(imgNormal.rows, imgNormal.cols, imgNormal.type());
    threshold(imgNormal, imgOut, 200, 255, THRESH_BINARY);

    return imgOut;
}

Mat bird_eyes_view(Mat img)
{
	int width = img.cols;
	int height = img.rows;

	width = img.cols;
	height = img.rows;
	Point2f warp_src_point[4];
	Point2f warp_dst_point[4];
	// -----------------------------------------------------------------------------------------------

	// //원본의 좌표(좌하단, 우하단, 좌상단, 우상단) <ORIGIN>
	warp_src_point[0].x = 5;
	warp_src_point[0].y = height;
	warp_src_point[1].x = width - warp_src_point[0].x;
	warp_src_point[1].y = warp_src_point[0].y;
	warp_src_point[2].x = 280;
	warp_src_point[2].y = 80;
	warp_src_point[3].x = width - warp_src_point[2].x;
	warp_src_point[3].y = warp_src_point[2].y;

	//원본의 좌표(좌하단, 우하단, 좌상단, 우상단) _ need to be retouched. <EXP>
	// warp_src_point[0].x = 8; // fix
	// warp_src_point[0].y = height; // fix
	// warp_src_point[1].x = width - warp_src_point[0].x; // fix
	// warp_src_point[1].y = warp_src_point[0].y; // fix
	// warp_src_point[2].x = 274; 
	// warp_src_point[2].y = 279;
	// warp_src_point[3].x = width - warp_src_point[2].x;
	// warp_src_point[3].y = warp_src_point[2].y;


    // -----------------------------------------------------------------------------------------------
	//목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단) _ modified
	// warp_dst_point[0].x = 10;
	// warp_dst_point[0].y = height;
	// warp_dst_point[1].x = width - warp_dst_point[0].x;
	// warp_dst_point[1].y = height;
	// warp_dst_point[2].x = 150;
	// warp_dst_point[2].y = 0;
	// warp_dst_point[3].x = width - warp_dst_point[2].x;
	// warp_dst_point[3].y = 0;

	// //목표이미지의 좌표(좌하단, 우하단, 좌상단, 우상단) _ origin
	warp_dst_point[0].x = 150;
	warp_dst_point[0].y = height;
	warp_dst_point[1].x = width - warp_dst_point[0].x;
	warp_dst_point[1].y = height;
	warp_dst_point[2].x = 150;
	warp_dst_point[2].y = 0;
	warp_dst_point[3].x = width - warp_dst_point[2].x;
	warp_dst_point[3].y = 0;
	
	warp_matrix = cv::getPerspectiveTransform(warp_src_point, warp_dst_point);
	invert(warp_matrix, warp_matrix_inv);

	//------------------------copying ROI : local to global---------------------------------------
	warp_SRC_ROI[0].x = warp_src_point[0].x;
	warp_SRC_ROI[0].y = warp_src_point[0].y;
	warp_SRC_ROI[1].x = warp_src_point[1].x;
	warp_SRC_ROI[1].y = warp_src_point[1].y;
	warp_SRC_ROI[2].x = warp_src_point[2].x;
	warp_SRC_ROI[2].y = warp_src_point[2].y;
	warp_SRC_ROI[3].x = warp_src_point[3].x;
	warp_SRC_ROI[3].y = warp_src_point[3].y;
	
	warp_DST_ROI[0].x = warp_dst_point[0].x;
	warp_DST_ROI[0].y = warp_dst_point[0].y;
	warp_DST_ROI[1].x = warp_dst_point[1].x;
	warp_DST_ROI[1].y = warp_dst_point[1].y;
	warp_DST_ROI[2].x = warp_dst_point[2].x;
	warp_DST_ROI[2].y = warp_dst_point[2].y;
	warp_DST_ROI[3].x = warp_dst_point[3].x;
	warp_DST_ROI[3].y = warp_dst_point[3].y;
	//-------------------------------------------------------------------------------------------------

	Mat dst;
	cv::warpPerspective(img, dst, warp_matrix, cv::Size(width, height));
	// imshow("dst", dst);  //top view

	return dst;
}

Mat mask_filter(Mat img, int _mask_w, int _mask_h, int thresh)
{
	//2020 1st member made.
	int height = img.rows;
	int width = img.cols;
	Mat img_maskfilter;
	img_maskfilter = Mat::zeros(height, width, CV_8UC1); // zerolize filter
	Mat img_stop;
	img_stop = Mat::zeros(height, width, CV_8UC3); // zerolize stop_img
	float mask[3];
	int sx = 0;
	isStop = 100;

	uint *image = (uint *)img.data;
	uchar *score_data = (uchar *)img_maskfilter.data;
	int mask_w = _mask_w, mask_h = _mask_h;

	int sy = 0;

	int roi_w = 80; // 80         check!!!!!!!!!!!!!!! 꼭 체크!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@
	int histo = 0;

	for (int y = 20; y < height - 15; y++)
	{
		histo = 0;
		for (int x = int(width / 2) - roi_w; x <= int(width / 2) + roi_w; x++)
		{
			for (int i = 0; i < 3; i++)
			{
				sy = y + (2 * mask_h + 1) * (i - 1);
				int dx, cx, bx, ax;
				int dy, cy, by, ay;
				dy = sy + mask_h;
				dx = x + mask_w;
				cy = sy - mask_h - 1;
				cx = x + mask_w;
				by = sy + mask_h;
				bx = x - mask_w - 1;
				ay = sy - mask_h - 1;
				ax = x - mask_w - 1;
				mask[i] = image[(dy)*width + dx] - image[(cy)*width + cx] - image[(by)*width + bx] + image[(ay)*width + ax];
			}
			float sum = ((mask[1] - mask[0]) + (mask[1] - mask[2])) / 2;
			// cout<<"sum"<<sum<<endl;
			if (sum > 6000) // 10000  6000(5*5)  ---------------if(mask_h*mask_w == 40) -> sum > 100000----------------
			{
				score_data[width * y + x] = 255;
				histo++;
			}
		}
		line(img_stop, Point(int(width / 2) + roi_w, 20), Point(int(width / 2) + roi_w, height), Scalar(255, 255, 0), 5);
		line(img_stop, Point(int(width / 2) - roi_w, 20), Point(int(width / 2) - roi_w, height), Scalar(255, 255, 0), 5);

		if (histo > thresh)
		{
			line(img_stop, Point(int(width / 2) - roi_w, y), Point(int(width / 2) + roi_w, y), Scalar(255, 0, 0), 30);
			// cout << "histo : " << histo << endl;
			// cout<<"y"<<y<<endl;
			if (y < 225)
			{
				cout << "stop line distance : 10M\n"
					 << endl;
				isStop = 10;
			}
			else if (y < 245)
			{
				cout << "stop line distance : 9M\n"
					 << endl;
				isStop = 9;
			}
			else if (y < 275)
			{
				cout << "stop line distance : 8M\n"
					 << endl;
				isStop = 8;
			}
			else if (y < 305)
			{
				cout << "stop line distance : 7M\n"
					 << endl;
				isStop = 7;
			}
			else if (y < 335)
			{
				cout << "stop line distance : 6M\n"
					 << endl;
				isStop = 6;
			}
			else if (y < 370)
			{
				cout << "stop line distance : 5M\n" ///12m
					 << endl;
				isStop = 5;
			}
			else if (y < 410)
			{
				cout << "stop line distance : 4M\n"
					 << endl;
				isStop = 4;
			}
			else if (y < 450)
			{
				cout << "stop line distance : 3M\n"
					 << endl;
				isStop = 3;
			}
			else if (y < 500)
			{
				cout << "stop line distance : 2M\n"
					 << endl;
				isStop = 2;
			}
			
			break;
		}
	}
	imshow("img_stop", img_stop);
	return img_stop;
}

Mat preprocessing(Mat img, Mat img_warp){
	// cerr << "1-----------------"<<endl;
	Mat img_warp_clone,img_warp_clone_hls,img_show;

	Mat white = Mat::zeros(480, 640, img_warp.type());
    rectangle(white, Rect(0, 450, 30, 30), Scalar(250, 250, 250), -1);
	//imshow("11", white);
	img_warp_clone = img_warp.clone();
    img_show = img_warp.clone();
	img_warp_clone_hls = img_warp_clone.clone();
	
	Mat img_warp_clone_hls_white= img_warp_clone_hls + white;
	// imshow("11", img_warp_clone_hls_white);
		
	Mat img_OUT_HLS;
	img_OUT_HLS= normalize_HLS_L(img_warp_clone_hls_white);
	imshow("img_out_HLS",img_OUT_HLS);
	
	Mat BLURRED;
	medianBlur(img_OUT_HLS, BLURRED,3);
	imshow("bluur~~",BLURRED);
	
	Mat OPEN_CLOSE;	
	cv::morphologyEx(BLURRED,OPEN_CLOSE,MORPH_CLOSE,Mat());
	cv::morphologyEx(OPEN_CLOSE,OPEN_CLOSE,MORPH_OPEN,Mat());
	// imshow("OPEN~~",OPEN_CLOSE);

	// rectangle(img, Rect(Point(img_warp.cols * 1 / 5, img_warp.rows * 3 / 5), Point(img_warp.cols * 2 / 5, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0); // why one?
	// rectangle(img, Rect(Point(img_warp.cols * 3 / 5, img_warp.rows * 3 / 5), Point(img_warp.cols * 4 / 5, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0);
	rectangle(img_show, Rect(Point(img_warp.cols * 1 / 7, img_warp.rows * 3 / 5), Point(img_warp.cols * 2 / 7, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0); // why one?
	rectangle(img_show, Rect(Point(img_warp.cols * 4 / 7, img_warp.rows * 3 / 5), Point(img_warp.cols * 5 / 7, img_warp.rows * 4 / 5)), Scalar(0, 0, 255), 1, 8, 0);
    rectangle(img_show, Rect(Point(img_warp.cols * 1 / 2- 30 , img_warp.rows * 1 / 7), Point(img_warp.cols * 1/2 +30 , img_warp.rows * 2/7)), Scalar(0, 0, 255), 1, 8, 0);
	// unWarp.rows * 1 / 7 ,unWarp.rows * 1/ 7 , unWarp.cols * 1/ 2, unWarp.cols * 1/5
    
    imshow("img_warp_color_mean", img_show); // 색 평균 영역 설정

	// // ----------------------------------------------------INTEGRATE_EXP----------------------------------------------------
	// Scalar mean_color1 = IMG_mean(img_warp, img_warp.rows * 3 / 5, img_warp.rows * 1/ 5, img_warp.cols * 1 / 7, img_warp.cols / 7);
	// Scalar mean_color2 = IMG_mean(img_warp, img_warp.rows * 3 / 5, img_warp.rows * 1/ 5, img_warp.cols * 4 / 7, img_warp.cols * 1 / 7);
	// cout << "<BGR_ mean1 color> " << mean_color1 << "  <BGR_ mean2 color>" << mean_color2 << endl;

	Mat img_binary;
	// cv::inRange(img_warp, (mean_color1 + mean_color2) / 2 + cv::Scalar(10, 10, 10), cv::Scalar(255, 255, 255), img_binary);
	img_binary = OPEN_CLOSE;
	imshow("img_binary", img_binary);

	Mat img_integral;
	cv::integral(img_binary, img_integral);
	// imshow("img_integral",img_integral);

	Mat img_mask;
	img_mask = mask_filter(img_integral, 5, 5, 120); // (5,5,85)   (5,8,120)      check!!!!!!!!!!!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	// imshow("img_mask", img_mask);   //mask filter 이거 체크!!!!!!!!!!!!!

	Mat warp_inv;
	cv::warpPerspective(img_mask, warp_inv, warp_matrix_inv, cv::Size());
	imshow("warp_inv",warp_inv);

	return warp_inv;
}

void DISPLAY_meter(Mat final, int count){
			if (isStop == 12)
		{
			count = 12;
			putText(final, "12M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 10)
		{
			count = 10;
			putText(final, "10M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 9)
		{
			count = 9;
			putText(final, "9M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}
		else if (isStop == 8)
		{
			count = 8;
			putText(final, "8M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}
		else if (isStop == 7)
		{
			count = 7;
			putText(final, "7M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}

		else if (isStop == 6)
		{
			count = 6;
			putText(final, "6M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}
		else if (isStop == 5)
		{
			count = 5;
			putText(final, "5M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop=100;
		}
		else if (isStop == 4)
		{
			count = 4;
			putText(final, "4M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop = 100;
		}
		else if (isStop == 3)
		{
			count = 3;
			putText(final, "3M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			// isStop = 100;
		}
		else if (isStop == 2)
		{
			count = 2;
			putText(final, "2M", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
			isStop = 100;
		}

		else if (isStop == 100)
		{
			count = 100;
			putText(final, "Go", Point(500, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 255));
		}


}

void on_mouse(int event, int x, int y, int flags, void *)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		ptOld1 = Point(x, y);
		cout << "EVENT_LBUTTONDOWN: " << x << ", " << y << endl;
		break;
	case EVENT_LBUTTONUP:
		cout << "EVENT_LBUTTONUP: " << x << ", " << y << endl;
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

		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "H= " << H << endl;
		cout << "S= " << S << endl;
		cout << "V = " << V << "\n"
			 << endl;

		H = H - 200;
		S = S - 50;
		V = V - 50;

		if (H < 0)
			H = 0;

		if (S < 0)
			S = 0;

		if (V < 0)
			V = 0;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stopline_publisher");
	// ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::Publisher stopline_pub = nh1.advertise<std_msgs::Float64>("stopline", 100); //int형 메시지
	image_transport::ImageTransport it(nh1);
	image_transport::Publisher image_raw_pub = it.advertise("/camera/stopline/image_raw", 100); //카메라에서 이미지 읽어서 송신
	sensor_msgs::ImagePtr msg1;
	ros::Rate loop_rate(50);
	int count = 100;

	TickMeter tm;

	// VideoCapture cap1(0); //전방 정면캠 
	VideoCapture cap1("/home/kroad/stopline_video/0329_6.mp4"); // INTEGRATED SUBJECT VER	

	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	while (ros::ok())
	{
        tm.start();
		
		cap1 >> frame1;
		msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
		image_raw_pub.publish(msg1);
		if (!cap1.isOpened())
		{
			cerr << "finish!\n"
				 << endl;
		}
		// imshow("frame1", frame1);

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

		// circle(img_resize, warp_SRC_ROI[0], 5, (0,0,255), -1);
		// circle(img_resize, warp_SRC_ROI[1], 5, (0,0,255), -1);
		// circle(img_resize, warp_SRC_ROI[2], 5, (0,0,255), -1);
		// circle(img_resize, warp_SRC_ROI[3], 5, (0,0,255), -1);
		imshow("img_resize", img_resize);

		Mat img_resize1;
		cv::resize(img_color, img_resize1, cv::Size(641, 481), 0, 0);
		// imshow("img_resize1",img_resize1);

		Mat final;
		addWeighted(preprocessing(img_warp_clone, img_warp), 1, img_resize1, 1, 0, final);

		DISPLAY_meter(final, count);
		imshow("final", final);
		
		// setMouseCallback("img_warp",on_mouse);

		std_msgs::Float64 msg;
		msg.data = count;
		// ROS_INFO("%f", msg.data); // data 메시지를표시한다
		stopline_pub.publish(msg);

        tm.stop();

        cerr<< "-----Compute Time per frame  " << tm.getTimeMilli()<<"ms." <<endl;

        tm.reset();

		waitKey(1);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// CLASS-alpha ---------------------
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "mapping");
//     CMapping mapping;
//     if (mapping.init())
//     {
//         ROS_FATAL("Mapping initialization failed");
//         return -1;
//     }

//     mapping.publish();

//     return 0;
// }
