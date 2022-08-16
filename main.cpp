#include "StereoVision.h"


using namespace cv;
using namespace std;

void parking_cb(const std_msgs::Bool::ConstPtr &msgs);

Mat img_color;
Mat img_color_2;
int H, S, V;

float baseline = 23;
float focal_pixels = 800; // size(1280, 720) 일때 focal_pixels
float alpha = 25;
float beta = 45; 	//alpha = 카메라 머리 숙인 각도, beta = erp 헤딩으로부터 카메라 각도


void mouse_callback(int event, int x, int y, int flags, void *param);
void mouse_callback_2(int event, int x, int y, int flags, void *param);
void on_mouse(int event, int x, int y, int flags, void *);

bool parking1 = false;
ros::Subscriber Parking_level1_sub;

void parking_cb(const std_msgs::Bool::ConstPtr &msgs)
{
	parking1 = msgs->data;
}

Point ptOld1;

float camera_values();



int main(int argc, char **argv) 
{   
    // VideoCapture capLeft(2);
    // VideoCapture capRight(4);
    VideoCapture capLeft("/home/kimtaehyeong/catkin_ws/src/taehyeong/kcity_parking1.mp4");
    VideoCapture capRight("/home/kimtaehyeong/catkin_ws/src/taehyeong/kcity_parking2.mp4");
    
    ros::init(argc, argv, "parking_publisher");
	ros::NodeHandle nh;
    ros::Publisher center_XZ_pub = nh.advertise<std_msgs::Float32MultiArray>("center_XZ", 10);
	Parking_level1_sub = nh.subscribe("Lidar_Stop", 10, parking_cb); //parkingstop

    ros::Rate loop_rate(10);
    Mat leftFrame, rightFrame;

    capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    StereoVision stereovision(baseline, focal_pixels);

        if (!capLeft.isOpened()) {
        cout << "Cannot Open Left Camera" << endl;
    }

    if (!capRight.isOpened()) {
        cout << "Cannot Open Right Camera" << endl;
    }

    Mat leftMask, rightMask;
    Mat leftResFrame, rightResFrame;

    Point leftCircle, rightCircle;

    Point2f top_XZ, mid_XZ, bottom_XZ;
        
    float left_array[6] = {0,0,0,0,0,0};
    float right_array[6] = {0,0,0,0,0,0};

    while (ros::ok) 
    {
        capLeft.read(leftFrame);
        capRight.read(rightFrame);
            
        resize(leftFrame, leftFrame, Size(1280, 720));
        resize(rightFrame, rightFrame, Size(1280, 720));
        
        // stereovision.line_symmetry(leftFrame, 0);
        // stereovision.line_symmetry(rightFrame, 1);

        // Calibration of the frames (640,480)
        // leftFrame = stereovision.undistortFrame(leftFrame);
        // rightFrame = stereovision.undistortFrame(rightFrame);
        
        // rectangle(level_one_src, Rect(0, 0, 1280, 100), Scalar(0, 0, 0), -1); //상단 for koreatech
        // rectangle(level_one_src, Rect(0, 0, 1280, 140), Scalar(0, 0, 0), -1); //for k-citys
        
        // Frames after applyting HSV-filter mask
        leftMask = stereovision.add_HSV_filter(leftFrame, 0);
        rightMask = stereovision.add_HSV_filter(rightFrame, 1);

        bitwise_and(leftFrame, leftFrame, leftResFrame, leftMask);
        bitwise_and(rightFrame, rightFrame, rightResFrame, rightMask);

        imshow("Left Mask", leftMask);
        imshow("Right Mask", rightMask);

        if(parking1 == true)
        {
            
            // Detect Circles - Hough Transforms can be used aswell or some neural network to do object detection
            // leftCircle = stereovision.find_ball(leftMask, leftMask);
            // rightCircle = stereovision.find_ball(rightMask, rightMask);

            float *ptr_left = stereovision.mass_center(leftMask, left_array, 0);
            float *ptr_right = stereovision.mass_center(rightMask, right_array, 1);

            // If no ball is detected in one of the cameras - show the text "tracking lost"
            if (!left_array[2] || !right_array[2])
            {
                putText(leftMask, "Tracking Lost!", { 75, 50 }, FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2);
                putText(rightMask, "Tracking Lost!", { 75, 75 }, FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2);
            } 
            else
            {
                bottom_XZ = stereovision.find_XZ({left_array[0],left_array[1]}, {right_array[0],right_array[1]}, leftFrame, rightFrame, alpha, beta);
                mid_XZ = stereovision.find_XZ({left_array[2],left_array[3]}, {right_array[2],right_array[3]}, leftFrame, rightFrame, alpha, beta);
                top_XZ = stereovision.find_XZ({left_array[4],left_array[5]}, {right_array[4],right_array[5]}, leftFrame, rightFrame, alpha, beta);

                putText(leftMask, "Tracking!", { 75, 50 }, FONT_HERSHEY_SIMPLEX, 0.7, (50,170,50), 2);
                putText(rightMask, "Tracking!", { 75, 75 }, FONT_HERSHEY_SIMPLEX, 0.7, (50,170,50), 2);
            }
            cout << "bottom_XZ : " << bottom_XZ << endl;
            cout << "mid_XZ : " << mid_XZ << endl;
            cout << "top_XZ : " << top_XZ << endl << endl;

            float arr[6];
            arr[0]= bottom_XZ.x/100.00;
            arr[1]= bottom_XZ.y/100.00;
            arr[2]= mid_XZ.x/100.00;
            arr[3]= mid_XZ.y/100.00;
            arr[4]= top_XZ.x/100.00;
            arr[5]= top_XZ.y/100.00;

            std_msgs::Float32MultiArray center_XZ_msg;
            center_XZ_msg.data.clear();
            for(int i=0; i<6; i++)
            {
                center_XZ_msg.data.push_back(arr[i]);
                // printf("arr[] : %f\n", arr[i]);
            }
            center_XZ_pub.publish(center_XZ_msg);




            // imshow("Left Frame", leftFrame);
            // imshow("Right Frame", rightFrame);

            // img_color = leftFrame.clone();
            // img_color_2 = rightFrame.clone();
            // setMouseCallback("Left Frame", mouse_callback);
            // setMouseCallback("Right Frame", mouse_callback_2);
            // setMouseCallback("Left Frame",on_mouse);
            // setMouseCallback("Right Frame",on_mouse);
        }
        else if(parking1 == false)
        {
            cout << "sub fail" << endl;
        }

        if (waitKey(1) == 27)
        {
            break;
            printf("end");
            
        }
        // capLeft.release();
        // capRight.release();
        ros::spinOnce();
	    loop_rate.sleep();
    }
    return 0;
}

void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat hsv_color = Mat(1, 1, CV_8UC3, color_pixel);


		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "left H= " << H << endl;
		cout << "left S= " << S << endl;
		cout << "left V = " << V << "\n"
			 << endl;

	}
}
void mouse_callback_2(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		Vec3b color_pixel = img_color_2.at<Vec3b>(y, x);

		Mat hsv_color = Mat(1, 1, CV_8UC3, color_pixel);


		H = hsv_color.at<Vec3b>(0, 0)[0];
		S = hsv_color.at<Vec3b>(0, 0)[1];
		V = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "right H= " << H << endl;
		cout << "right S= " << S << endl;
		cout << "right V = " << V << "\n"
			 << endl;

	}
}

float camera_values()
{
    cv::Mat K(3,3,CV_64F);

    K = (Mat_<_Float64>(3, 3) << 538.39128993937,   0.,   308.049009633327, 0.,  539.777910345317,  248.065244763797, 0., 0., 1.);

    cout << "K : " << K << endl;


    cv::Size imageSize(640,480);
    double apertureWidth = 0;
    double apertureHeight = 0;
    double fieldOfViewX;
    double fieldOfViewY;
    double focalLength2;
    cv::Point2d principalPoint;
    double aspectRatio;
    cv::calibrationMatrixValues(K, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength2, principalPoint, aspectRatio);

    
    cout << fieldOfViewX << endl;

    return (float)focalLength2;
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
