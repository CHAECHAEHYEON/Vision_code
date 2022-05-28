// K-BUB team, PyoSH retouched
// preprocessing 
// 0409 update : 1. RGB_mean -> IMG_mean


#include "Vision_func.h"
#include "Vision_func.cpp"

using namespace cv;
using namespace std;

WeAreVision WAV;

// --------MAIN-----------------------------------------------------------------------
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

	VideoCapture cap1(2); //전방 정면캠 
	// VideoCapture cap1("/home/usera/stopline_vedio/good/0409_1.mp4"); // INTEGRATED SUBJECT VER	

	cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	while (ros::ok())
	{
        tm.start();
		
		cap1 >> WAV.frame1;
		msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", WAV.frame1).toImageMsg();
		image_raw_pub.publish(msg1);
		if (!cap1.isOpened())
		{
			cerr << "finish!\n"
				 << endl;
		}
		// imshow("frame1", frame1);

		Mat img_resize;
		cv::resize(WAV.frame1, img_resize, cv::Size(640, 480), 0, 0);

		WAV.img_color = img_resize.clone();

		Mat img_warp, img_warp_clone;
		if (WAV.first_run == 1)
		{
			img_warp = WAV.bird_eyes_view(img_resize);
			WAV.first_run = 0;
		}
		else
		{
			cv::warpPerspective(img_resize, img_warp, WAV.warp_matrix, cv::Size());
		}
		img_warp_clone = img_warp.clone();

		// circle(img_resize, warp_SRC_ROI[0], 5, (0,0,255), -1);
		// circle(img_resize, warp_SRC_ROI[1], 5, (0,0,255), -1);
		// circle(img_resize, warp_SRC_ROI[2], 5, (0,0,255), -1);
		// circle(img_resize, warp_SRC_ROI[3], 5, (0,0,255), -1);
		imshow("img_resize", img_resize);

		Mat img_resize1;
		cv::resize(WAV.img_color, img_resize1, cv::Size(641, 481), 0, 0);
		// imshow("img_resize1",img_resize1);

		Mat final;
		addWeighted(WAV.STOP_preprocessing(img_warp_clone, img_warp), 1, img_resize1, 1, 0, final);

		count = WAV.DISPLAY_meter(final, count);
		imshow("final", final);

		// cout << 
		
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
