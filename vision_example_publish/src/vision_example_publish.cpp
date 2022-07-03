#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
Mat frame1;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_raw_pub = it.advertise("camera/rgb/image_raw", 100); //카메라에서 이미지 읽어서 송신
	sensor_msgs::ImagePtr msg;
	ros::Rate loop_rate(50);
	VideoCapture cap(0); //전방 정면캠

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

	while (ros::ok())
	{	
		waitKey(1);
		cap >> frame1;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
		
		image_raw_pub.publish(msg);

		if (!cap.isOpened())
		{
			cerr << "finish!\n"
				 << endl;
		}

		imshow("publish_image", frame1);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

