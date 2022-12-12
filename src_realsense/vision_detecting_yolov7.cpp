#include "opencv2/opencv.hpp"
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"
#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/Detection2D.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"

#define CAM_NUM 0

using namespace std;
using namespace cv::ml;
using namespace cv;

Mat frame1;
Mat sub_image1;

bool callback = false;

int detect_length = 0;
bool detect = false;

float Max_Area = 0;

bool loop_flag = false;
float Area = 0;

bool pub_flag = false;



//###########################KORUS ROS#################################

ros::Subscriber detection_sub;
ros::Subscriber detect_length_sub;
ros::Subscriber detect_flag_sub;

//###################33vision_msgs ROS################################





void imageCallback(const sensor_msgs::ImageConstPtr &msg);

void detectionlength_callback(const std_msgs::Int16::ConstPtr &msg)
{
	detect_length = msg->data;

	loop_flag = true;
}

void detect_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
	detect = msg -> data;
}

void detections_callback(const vision_msgs::Detection2DArray::ConstPtr &detections2dArray)
{
    int isDone = false;
    int detectionsCount = 0;

   	ROS_INFO("---- ");
    ROS_INFO("Object detection message");


    try
    {

		cout << "detect_length : " << detect_length << endl;
		cout << "loop_flag : " << loop_flag << endl;

		string Class_name;
        Max_Area = 0;
        // mission_flag=0;
        
		if(loop_flag == true)
		{
			for(int i = 0; i < detect_length; i++)
			{
				
				vision_msgs::Detection2D aDetection = detections2dArray->detections[i];

				
				// ROS_INFO(" ");

				// Id and confidence
				vision_msgs::ObjectHypothesisWithPose result = aDetection.results[0];
				int id = result.id;
				float score = result.score;
				// ROS_INFO("   id    %i", id);
				// ROS_INFO("   score %f", score);



				

				// Bounding boxes
				vision_msgs::BoundingBox2D boundingBox2d = aDetection.bbox;
				geometry_msgs::Pose2D center = boundingBox2d.center;
				// ROS_INFO("   position (%f, %f)", center.x, center.y);
				// ROS_INFO("   size %f x %f", boundingBox2d.size_x, boundingBox2d.size_y);

				Area = boundingBox2d.size_x * boundingBox2d.size_y;

	
			}
				
			loop_flag = false;

		}

		
		
		else
		{
			cout << "not yet" << endl;
		}
		
        // cout<<"light_test: "<<light_test<<endl;
        // traffic_sign_test_pub.publish(sign_test_msg);
    }
	catch (exception& e)
    {
    //   ROS_INFO("Exception %s", e.what());
      isDone = true;
    }

}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// printf("이미지 수신중\n");
	Mat image1 = cv_bridge::toCvShare(msg)->image;
	sub_image1 = image1.clone();
	cv::waitKey(1);
	callback = true;
	// imshow("image1", image1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obj_detecting");
	ros::NodeHandle nh;
	// ros::AsyncSpinner spinner(2);
	// spinner.start();
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/vision/image_raw", 1);
	image_transport::Subscriber sub_image = it.subscribe("/yolov7/yolov7/visualization", 1, imageCallback);
	
	detect_length_sub = nh.subscribe("/yolov7/detect_length", 1, detectionlength_callback);
	detection_sub = nh.subscribe("/yolov7/yolov7", 1, detections_callback);
	detect_flag_sub = nh.subscribe("/yolov7/flag_pub", 10, detect_flag_callback);

	// Korus ver. topic
    


	ros::Rate loop_rate(50);
	sensor_msgs::ImagePtr msg;
	// printf("Waiting for ---/yolov7/yolov7/visualization---\n");
	string cam_index = "/dev/video" + to_string(CAM_NUM);

	VideoCapture cap(cam_index);
	// VideoCapture cap("/home/korus/catkin_ws/src/VISION/src/video/0923_kcity/kcity_0923_02.mp4");
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	
	std_msgs::Bool mission_stop_msg;
	std_msgs::Bool mission_speed_down;
	
	while (ros::ok())
	{

		cap >> frame1;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();

			
		pub.publish(msg);
		

		imshow("publish_image", frame1);

		if (callback == true)
		{

			Mat img;
			cv::resize(sub_image1, img, cv::Size(640, 480), 0, 0);
			imshow("YoloV7",img);

		}

		
		waitKey(10);

		// ros::waitForShutdown();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}