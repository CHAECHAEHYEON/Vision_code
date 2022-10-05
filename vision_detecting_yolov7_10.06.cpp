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
int mission_A = 0;
int mission_B = 0;
int count_mission = 0;
bool mission_stop = false;
int count_A1 = 0;
int count_A2 = 0;
int count_A3 = 0;
// int sign[4]={0,0,0,0};
int red = 0;
int green = 0;
int greenleft = 0;
int uturn = 0;
int mission_flag = 0;
bool A_flag = false;
bool speed_down = false;

bool loop_flag = false;
float Area = 0;

bool pub_flag = false;



//###########################KORUS ROS#################################
std_msgs::Int32MultiArray sign_msg;
std_msgs::Int32 sign_test_msg;
std_msgs::Bool A_flag_msg;
ros::Publisher final_pub;
ros::Publisher Stop_final_pub;
ros::Publisher traffic_sign_pub;
ros::Publisher traffic_sign_test_pub;
ros::Subscriber mission_sub;
ros::Publisher A_flag_pub;
ros::Publisher Speed_pub;

ros::Subscriber detection_sub;
ros::Subscriber detect_length_sub;
ros::Subscriber detect_flag_sub;

//###################33vision_msgs ROS################################





void imageCallback(const sensor_msgs::ImageConstPtr &msg);

void mission_cb(const std_msgs::Int16::ConstPtr& msgs)
{
	mission_flag = msgs->data;
}

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

	A_flag_msg.data = A_flag;
    A_flag_pub.publish(A_flag_msg);

   	ROS_INFO("---- ");
    ROS_INFO("Object detection message");

	sign_msg.data.clear();

	// aDetection.clear();
	red = 0;
	green = 0;
	greenleft = 0;
	uturn = 0;

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



				switch (id) 
				{
					case 0:
						Class_name = "green";
						break;
					case 1:
						Class_name = "red";
						break;
					case 2:
						Class_name = "greenleft";
						break;
					case 3:
						Class_name = "uturn";
						break;
					case 4:
						Class_name = "A1";
						break;
					case 5:
						Class_name = "A2";
						break;
					case 6:
						Class_name = "A3";
						break;
					case 7:
						Class_name = "B1";
						break;
					case 8:
						Class_name = "B2";
						break;
					case 9:
						Class_name = "B3";
						break;
				} 

				// Bounding boxes
				vision_msgs::BoundingBox2D boundingBox2d = aDetection.bbox;
				geometry_msgs::Pose2D center = boundingBox2d.center;
				// ROS_INFO("   position (%f, %f)", center.x, center.y);
				// ROS_INFO("   size %f x %f", boundingBox2d.size_x, boundingBox2d.size_y);

				Area = boundingBox2d.size_x * boundingBox2d.size_y;

				cout << "Class_name [ " << detectionsCount << "] : " << Class_name << " Area: " << Area << endl;

				cout << "score : " << score << endl;

				if (Class_name == "green")
				{
					green = 1;
				}
				else if (Class_name == "red")
				{
					red = 1;
				}
				else if (Class_name == "greenleft")
				{
					greenleft = 1;
				}
				else if (Class_name == "uturn")
				{
					uturn = 1;
				}
				else if (Class_name == "A1" && Area>1150 && mission_flag==7)   //1500
				{
					count_A1 += 1;
					A_flag=true;
					cout<<"Area: "<<Area<<endl;
				}
				else if (Class_name == "A2" && Area>1150 && mission_flag==7)
				{
					count_A2 += 1;
					A_flag=true;
					cout<<"Area: "<<Area<<endl;
				}
				else if (Class_name == "A3" && Area>1150 && mission_flag==7)
				{
					count_A3 += 1;
					A_flag=true;
					cout<<"Area: "<<Area<<endl;
				}
				else if (Class_name == "B1" || Class_name == "B2" || Class_name == "B3")
				{
					if((mission_flag== 8)&&(Area > 1000))
					{
						speed_down = true;
					}
					
					cout << "Class_name [ " << detectionsCount << "] : " << Class_name << " Area: " << Area << endl;

					double ratio = ( boundingBox2d.size_y)/(boundingBox2d.size_x);

					// cout << "Ratio : " << ratio << endl;

					if ((Area>2300) && ( ratio <=1.0) && (mission_flag==8))   //2700
					{
						
						speed_down = true;
						
						if (Class_name == "B1" && mission_flag==8)
						{
							mission_B = 1;
							speed_down = true;
						}

						else if (Class_name == "B2" && mission_flag==8)
						{
							mission_B = 2;
							speed_down = true;
						}

						else if (Class_name == "B3" && mission_flag==8)
						{
							mission_B = 3;
							speed_down = true;
							
						}
					}
				}
				// else if ((Class_name == "A3") && ( ( boundingBox2d.size_y)/(xmax - xmin)<=1.5) && (Area >2300) && (mission_flag==8))
				// {

				// 	cout << "Class_name [ " << detectionsCount << "] : " << Class_name << " Area: " << Area << endl;
				// 	speed_down = true;
				// 	mission_B = 3;

				// }

				
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
    traffic_sign_pub = nh.advertise<std_msgs::Int32MultiArray>("Vision/traffic_sign", 100);
    Stop_final_pub = nh.advertise<std_msgs::Bool>("Vision/B_sign", 10);
    A_flag_pub = nh.advertise<std_msgs::Bool>("Vision/A_sign", 10);
    Speed_pub = nh.advertise<std_msgs::Bool>("Vision/Speed", 10);
    mission_sub = nh.subscribe("/Planning/mission", 10, mission_cb);


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

		if(mission_flag == 1)
		{
			pub_flag = true;
		}
		else if(mission_flag == 7)
		{
			pub_flag = true;
		}
		else if(mission_flag == 8)
		{
			pub_flag = true;
		}
		else if(mission_flag == 20)
		{
			pub_flag = true;
		}
		else if(mission_flag == 28)
		{
			pub_flag = true;
		}
		else
		{
			pub_flag = false;
		}


		// if(pub_flag == true)
		// {
		// 	pub.publish(msg);
		// }

		imshow("publish_image", frame1);

		if (callback == true)
		{

			Mat img;
			cv::resize(sub_image1, img, cv::Size(640, 480), 0, 0);
			imshow("YoloV7",img);

		}

		if (detect == true)
		{
			cout<<"A_flag: "<<A_flag<< " [1 = A detect, 0 = nothing detect]" << endl;

			if (count_A1 < count_A2 && count_A3 < count_A2)
			{
				mission_A = 2;
			}
			else if (count_A1 < count_A3 && count_A2 < count_A3)
			{
				mission_A = 3;
			}
			else if (count_A2 < count_A1 && count_A3 < count_A1)
			{
				mission_A = 1;
			}
			cout<<"count_A == [ "<<count_A1<< ", " << count_A2<<", "<< count_A3<< " ]"<<endl;

			cout << "mission_A: " << mission_A << endl;
			cout << "mission_B: " << mission_B << endl;

			if ((mission_A == mission_B) && (mission_flag == 8))
			{
				mission_stop = true;
			}
			else if (mission_A != mission_B)
			{
				mission_stop = false;
			}
			cout << "mission_stop: " << mission_stop << endl;
			cout << "speed_down:" << speed_down << endl;

			mission_speed_down.data = speed_down;
			Speed_pub.publish(mission_speed_down);

			mission_stop_msg.data = mission_stop;
			Stop_final_pub.publish(mission_stop_msg);
			
			
			sign_msg.data = {red,green,greenleft, uturn};

			cout<<"red : "<<sign_msg.data[0] << "\t\tgreen : "<<sign_msg.data[1]<<"\tgreenleft : "<<sign_msg.data[2] <<endl;
			cout <<  "uturn : "<<sign_msg.data[3]  << endl;
			
			traffic_sign_pub.publish(sign_msg);

		}
		else
		{	
			cout << "-------------------------------------------------\n" << endl;
			cout << "\n" << endl;
			ROS_INFO("---- ");
    		ROS_INFO("Detect Nothing");


			A_flag_msg.data = A_flag;
   			A_flag_pub.publish(A_flag_msg);

			cout<<"A_flag: "<<A_flag<< " [1 = A detect, 0 = nothing detect]" <<endl;
			cout<<"count_A == [ "<<count_A1<< ", " << count_A2<<", "<< count_A3<< " ]"<<endl;

			cout << "mission_A: " << mission_A << endl;
			cout << "mission_B: " << mission_B << endl;

			cout << "mission_stop: " << mission_stop << endl;
        	cout << "speed_down:" << speed_down << endl;

			red = 0;
			green = 0;
			greenleft = 0;
			uturn = 0;

			sign_msg.data = {0, 0, 0, 0};

        	cout<<"red : "<<sign_msg.data[0] << "\t\tgreen : "<<sign_msg.data[1]<<"\tgreenleft : "<<sign_msg.data[2] << endl;
			cout <<  "uturn : "<<sign_msg.data[3]  << endl;
			traffic_sign_pub.publish(sign_msg);

			cout << "-------------------------------------------------\n" << endl;
			cout << "\n" << endl;
		}

		waitKey(10);

		// ros::waitForShutdown();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
