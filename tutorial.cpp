#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <image_transport/image_transport.h>
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
#include <librealsense2/rs.hpp>
#include <vector>
using namespace std;

double target_speed = 20.0;
double current_speed = 20.0;

cv::Mat sub_image1;
bool callback = false;
int stt;
int detect_length = 0;
bool detect = false;

vector <int> center_point;
vector <int> center_point_2;

using namespace cv;
using namespace std;

ros::Subscriber detection_sub;
ros::Subscriber detect_length_sub;
ros::Subscriber detect_flag_sub;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	// printf("이미지 수신중\n");
	Mat image1 = cv_bridge::toCvShare(msg)->image;
	sub_image1 = image1.clone();
	cv::waitKey(1);
	callback = true;
	// imshow("image1", image1);
}

void detectionlength_callback(const std_msgs::Int16::ConstPtr &msg)
{
	detect_length = msg->data;
}

void detect_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
	detect = msg -> data;
}

void detections_callback(const vision_msgs::Detection2DArray::ConstPtr &detections2dArray)
{
  int isDone = false;
  int detectionsCount = 0;

  ROS_INFO("Object detection message");

  try
  {
    // cout << "detect_length : " << detect_length << endl;
    // cout << "loop_flag : " << loop_flag << endl;

    string Class_name;
        // Max_Area = 0;
        // mission_flag=0;
        

    for(int i = 0; i < detect_length; i++)
    {
      Class_name = "0";
      vision_msgs::Detection2D aDetection = detections2dArray->detections[i];
      // Id and confidence
      vision_msgs::ObjectHypothesisWithPose result = aDetection.results[0];
      int id = result.id;
      float score = result.score;
      // ROS_INFO("   id    %i", id);
      // ROS_INFO("   score %f", score);

      switch (id) 
      {
        case 67:
          Class_name = "speedbomp";
          break;
        case 0:
          Class_name = "pothole";
          break;
        // case 2:
        //   Class_name = "speedbomp";
        //   break;
      } 

      if(Class_name == "speedbomp")
      {
        vision_msgs::BoundingBox2D boundingBox2d = aDetection.bbox;
        geometry_msgs::Pose2D center = boundingBox2d.center;

        cout << " speed_bomp : " << center << endl;
        
        center_point.push_back(center.x);
        center_point.push_back(center.y);
      }
      if(Class_name == "pothole")
      {
        vision_msgs::BoundingBox2D boundingBox2d = aDetection.bbox;
        geometry_msgs::Pose2D center = boundingBox2d.center;

        cout << " pothole : " << center << endl;
        
        center_point_2.push_back(center.x);
        center_point_2.push_back(center.y);
      }
    }
    // for(int i = 0; i < center_point.size(); i += 2)
    // {
    //   cout << "center_point_x : " << center_point[i] << endl;
    //   cout << "center_point_y : " << center_point[i+1] << endl;  
    // }
      
  }
	catch (exception& e)
  {
  //   ROS_INFO("Exception %s", e.what());
    isDone = true;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/vision/image_raw", 100);
	image_transport::Subscriber sub_image = it.subscribe("/yolov7/yolov7/visualization", 1, imageCallback);

  sensor_msgs::ImagePtr msg;

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : "  << CV_MAJOR_VERSION << endl;

  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::frameset frames;
  rs2::frame color_frame;

  detect_length_sub = nh.subscribe("/yolov7/detect_length", 1000, detectionlength_callback);
	detection_sub = nh.subscribe("/yolov7/yolov7", 10, detections_callback);
	detect_flag_sub = nh.subscribe("/yolov7/flag_pub", 10, detect_flag_callback);

  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);


  pipe.start(cfg);
  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }
  rs2::depth_frame depth = frames.get_depth_frame();
 
  float width = depth.get_width();
  float height = depth.get_height();
 
  namedWindow("Display Image", WINDOW_AUTOSIZE);

  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    if (callback == true)
		{
			Mat img;
			cv::resize(sub_image1, img, cv::Size(640, 480), 0, 0);
			// cvtColor(img, img, CV_RGB2BGR);
			imshow("YoloV7",img);
		}

    frames = pipe.wait_for_frames();
    color_frame = frames.get_color_frame();
    depth = frames.get_depth_frame();

    Mat color(Size(640,480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
		
		pub.publish(msg);

    float dist_to_center = 0;
    float dist_to_center_2 = 0;
    // if((center_point.x > 0)&&(center_point.y > 0))
    // {
    //   dist_to_center = depth.get_distance(center_point.x, center_point.y);
    //   cout << "dist_to_center : " << dist_to_center << endl<< endl;
  
    //   string dist_Text = to_string(dist_to_center);
    //   putText(color, dist_Text, Point(center_point.x-30,center_point.y), 1, 2, Scalar(255, 255, 0), 1, 8);
    //   circle(color, Point(center_point.x,center_point.y), 10, Scalar(255, 255, 255), -1, 8, 0);
    // }

    if(center_point.size() > 0)
    {
      for(int i = 0; i < center_point.size(); i+=2)
      {
        dist_to_center = depth.get_distance(center_point[i], center_point[i+1]);
        // cout << "dist_to_center : " << dist_to_center << endl<< endl;
        stt = 1;
        if(dist_to_center <= 1)
        {
          target_speed = 10.0;
        }
        else
        {
          target_speed = 20.0;
        }
        string dist_Text = to_string(dist_to_center);
        putText(color, dist_Text, Point(center_point[i]-40,center_point[i+1]-20), 1, 2, Scalar(255, 255, 0), 1, 8);
        circle(color, Point(center_point[i],center_point[i+1]), 5, Scalar(255, 255, 255), -1, 8, 0);
      }
    }
    else
    {
      stt = 0;
    }

    if(stt == 1)
    {
      if(current_speed < 10.0)
      {
        current_speed = 10.0;
      }
      else if(current_speed > target_speed)
      {
        current_speed -= 0.1;
      }
      else if(current_speed < target_speed)
      {
        current_speed +=0.1;
      }
    }
    else
    {
      target_speed = 20.0;
      if(current_speed < target_speed)
      {
        current_speed +=0.1;
      }
    }

    if(center_point_2.size() > 0)
    {
      for(int i = 0; i < center_point_2.size(); i+=2)
      {
        dist_to_center_2 = depth.get_distance(center_point_2[i], center_point_2[i+1]);
        // cout << "dist_to_center : " << dist_to_center << endl<< endl;

        if(dist_to_center_2 <= 2)
        {
        string dist_Text = "!";
          putText(color, dist_Text, Point(center_point_2[i],center_point_2[i+1]), 1, 5, Scalar(0, 0, 255), 1, 8);
        }
      }
    }

    string target_speed_string = " target speed : "+to_string((int)target_speed) + "km/h";
    putText(color, target_speed_string, Point(30,30), 1, 2, Scalar(255, 255, 0), 1, 8);
    string current_speed_string = "currunt speed : " + to_string((int)current_speed) + "km/h";
    putText(color, current_speed_string, Point(30,60), 1, 2, Scalar(255, 255, 0), 1, 8);

    center_point.clear();
    center_point_2.clear();

    // center_point = {0,0};
    imshow("Display Image", color);

    if(waitKey(10)==27) break;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
