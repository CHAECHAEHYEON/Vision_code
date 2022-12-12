#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
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

using namespace cv;
using namespace std;

Mat sub_image1;

bool callback = false;

double max_speed = 20.0;
double current_speed = 20.0;

int stt;

int detect_count = 0;
bool detect = false;
int detect_length = 0;

float Max_Area = 0;
float Area = 0;

int yolov7_sub_count = 0;
int detect_length_count = 0;

bool sub_flag = true;

bool calc_flag = true;

ros::Subscriber detection_sub;
ros::Subscriber detect_length_sub;
ros::Subscriber detect_flag_sub;

ros::Publisher pub_flag_pub;

// vector<float> center_interest_x;
// vector<float> center_interest_y;
vector<geometry_msgs::Pose2D> center_interest;
vector<float> boundingbox_x_size;
vector<float> boundingbox_y_size;

void detectionlength_callback(const std_msgs::Int16::ConstPtr &msg)
{

  // detect_length_count++;
  sub_flag = false;
  detect_length = msg->data;
  // cout << "\n" << "detect_length sub_flag 1 : " << sub_flag << "\n" << endl;
  cout << "\n" << "detect_length : " << detect_length << "\n" << endl;

  sub_flag = true;  

}

void detect_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
	detect = msg -> data;
}

void detections_callback(const vision_msgs::Detection2DArray::ConstPtr &detections2dArray)
{

  // yolov7_sub_count++;

  // if(detect_length_count == yolov7_sub_count)
  // {
  //   sub_flag = true;
  //   // cout << "\n" << "detect_length sub_flag 2 : " << sub_flag << "\n" << endl;
  //   detect_length_count = 0;
  //   yolov7_sub_count = 0;
  // }
  // else
  // {
  //   sub_flag = false;
  //   cout << "detect_length_count : " << detect_length_count << endl;
  //   cout << "yolov7_sub_count : " << yolov7_sub_count << endl;
  // }

  std_msgs::Bool pub_flag_msg;

  pub_flag_msg.data = false;

  pub_flag_pub.publish(pub_flag_msg);

  cout << "sub_flag" << sub_flag << endl;

  calc_flag = true;

  int isDone = false;
  int detectionsCount = 0;

  // ROS_INFO_STREAM("---- ");
  // ROS_INFO_STREAM("Object detection message");

  int detect_length_copy = detect_length;
  if(detect == true && sub_flag == true)
  {

    // sub_flag = false;

    // cout << "detect_length : " << detect_length_copy << endl;
    // cout << "sub_flag : " << sub_flag << endl;

    string Class_name;
    Max_Area = 0;
      
    for(int i = 0; i < (*detections2dArray).detections.size(); i++)
    {
      cout << "55" << endl;
      
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

      // ROS_INFO("Area : %f", Area);

      if(id == 0 || id == 1)
      {
        detect_count++;

        boundingbox_x_size.push_back(boundingBox2d.size_x);

        boundingbox_y_size.push_back(boundingBox2d.size_y);
        // center_interest_x.push_back(center.x);
        // center_interest_y.push_back(center.y);
        center_interest.push_back(center);

      }
    }

      
    
		
  }
  else
  {
    cout << "Don't" << endl;
  //   calc_flag = false;

  //   cout << "not yet to detect" << endl;
  //   boundingbox_x_size.clear();
  //   boundingbox_y_size.clear();
  //   center_interest.clear();
  }
  // catch (exception& e)
  // {
  // //   ROS_INFO("Exception %s", e.what());
  //   isDone = true;
  // }

  pub_flag_msg.data = true;
  pub_flag_pub.publish(pub_flag_msg);


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
  ros::init(argc, argv, "realsense2");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/vision/image_raw", 1);
  image_transport::Subscriber sub_image = it.subscribe("/yolov7/yolov7/visualization", 1, imageCallback);

  pub_flag_pub = nh.advertise<std_msgs::Bool>("yolov7/pub_flag", 1);

  detect_length_sub = nh.subscribe("/yolov7/detect_length", 10, detectionlength_callback);
	detection_sub = nh.subscribe("/yolov7/yolov7", 1, detections_callback);
	detect_flag_sub = nh.subscribe("/yolov7/flag_pub", 10, detect_flag_callback);

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : "  << CV_MAJOR_VERSION << endl;

  cout << "🚀️🚀️🚀️🚀️🚀️🚀️🚀️🚀️" << endl;

  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::frameset frames;
  rs2::frame color_frame;
  
  // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  // cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30);  # stream 종류, size, format 설정등록
  // cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30);

  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30); // 깊이값 들여옴
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30); //RGB 컬러값 들여옴
  cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30); //적외선 값 들여옴


  pipe.start(cfg);
  // cout << "111111" << endl;
  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }
  rs2::depth_frame depth = frames.get_depth_frame();

  float width = depth.get_width();
  float height = depth.get_height();

  namedWindow("Display Image", WINDOW_AUTOSIZE);

  ros::Rate loop_rate(1000);
  sensor_msgs::ImagePtr msg;

  while(ros::ok())
  {
    int compair_detect_count = 0;


    frames = pipe.wait_for_frames();
    color_frame = frames.get_color_frame();
    depth = frames.get_depth_frame();

    Mat color(Size(640,480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    // imshow("Display Image", color);

    //이미지 화면 퍼블리쉬
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();

    pub.publish(msg);
    // cout << "4444" << endl;

    if (callback == true)
		{

			Mat img;
			cv::resize(sub_image1, img, cv::Size(640, 480), 0, 0);
			imshow("YoloV7",img);

		}

    // cout << "4444" << endl;

    cout <<"detect count : " <<  detect_count << endl;

    // cout << "yet loop" << endl;

    // ROS_INFO_STREAM(boundingbox_x_size.size());
    // ROS_INFO_STREAM(boundingbox_y_size.size());
    // ROS_INFO_STREAM(center_interest.size());

    cv::Scalar calor(rand() & 255, rand() & 255, rand() & 255);

    float dist_to_center;
    float dist_to_center_compair = 1000000;
    float dist_to_center_minimmum = 0;

      for(int d = 0; d < detect_count; d++)
      {
        // cout << "444" << endl;

        //1번부터 2번(좌상단부터 반시계방향)
        // line(color, Point(center_interest_x[d]- (boundingbox_x_size[d]/2),center_interest_y[d]-(boundingbox_y_size[d]/2)), Point(center_interest_x[d]-(boundingbox_x_size[d]/2),center_interest_y[d]+(boundingbox_y_size[d]/2)),Scalar(100,100,100), 1, 8, 0);
        line(color, Point(center_interest[0].x- (boundingbox_x_size[0]/2),center_interest[0].y-(boundingbox_y_size[0]/2)), Point(center_interest[0].x-(boundingbox_x_size[0]/2),center_interest[0].y+(boundingbox_y_size[0]/2)),calor, 5, 8, 0);

        //2번부터 3번
        // line(color, Point(center_interest_x[d]-(boundingbox_x_size[d]/2),center_interest_y[d]+(boundingbox_y_size[d]/2)), Point(center_interest_x[d]+(boundingbox_x_size[d]/2),center_interest_y[d]+(boundingbox_y_size[d]/2)),Scalar(100,100,100), 1, 8, 0);
        line(color, Point(center_interest[0].x-(boundingbox_x_size[0]/2),center_interest[0].y+(boundingbox_y_size[0]/2)), Point(center_interest[0].x+(boundingbox_x_size[0]/2),center_interest[0].y+(boundingbox_y_size[0]/2)),calor, 5, 8, 0);

        //3번부터 4번
        // line(color, Point(center_interest_x[d]+(boundingbox_x_size[d]/2),center_interest_y[d]+(boundingbox_y_size[d]/2)), Point(center_interest_x[d]+(boundingbox_x_size[d]/2),center_interest_y[d]-(boundingbox_y_size[d]/2)),Scalar(100,100,100), 1, 8, 0);
        line(color, Point(center_interest[0].x+(boundingbox_x_size[0]/2),center_interest[0].y+(boundingbox_y_size[0]/2)), Point(center_interest[0].x+(boundingbox_x_size[0]/2),center_interest[0].y-(boundingbox_y_size[0]/2)),calor, 5, 8, 0);

        //4번부터 1번
        // line(color, Point(center_interest_x[d]+(boundingbox_x_size[d]/2),center_interest_y[d]-(boundingbox_y_size[d]/2)), Point(center_interest_x[d]-(boundingbox_x_size[d]/2),center_interest_y[d]-(boundingbox_y_size[d]/2)),Scalar(100,100,100), 1, 8, 0);
        line(color, Point(center_interest[0].x+(boundingbox_x_size[0]/2),center_interest[0].y-(boundingbox_y_size[0]/2)), Point(center_interest[0].x-(boundingbox_x_size[0]/2),center_interest[0].y-(boundingbox_y_size[0]/2)),calor, 5, 8, 0);

        dist_to_center = depth.get_distance(center_interest[0].x, center_interest[0].y);

      if(dist_to_center_compair >= dist_to_center)
      {
        dist_to_center_minimmum = dist_to_center;
        dist_to_center_compair = dist_to_center;
      }

        cout << "dist_to_center : " << dist_to_center << endl;

        string dist_Text = to_string(dist_to_center);
        putText(color, dist_Text, Point(center_interest[0].x-40,center_interest[0].y-20), 1, 2, Scalar(255, 255, 0), 1, 8);
        circle(color, Point(center_interest[0].x,center_interest[0].y), 5, Scalar(255, 255, 255), -1, 8, 0);

        // string Text = "🚀️🚀️🚀️🚀️🚀️🚀️🚀️🚀️";
        // putText(color, Text, Point(center_interest[0].x,center_interest[0].y), 1, 5, Scalar(0, 0, 255), 1, 8);

        boundingbox_x_size.erase(boundingbox_x_size.begin());
        boundingbox_y_size.erase(boundingbox_y_size.begin());
        center_interest.erase(center_interest.begin());

        // cout << "boundingbox_x_size" << boundingbox_x_size[0] << endl;
        // cout << "boundingbox_y_size" << boundingbox_y_size[0] << endl;
        // cout << "center_interest" << center_interest[0] << endl;

        compair_detect_count++;

        stt = 1;
        if(dist_to_center_minimmum <= 2)
        {
          	max_speed = 10.0;
        }
        else
        {
          	max_speed = 20.0;
        }

        if(compair_detect_count == detect_count)
        {
          detect_count = 0;
          compair_detect_count = 0;

          boundingbox_x_size.clear();
          boundingbox_y_size.clear();
          center_interest.clear();

          // cout << "boundingbox_x_size" << boundingbox_x_size[0] << endl;
          // cout << "boundingbox_y_size" << boundingbox_y_size[0] << endl;
          // cout << "center_interest" << center_interest[0] << endl;
        }
      
      }

    
      if(current_speed < 10.0)
      {
        current_speed = 10.0;
      }
      else if(current_speed > max_speed)
      {
        current_speed -= 0.1;
      }
      else if(current_speed < max_speed)
      {
        current_speed +=0.1;
      }
    

    string target_speed_string = " target speed : "+to_string((int)max_speed) + "km/h";
    putText(color, target_speed_string, Point(30,30), 1, 2, Scalar(255, 255, 0), 1, 8);
    string current_speed_string = "currunt speed : " + to_string((int)current_speed) + "km/h";
    putText(color, current_speed_string, Point(30,60), 1, 2, Scalar(255, 255, 0), 1, 8);
    // else
    // {
    //   cout << "timming error !!!" << endl;
    //   detect_count = 0;
    //   boundingbox_x_size.clear();
    //   boundingbox_y_size.clear();
    //   center_interest.clear();
    // }

    // cout << "end loop" << endl;

    // ROS_INFO_STREAM(boundingbox_x_size.size());
    // ROS_INFO_STREAM(boundingbox_y_size.size());
    // ROS_INFO_STREAM(center_interest.size());

    imshow("Display Image", color);

    // cout << "4444" << endl;

    // width = depth.get_width();
    // height = depth.get_height();

    // float dist_to_center = depth.get_distance(width / 2, height / 2);
    
    // cout << "dist_to_center : " << dist_to_center << endl;

    // cout << "5555" << endl;



    if(waitKey(1)==27) break;
    loop_rate.sleep();
    ros::spinOnce();

    // boundingbox_x_size.clear();
    // boundingbox_y_size.clear();
    // center_interest.clear();
  }
  return 0;
}