#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : "  << CV_MAJOR_VERSION << endl;

  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::frameset frames;
  rs2::frame color_frame;
  
  // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  // cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30);  # stream 종류, size, format 설정등록
  // cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30);

  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);


  pipe.start(cfg);
  cout << "111111" << endl;
  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }
  cout << "2222" << endl;
  rs2::depth_frame depth = frames.get_depth_frame();
  cout << "3333" << endl;

  float width = depth.get_width();
  float height = depth.get_height();
  cout << "3333" << endl;

  namedWindow("Display Image", WINDOW_AUTOSIZE);

  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    frames = pipe.wait_for_frames();
    color_frame = frames.get_color_frame();
    depth = frames.get_depth_frame();

    Mat color(Size(640,480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    imshow("Display Image", color);
    // cout << "4444" << endl;

    width = depth.get_width();
    height = depth.get_height();

    float dist_to_center = depth.get_distance(width / 2, height / 2);
    cout << "dist_to_center : " << dist_to_center << endl;
    // cout << "5555" << endl;
    if(waitKey(1)==27) break;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
