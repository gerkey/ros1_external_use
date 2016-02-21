#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sensor_msgs/Image.h>
using std::cout;
using std::endl;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_publisher");
  ros::NodeHandle n;
  ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("image", 1);
  cv::VideoCapture cap;
  if (!cap.open(0))
  {
    cout << "couldn't open cv::VideoCapture!" << endl;
    return 1;
  }
  cv::Mat cv_frame;
  cap >> cv_frame;
  if (cv_frame.empty())
  {
    cout << "couldn't get a frame from cv::VideoCapture!" << endl;
    return 1;
  }
  cout << "publishing " << cv_frame.cols << "x" << cv_frame.rows
       << " capture stream to ROS..." << endl 
       << "to view it: 'rosrun image_view image_view'" << endl;
  sensor_msgs::Image image_msg;
  image_msg.header.frame_id = "webcam";
  image_msg.height = cv_frame.rows;
  image_msg.width = cv_frame.cols;
  image_msg.encoding = "bgr8"; // hopefully works for most webcams
  image_msg.is_bigendian = 0;
  image_msg.step = cv_frame.step[0];
  int data_len = cv_frame.step[0] * cv_frame.rows;
  image_msg.data.resize(data_len);
  while (ros::ok() && !cv_frame.empty())
  {
    cap >> cv_frame;
    memcpy(&image_msg.data[0], cv_frame.data, data_len);
    image_pub.publish(image_msg);
    ros::spinOnce();
  }
  return 0;
}
