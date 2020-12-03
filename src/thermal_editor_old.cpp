#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//Callbacked by spin()
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrmal_editor");//initialize node. decrear name of onde is "thermal_editor"
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  //define subscriber using image_transport. Subscribing topic is "/camera/image_raw".
  //Second argument shows queue size.
  //Third argument is function which is called when topic is "/camera/image_raw"
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
  
  //spin() calls Callback function everytime topic received.
  ros::spin();
  cv::destroyWindow("view");
}