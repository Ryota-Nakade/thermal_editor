//サーモ画像のトピック化から画像を保存するノード

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Callbacked by spin()
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr src;
  try {
    src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  std_msgs::Header h = src->header;
  std::cout << "h: " << h << std::endl; //all at once
  std::cout << "h.stamp: " << h.stamp << std::endl; //specific parts of it
  std::cout << "h.stamp.sec: " << h.stamp.sec << std::endl;
  std::cout << "h.stamp.nsec: " << h.stamp.nsec << std::endl;
  std::cout << "h.seq: " << h.seq << std::endl;

  cv::Mat dst = (src->image).clone();
  cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);//normalize to 0-255
  dst.convertTo(dst, CV_8UC1);

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(6) << h.seq;
  cv::imwrite("pic_" + oss.str() + ".png", dst);//画像を保存

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrmal_photo");//initialize node. decrear name of onde is "thermal_editor"
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  //define subscriber using image_transport. Subscribing topic is "/camera/image_raw".
  //Second argument shows queue size.
  //Third argument is function which is called when topic is "/camera/image_raw"
  image_transport::Subscriber sub = it.subscribe("/thermal_image", 1, imageCallback);
  
  //spin() calls Callback function everytime topic received.
  ros::spin();
  cv::destroyWindow("view");
}