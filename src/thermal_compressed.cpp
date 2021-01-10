// sensor_msgs/CompressedImage を展開してPublishするプログラム
// サーマルカメラの画像を圧縮すると，8bitになってしまい，16bitには戻せない．

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

image_transport::Publisher* _pub;

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  cv::Mat image;
  try {
    image = cv::imdecode(cv::Mat(msg->data),cv::IMREAD_COLOR);//convert compressed image data to cv::Mat RGBカメラ用
      // image = cv::imdecode(cv::Mat(msg->data),CV_LOAD_IMAGE_ANYDEPTH);//サーモ用
    
    std::cout << "type : " << image.type() << std::endl;
    // double minT, maxT;
    // cv::minMaxLoc(image, &minT, &maxT, NULL, NULL);
    // std::cout << "Min: " << minT << " //Max: " << maxT << std::endl;

    cv::imshow("view", image);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert to image!");
  }

  _pub->publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "thermal_compressed");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pubt = it.advertise("/image_raw8", 1);
  ros::Subscriber image_sub = nh.subscribe("/usb_cam/image_raw/compressed", 1, imageCallback);

  _pub = &pubt;

  ros::Rate loop_rate(30);
  while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
  }
}