#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// 12/3: usbカメラの画像をimage_rawとしてpublishするノード
//コメントを入れ替えればuvc_canera_nodeからの画像をsubscribeすることもできる

// cv::Mat image;
// void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
//     try {
//         image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
//     }
//     catch (cv_bridge::Exception &e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
//     cv::imshow("image", image);
//     cv::waitKey(1);
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "usb_pub");//ノード名の設定
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/image_raw", 10);//Publisherを定義．topic名は/image_raw
    // image_transport::Subscriber image_sub = it.subscribe("/image_raw", 10, imageCallback);
    //Subscriverを定義topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
    cv::Mat src;
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        ROS_INFO("ERROR : failed to open camera");
        return -1;
    }

    ros::Rate looprate (66);//topicを受信する周期．10Hz

    while (ros::ok())
    {
        cap >> src;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();//Mat型からsensor_msgs/ImagePtr型へ変換
        image_pub.publish(msg);//msgをpublish
        ros::spinOnce();
        looprate.sleep();//設定した周期でスリープ
    }
    // ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
    
}