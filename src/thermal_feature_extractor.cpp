// 12/13 thermal_imgae_divider.cpp によって分割されたサーモ画像を読み込み，特徴点の検出と合成を行えたらいいな

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to "publish & subscribe" images in ROS
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/time_synchronizer.h>//arrow you to synchronize topics

void orbDtec(cv::Mat &src1, cv::Mat &src2, cv::Mat &src3, cv::Mat &dst) {
    cv::normalize(src1, src1, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
    cv::normalize(src2, src2, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
    cv::normalize(src3, src3, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
    src1.convertTo(src1, CV_8UC1);//8bitにconvert
    src2.convertTo(src2, CV_8UC1);//8bitにconvert
    src3.convertTo(src3, CV_8UC1);//8bitにconvert

    cv::Mat src = src2.clone();
    cv::cvtColor(src, src, cv::COLOR_GRAY2BGR);

    //ORB抽出器
    std::vector<cv::KeyPoint> keypoint1, keypoint2, keypoint3;//特徴点を格納する領域を用意
    cv::Mat descriptor1, descriptor2, descriptor3;//特徴記述子を格納する領域
    cv::Ptr<cv::ORB> feature = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);//ORBオブジェクト生成．第1引数は抽出する特徴点の上限
    feature->detectAndCompute(src1, cv::noArray(), keypoint1, descriptor1);
    feature->detectAndCompute(src2, cv::noArray(), keypoint2, descriptor2);
    feature->detectAndCompute(src3, cv::noArray(), keypoint3, descriptor3);

    //特徴点の配列の合成．keypoint1,2,3をくっつける
    std::vector<cv::KeyPoint> keypoint_merge;//合成後の配列の宣言
    keypoint_merge.reserve( keypoint1.size() + keypoint2.size() + keypoint3.size() );//メモリの確保
    keypoint_merge.insert( keypoint_merge.end(), keypoint1.begin(), keypoint1.end() );//keypoint_mergeの最後尾にkeypoint1をくっつける
    keypoint_merge.insert( keypoint_merge.end(), keypoint2.begin(), keypoint2.end() );//keypoint_mergeの最後尾にkeypoint2をくっつける
    keypoint_merge.insert( keypoint_merge.end(), keypoint3.begin(), keypoint3.end() );//keypoint_mergeの最後尾にkeypoint3をくっつける

    // //記述子の配列の合成．descriptor1,2,3をくっつける
    std::vector<cv::Mat> descriptor_merge;//合成後の配列の宣言
    // descriptor_merge.reserve( descriptor1.rows + descriptor2.rows + descriptor3.rows );//メモリの確保
    for (int j=0; j<descriptor1.rows; j++)
        descriptor_merge.push_back(descriptor1.row(j));
    for (int j=0; j<descriptor2.rows; j++)
        descriptor_merge.push_back(descriptor2.row(j));
    for (int j=0; j<descriptor3.rows; j++)
        descriptor_merge.push_back(descriptor3.row(j));
    std::cout << "記述子1：" << descriptor1.size() << std::endl;
    std::cout << "記述子2：" << descriptor2.size() << std::endl;
    std::cout << "記述子3：" << descriptor3.size() << std::endl;
    std::cout << "記述子_merge：" << descriptor_merge.size() << std::endl;

    //特徴点表示のときの四角の大きさ．中心からこの座標値離れた位置に四角の頂点が来る
    cv::Point2f rec {5.0, 5.0};

    for (double i = 0; i < keypoint_merge.size(); i++) {
        cv::KeyPoint *point = &(keypoint_merge[i]);
        cv::circle(src, point->pt, 6, cv::Scalar(0,255,255), 1);//特徴点を円で表示
        // cv::rectangle(src, point->pt - rec, point->pt + rec, cv::Scalar(0, 255, 255), 1);//特徴点を四角で表示
    }
    dst = src.clone();    
}

void featureExtraction( const sensor_msgs::ImageConstPtr &msg1, const  sensor_msgs::ImageConstPtr &msg2, const  sensor_msgs::ImageConstPtr &msg3) {
    cv::Mat src, src1, src2, src3;
    try {
        src1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::MONO16)->image;
        src2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::MONO16)->image;
        src3 = cv_bridge::toCvCopy(msg3, sensor_msgs::image_encodings::MONO16)->image;
        // src = cv_bridge::toCvCopy(msg4, sensor_msgs::image_encodings::MONO16)->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat dst;
    orbDtec(src1, src2, src3, dst);

    cv::namedWindow("dst", CV_WINDOW_NORMAL);
    cv::resizeWindow("dst", 640, 480);
    cv::imshow("dst", dst);

    cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_image_extractor");//ノード名の設定
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::SubscriberFilter image_sub1(it, "/thermal_image_edit", 1);
    image_transport::SubscriberFilter image_sub2(it, "/thermal_image_edit2", 1);
    image_transport::SubscriberFilter image_sub3(it, "/thermal_image_edit3", 1);
    // image_transport::SubscriberFilter image_sub4(it, "/thermal_image", 1);
    // image_transport::Publisher image_pub1 = it.advertise("/thermal_image_canny1", 1);
    // image_transport::Publisher image_pub2 = it.advertise("/thermal_image_canny2", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(image_sub1, image_sub2, image_sub3, 10);// 3つのトピックを同時にSubscribeする．タイムスタンプが同じでないといけない．
    sync.registerCallback(boost::bind(&featureExtraction, _1, _2, _3));// 3つのトピックを同時にcallback関数に渡す
    // image_pub1.publish(mess1);//msgをpublish
    // image_pub2.publish(mess2);//msgをpublish

    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}