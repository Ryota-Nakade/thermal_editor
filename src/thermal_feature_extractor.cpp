// 12/13 thermal_imgae_divider.cpp によって分割されたサーモ画像を読み込み，特徴点の検出と合成を行えたらいいな

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

class FeatureExtractor{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;//Subscriverを定義
    image_transport::Subscriber image_sub2;//Subscriverを定義
    image_transport::Subscriber image_sub3;//Subscriverを定義
    image_transport::Publisher image_pub;//Publisherを定義

    public:
        //コンストラクタ
        FeatureExtractor()
            : it(nh){
                image_sub = it.subscribe("/thermal_image_edit", 1, &FeatureExtractor::FeatureEx, this);
                image_sub2 = it.subscribe("/thermal_image_edit2", 1, &FeatureExtractor::FeatureEx, this);
                image_sub3 = it.subscribe("/thermal_image_edit3", 1, &FeatureExtractor::FeatureEx, this);
                //subscriberを定義．第1引数はtopic名，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                image_pub = it.advertise("/thermal_image_feature", 1);//publisherを定義．Publishするトピック名を設定．                
                // cv::namedWindow("src", CV_WINDOW_NORMAL);//ウィンドウをリサイズ可能に設定
                // cv::resizeWindow("src", 640, 480);//ウィンドウサイズのリサイズ
            }

        //デストラクタ
        ~FeatureExtractor() {
            cv::destroyWindow("src");
        }

        void FeatureEx(const sensor_msgs::ImageConstPtr &msg) {

        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_image_extractor");//ノード名の設定
    FeatureExtractor fe;//FeatureExtractorコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}