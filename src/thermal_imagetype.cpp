#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

// サーモ画像のビット深度とチャンネルを判定するプログラム

class ImageType{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;//Publisherを定義．topic名は/image_raw2
    image_transport::Subscriber image_sub;
    //Subscriverを定義topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定

    public:
        //コンストラクタ
        ImageType()
            : it(nh){
                image_sub = it.subscribe("/thermal_corrected", 1, &ImageType::imageType, this);//Subscriberを定義．topic名は/thermal_image
                image_pub = it.advertise("/thermal_type", 1);    //Publisherを定義．
                //topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
            }

        //デストラクタ
        ~ImageType() {
            // cv::destroyWindow("threshold");
        }

        //Callback関数
        void imageType(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr src;
            try {
                src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//sensor_msgs::ImageConstPtr型から16bitのcv::Mat型に変換
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Mat dst = (src->image).clone();
            double minT, maxT, minRaw, maxRaw;//元画像の最大値と最小値の格納用変数
            cv::minMaxLoc(dst, &minRaw, &maxRaw, NULL, NULL);//元画像の最大値と最小値のポインタを得る

            std::cout << "Min: " << minRaw << " //Max: " << maxRaw << std::endl;
            minT = (double)(minRaw - 1000.0)/10.0;//画素値から摂氏温度に変換
            maxT = (double)(maxRaw - 1000.0)/10.0;//画素値から摂氏温度に変換

            std::cout << "Min Temperature: " << minT << " //Max Temperature: " << maxT << std::endl;
            std::cout << " / size: " << dst.size().width 
             << " x " << dst.size().height << " / channel: " << dst.channels() << " / type: " << dst.type() << std::endl;
            std::cout << "type : CV_8UC1 = 0 / CV_8UC3 = 16 / CV_16UC1 = 2 / CV_16UC3 = 18" << std::endl;

            sensor_msgs::ImagePtr mess = cv_bridge::CvImage(std_msgs::Header(), "mono8", dst).toImageMsg();//Mat型からsensor_msgs/ImagePtr型へ変換．モノクロ画像なので，encodingはmono8にしてある．
            image_pub.publish(mess);//msgをpublish
            cv::waitKey(1);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_imagetype");//ノード名の設定
    ImageType itp;//ImageConverterコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}