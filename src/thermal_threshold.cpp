#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

// 12/3: optrisからの/thermal_imageトピックを受取り，ROSにトピックpublish
// 16bit-thermal画像をトラックバーで閾値変更
// 8bitに変換して表示＆Publish

class ImageThreshold{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;//Publisherを定義．topic名は/image_raw2
    image_transport::Subscriber image_sub;
    //Subscriverを定義topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定

    public:
        //コンストラクタ
        ImageThreshold()
            : it(nh){
                image_sub = it.subscribe("/thermal_image", 1, &ImageThreshold::imageTh, this);//Subscriberを定義．topic名は/thermal_image
                image_pub = it.advertise("/thermal_image_threshold", 1);    //Publisherを定義．
                //topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                cv::namedWindow("threshold", CV_WINDOW_NORMAL);
                cv::resizeWindow("threshold", 640, 480);//ウィンドウサイズのリサイズ
                int valMin = 900;//閾値低い方の初期値
                int valMax = 1000;//閾値高い方の初期値
                cv::createTrackbar("Threshold_min", "threshold", &valMin, 2000);//トラックバー作成
                cv::createTrackbar("Threshold_max", "threshold", &valMax, 2000);//トラックバー作成
            }

        //デストラクタ
        ~ImageThreshold() {
            cv::destroyWindow("threshold");
        }

        void rangeThermalImage(cv::Mat &dst, ushort minTh, ushort maxTh, cv::Mat &dst_mask) {
            // std::cout << dst << std::endl;
            for (int i=0; i<dst.rows; i++) {
                for (int j=0; j<dst.cols; j++) {
                    if (dst.at<ushort>(i,j) <= minTh) {
                        dst_mask.at<ushort>(i,j) = minTh;
                    } else if ((minTh < dst.at<ushort>(i,j)) && (dst.at<ushort>(i,j) < maxTh)) {
                        dst_mask.at<ushort>(i,j) = dst.at<ushort>(i,j);
                    } else if (maxTh <= dst.at<ushort>(i,j)) {
                        dst_mask.at<ushort>(i,j) = maxTh;
                    }
                }
            }
        }


        //Callback関数
        void imageTh(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr src;
            try {
                src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);//sensor_msgs::ImageConstPtr型から16bitのcv::Mat型に変換
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Mat dst = (src->image).clone();

            ushort thresh_min = cv::getTrackbarPos("Threshold_min", "threshold");//トラックバーから閾値を得る
            ushort thresh_max = cv::getTrackbarPos("Threshold_max", "threshold");//トラックバーから初期値を得る

            cv::Mat dst_mask = cv::Mat::zeros(dst.rows, dst.cols, CV_16UC1);//マスクした16bit出力画像用
            rangeThermalImage(dst, thresh_min, thresh_max, dst_mask);//指定した範囲内の画素からなる画像"dst_mask"を生成．
            cv::normalize(dst_mask, dst_mask, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化
            dst_mask.convertTo(dst_mask, CV_8UC1);//8bitにconvert

            sensor_msgs::ImagePtr mess = cv_bridge::CvImage(std_msgs::Header(), "mono8", dst_mask).toImageMsg();//Mat型からsensor_msgs/ImagePtr型へ変換．モノクロ画像なので，encodingはmono8にしてある．
            image_pub.publish(mess);//msgをpublish
            cv::imshow("threshold", dst_mask);//OpenCV画面上で表示

            cv::waitKey(1);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_threshold");//ノード名の設定
    ImageThreshold ith;//ImageConverterコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}