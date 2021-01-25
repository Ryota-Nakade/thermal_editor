// 12/3 uvc_cameraからの/image_rawトピックを受取り，ROSにトピックpublish
//      16bit-thermal画像を8bit-grayscaleに変換して表示．特徴点検出は行わない
// 12/9 温度の最大値と最小値の表示を追加
//      8bitのヒストグラムの均一化と表示を追加．黒いヒストグラムが元画像（8bit）で，赤いヒストグラムが均一化後
// 12/13 指定した温度範囲の画素のみを抽出して画像を分割する

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

class ImageEqualizer{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    //Subscriverを定義
    image_transport::Publisher image_pub;//Publisherを定義

    public:
        //コンストラクタ
        ImageEqualizer()
            : it(nh){
                image_sub = it.subscribe("/thermal_image", 1, &ImageEqualizer::imageEq, this);//subscriberを定義．サブスクライブするtopic名を指定．
                //第1引数はtopic名，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                image_pub = it.advertise("/thermal_image_eqhist", 1);//publisherを定義．Publishするトピック名を設定．

                cv::namedWindow("histgram", CV_WINDOW_NORMAL);
                cv::namedWindow("equalize", CV_WINDOW_NORMAL);
                cv::resizeWindow("histgram", 300, 300);
                cv::resizeWindow("equalize", 640, 480);
            }

        //デストラクタ
        ~ImageEqualizer() {
            // cv::destroyWindow("src");
        }


        //Callback関数
        void imageEq(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr src;
            try {
                src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Mat dst = (src->image).clone();
           
            cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            dst.convertTo(dst, CV_8UC1);//8bitにconvert

            // 以下ヒストグラム均一化ーーーーーーーーーーーーーーーーー
            // std::vector<cv::Scalar> cScalar{cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255)};//表示するヒストグラムの色を指定
            std::vector<cv::Scalar> cScalar{cv::Scalar(0, 0, 0)};//表示するヒストグラムの色を指定
            cv::Mat hist(cv::Size(256, 256), CV_8UC3);//ヒストグラムのサイズを指定．ここでは256 x 256px で3チャンネル
            cv::Mat dst_eq;//ヒストグラム均一化後の画像を格納する領域
            cv::equalizeHist(dst, dst_eq);//dstをヒストグラム均一化してdst_eqに入力

            // ヒストグラム表示ーーーーーーーーーーーーーーーーーーー
            // std::vector<cv::Mat> ch{dst, dst_eq};
            std::vector<cv::Mat> ch{dst_eq};
            std::vector<int> size{256};//binの本数．つまりヒストグラムの縦棒の数
            std::vector<float> range{0, 256};//ヒストグラムの値の範囲．左の値”以上”，右の値”未満”
            hist.setTo(255);
            for (int c = 0; c < 1; c++ ) {
                std::vector<int> channel{c};//チャンネル指定用
                cv::Mat bins;//ヒストグラムの結果格納用配列
                cv::calcHist(ch, channel, cv::noArray(), bins, size, range);
                cv::Point prev = cv::Point(0, 255);
                for (int x = 0; x < 256; x++) {
                    cv::Point current = cv::Point(x,256 - (int) (bins.at<float>(x) / 75 ));//75で割って大きさ調整する
                    cv::line(hist, prev, current, cScalar[c]);
                    prev = current;
                }
            }
            //以上　ヒストグラム関係ーーーーーーーーーーーーー

            sensor_msgs::ImagePtr mess = cv_bridge::CvImage(std_msgs::Header(), "mono8", dst_eq).toImageMsg();//Mat型からsensor_msgs/ImagePtr型へ変換．モノクロ画像なので，encodingはmono16にしてある．
            image_pub.publish(mess);//msgをpublish
            cv::imshow("histogram", hist);//ヒストグラムを表示
            cv::imshow("equalize", dst_eq);
            cv::waitKey(1);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_image_EQhistgram");//ノード名の設定
    ImageEqualizer ie;//ImageConverterコンストラクタの呼び出し．引数はなく，変数ieはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}