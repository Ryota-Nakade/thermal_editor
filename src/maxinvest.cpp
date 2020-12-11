// 12/3 uvc_cameraからの/image_rawトピックを受取り，ROSにトピックpublish
//      16bit-thermal画像を8bit-grayscaleに変換して表示．特徴点検出は行わない
// 12/9 温度の最大値と最小値の表示を追加
//      8bitのヒストグラムの均一化と表示を追加．黒いヒストグラムが元画像（8bit）で，赤いヒストグラムが均一化後

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

cv::Mat src;//Subscribeした画像を1フレームごとに格納する場所を用意
class ImageConverter{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;//Publisherを定義．topic名は/image_raw2
    image_transport::Subscriber image_sub;
    //Subscriverを定義topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定

    public:
        //コンストラクタ
        ImageConverter()
            : it(nh){
                image_sub = it.subscribe("/thermal_image", 1, &ImageConverter::imageCb, this);//subscriberを定義．サブスクライブするtopic名を指定．
                image_pub = it.advertise("/image_edit", 1);//publisherを定義．Publishするトピック名を設定．
                //topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                cv::namedWindow("src", CV_WINDOW_NORMAL);//ウィンドウをリサイズ可能に設定
                cv::resizeWindow("src", 640, 480);//ウィンドウサイズのリサイズ
                cv::namedWindow("dst_eq", CV_WINDOW_NORMAL);//ウィンドウをリサイズ可能に設定
                cv::resizeWindow("dst_eq", 640, 480);//ウィンドウサイズのリサイズ
            }

        //デストラクタ
        ~ImageConverter() {
            cv::destroyWindow("src");
        }

        //Callback関数
        void imageCb(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr cv_ptr, src;
            try {
                // cv_ptr  = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
                src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Mat dst = src->image;
            double minT, maxT;//元画像の最大値と最小値の格納用変数
            cv::minMaxLoc(dst, &minT, &maxT, NULL, NULL);//元画像の最大値と最小値のポインタを得る
            std::cout << "Min: " << minT << " //Max: " << maxT << std::endl;
            minT = (double)(minT - 1000.0)/10.0;//画素値から摂氏温度に変換
            maxT = (double)(maxT - 1000.0)/10.0;//画素値から摂氏温度に変換
            
            // dst.convertTo(dst2, CV_32FC1);//32bit浮動小数点に変換
            // cv::normalize(dst2, dst2, 0, 1, cv::NORM_MINMAX);//0~1に正規化

            cv::Mat dst2 = src->image;
            cv::normalize(dst2, dst2, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            dst.convertTo(dst2, CV_8UC1);//8bitにconvert
            
            // double minT2, maxT2;//8bit画像の最大・最小輝度の格納用変数
            // cv::minMaxLoc(dst2, &minT2, &maxT2, NULL, NULL);//8bit画像の最大値と最小値のポインタを得る

            // //　以下　モルフォロジー演算ーーーーーーーー
            // cv::Mat dst_e, dst_d;
            // cv::Mat element8 = (cv::Mat_<uchar>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1); // 8近傍
            // cv::morphologyEx(dst2, dst_e, cv::MORPH_OPEN, element8, cv::Point(-1, -1), 1);
            // cv::morphologyEx(dst_e, dst_d, cv::MORPH_CLOSE, element8, cv::Point(-1, -1), 1);
            // cv::imshow("morphology", dst_d);
            // //　以上　モルフォロジー演算．これを有効化したときは，以下のequalizeHistの第一引数をdst_dの変更するーーーーーーーー

            // 以下ヒストグラム均一化ーーーーーーーーーーーーーーーーー
            std::vector<cv::Scalar> cScalar{cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255)};//表示するヒストグラムの色を指定
            cv::Mat hist(cv::Size(256, 256), CV_8UC3);//ヒストグラムのサイズを指定．ここでは256 x 256px で3チャンネル
            cv::Mat dst2_eq;//ヒストグラム均一化後の画像を格納する領域

            //ヒストグラム表示ーーーーーーーーーーーーーーーーーーー
            cv::equalizeHist(dst2, dst2_eq);//dst2をヒストグラム均一化してdst2_eqに入力
            std::vector<cv::Mat> ch{dst2, dst2_eq};
            std::vector<int> size{256};//binの本数．つまりヒストグラムの縦棒の数
            std::vector<float> range{0, 256};//ヒストグラムの値の範囲．左の値”以上”，右の値”未満”
            hist.setTo(255);
            for (int c = 0; c < 2; c++ ) {
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

            //特定の輝度の範囲のみマスクして抽出ーーーーーーー
            cv::Mat mask = cv::Mat::zeros(dst.rows, dst.cols, CV_16UC1);//マスク用画像
            cv::Mat dst_mask = cv::Mat::zeros(dst.rows, dst.cols, CV_16UC1);//マスクした出力画像用
            cv::inRange(dst, 0, 1000, mask);//指定した範囲内の画素からなる画像"mask"を生成．範囲外の画素値は0，範囲内はビット深度の最大値となる．
            // cv::bitwise_and(dst, mask, dst_mask);//画素ごとに論理積を取る．
            // cv::normalize(mask, mask, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            // mask.convertTo(dst_mask, CV_8UC1);//8bitにconvert
            //以上　特定の輝度の範囲のみ抽出ーーーーーーーーーー

            double minT2, maxT2;//8bit画像の最大・最小輝度の格納用変数
            dst_mask = mask;
            cv::minMaxLoc(dst_mask, &minT2, &maxT2, NULL, NULL);//8bit画像の最大値と最小値のポインタを得る

            //元画像のデータを表示ーーーーーーーーーーーーーー
            std::cout << "Min Temperature: " << minT << " //Max Temperature: " << maxT << std::endl;
            std::cout << "dims: " << dst.dims << " / depth: " << dst.depth() << " / size: " << dst.size().width 
             << " x " << dst.size().height << " / channel: " << dst.channels() << " / type: " << dst.type() << std::endl;

            std::cout << "Min: " << minT2 << " //Max: " << maxT2 << std::endl;
            std::cout << "dims: " << dst_mask.dims << " / depth: " << dst_mask.depth() << " / size: " << dst_mask.size().width << " x " << dst_mask.size().height << " / channel: " << dst_mask.channels() << " / type " << dst_mask.type() << std::endl;
            
            //.depth() : 0 = CV_8U : 1 = CV_8S : 2 = CV_16U : 3 = CV_16S : 4 = CV_32S : 5 = CV_32F : 6 = CV_64F 
            //.type()  : 0 = CV_8UC1 : 2 = CV_16UC1 : 16 = CV_8UC3 : 18 = CV_16UC3

            sensor_msgs::ImagePtr mess = cv_bridge::CvImage(std_msgs::Header(), "mono16", dst).toImageMsg();//Mat型からsensor_msgs/ImagePtr型へ変換．モノクロ画像なので，encodingはmono8にしてある．
            image_pub.publish(mess);//msgをpublish
            
            cv::imshow("src", dst2);//8bit画像グレースケールを表示
            cv::imshow("dst_eq", dst2_eq);//ヒストグラム均一化後の画像を表示
            cv::imshow("dst_mask", dst_mask);//マスクをした後の画像を表示
            cv::imshow("histogram", hist);//ヒストグラムを表示
            cv::waitKey(1);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "maxinvest");//ノード名の設定
    ImageConverter ic;//ImageConverterコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}