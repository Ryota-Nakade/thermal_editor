#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

// 12/3: uvc_cameraからの/image_rawトピックを受取り，ROSにトピックpublish
// 16bit-thermal画像を8bitに変換した後，特徴点抽出を行う

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
                image_sub = it.subscribe("/thermal_image", 1, &ImageConverter::imageCb, this);//Subscriberを定義．topic名は/thermal_image
                image_pub = it.advertise("/thermal_image_feature", 1);    //Publisherを定義．
                //topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                cv::namedWindow("ORB", CV_WINDOW_NORMAL);
                cv::resizeWindow("ORB", 640, 480);//ウィンドウサイズのリサイズ
            }

        //デストラクタ
        ~ImageConverter() {
            cv::destroyWindow("ORB");
        }

        //Callback関数
        void imageCb(const sensor_msgs::ImageConstPtr &msg) {
            // cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
            cv_bridge::CvImagePtr cv_ptr, src;
            try {
                //cv_ptr  = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
                src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);//sensor_msgs::ImageConstPtr型から16bitのcv::Mat型に変換
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Mat dst = (src->image).clone();
            double minT, maxT;//元画像の最大値と最小値の格納用変数
            // cv::minMaxLoc(dst, &minT, &maxT, NULL, NULL);//元画像の最大値と最小値のポインタを得る
            cv::minMaxLoc(dst, &minT, &maxT, NULL, NULL);//元画像の最大値と最小値のポインタを得る

            // cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            // dst.convertTo(dst, CV_8UC1);//8bitにconvert
            dst.convertTo(dst, CV_8UC1);//8bitにconvert

            // 以下ヒストグラム均一化ーーーーーーーーーーーーーーーーー
            // std::vector<cv::Scalar> cScalar{cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255)};//表示するヒストグラムの色を指定
            // cv::Mat hist(cv::Size(256, 256), CV_8UC3);//ヒストグラムのサイズを指定．ここでは256 x 256px で3チャンネル
            // cv::Mat dst2_eq;//ヒストグラム均一化後の画像を格納する領域
            cv::equalizeHist(dst, dst);//dst2をヒストグラム均一化してdst2_eqに入力
            cv::Mat dst_color = dst.clone();
            cv::cvtColor(dst_color, dst_color, cv::COLOR_GRAY2BGR);

            // // ヒストグラム表示ーーーーーーーーーーーーーーーーーーー
            // std::vector<cv::Mat> ch{dst2, dst2_eq};
            // std::vector<int> size{256};//binの本数．つまりヒストグラムの縦棒の数
            // std::vector<float> range{0, 256};//ヒストグラムの値の範囲．左の値”以上”，右の値”未満”
            // hist.setTo(255);
            // for (int c = 0; c < 2; c++ ) {
            //     std::vector<int> channel{c};//チャンネル指定用
            //     cv::Mat bins;//ヒストグラムの結果格納用配列
            //     cv::calcHist(ch, channel, cv::noArray(), bins, size, range);
            //     cv::Point prev = cv::Point(0, 255);
            //     for (int x = 0; x < 256; x++) {
            //         cv::Point current = cv::Point(x,256 - (int) (bins.at<float>(x) / 75 ));//75で割って大きさ調整する
            //         cv::line(hist, prev, current, cScalar[c]);
            //         prev = current;
            //     }
            // }
            // //以上　ヒストグラム関係ーーーーーーーーーーーーー

            //ORB抽出器
            std::vector<cv::KeyPoint> keypoint1;//特徴点を格納する領域を用意
            cv::Mat descriptor1;//特徴記述子を格納する領域
            cv::Ptr<cv::ORB> feature = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);//ORBオブジェクト生成．第1引数は抽出する特徴点の上限
            // feature->detectAndCompute(dst, cv::noArray(), keypoint1, descriptor1);
            feature->detectAndCompute(dst, cv::noArray(), keypoint1, descriptor1);

            //特徴点表示のときの四角の大きさ．中心からこの座標値離れた位置に四角の頂点が来る
            cv::Point2f rec {5.0, 5.0};

            for (double i = 0; i < keypoint1.size(); i++) {
                cv::KeyPoint *point = &(keypoint1[i]);
                //cv::circle(src, itk->pt, 6, cv::Scalar(0,255,255), 1);//円で表示
                // cv::rectangle(dst, point->pt - rec, point->pt + rec, cv::Scalar(0, 255, 255), 1);//四角で表示
                cv::rectangle(dst_color, point->pt - rec, point->pt + rec, cv::Scalar(0, 255, 255), 1);//四角で表示
            }

            //元画像のデータを表示
            // std::cout << "Min: " << minT << " //Max: " << maxT << std::endl;
            // std::cout << "dims: " << dst.dims << " / depth: " << dst.depth() << " / size: " << dst.size().width 
            //  << " x " << dst.size().height << " / channel: " << dst.channels() << " / type " << dst.type() << std::endl;

            cv::imshow("ORB", dst_color);//ORB特徴点検出結果を表示

            std::cout << "特徴点数:" << descriptor1.rows << std::endl; 
            std::cout << " / 次元数:" << descriptor1.cols << " / row :" << descriptor1.rows << std::endl;
            sensor_msgs::ImagePtr mess = cv_bridge::CvImage(std_msgs::Header(), "mono8", src->image).toImageMsg();//Mat型からsensor_msgs/ImagePtr型へ変換．モノクロ画像なので，encodingはmono8にしてある．
            image_pub.publish(mess);//msgをpublish

            cv::waitKey(1);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_imager");//ノード名の設定
    ImageConverter ic;//ImageConverterコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}