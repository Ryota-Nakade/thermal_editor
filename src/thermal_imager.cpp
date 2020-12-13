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
                image_sub = it.subscribe("/thermal_image_edit", 1, &ImageConverter::imageCb, this);//Subscriberを定義．topic名は/thermal_image
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

            cv::Mat dst = src->image;
            double minT, maxT;//元画像の最大値と最小値の格納用変数
            // cv::minMaxLoc(dst, &minT, &maxT, NULL, NULL);//元画像の最大値と最小値のポインタを得る
            cv::minMaxLoc(src->image, &minT, &maxT, NULL, NULL);//元画像の最大値と最小値のポインタを得る

            // cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            cv::normalize(src->image, src->image, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            // dst.convertTo(dst, CV_8UC1);//8bitにconvert
            dst.convertTo(src->image, CV_8UC1);//8bitにconvert

            //ORB抽出器
            std::vector<cv::KeyPoint> keypoint1;//特徴点を格納する領域を用意
            cv::Mat descriptor1;//特徴記述子を格納する領域
            cv::Ptr<cv::ORB> feature = cv::ORB::create(2000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);//ORBオブジェクト生成．第1引数は抽出する特徴点の上限
            // feature->detectAndCompute(dst, cv::noArray(), keypoint1, descriptor1);
            feature->detectAndCompute(src->image, cv::noArray(), keypoint1, descriptor1);

            //特徴点表示のときの四角の大きさ．中心からこの座標値離れた位置に四角の頂点が来る
            cv::Point2f rec {5.0, 5.0};

            for (double i = 0; i < keypoint1.size(); i++) {
                cv::KeyPoint *point = &(keypoint1[i]);
                //cv::circle(src, itk->pt, 6, cv::Scalar(0,255,255), 1);//円で表示
                // cv::rectangle(dst, point->pt - rec, point->pt + rec, cv::Scalar(0, 255, 255), 1);//四角で表示
                cv::rectangle(src->image, point->pt - rec, point->pt + rec, cv::Scalar(0, 255, 255), 1);//四角で表示
            }

            //元画像のデータを表示
            std::cout << "Min: " << minT << " //Max: " << maxT << std::endl;
            std::cout << "dims: " << dst.dims << " / depth: " << dst.depth() << " / size: " << dst.size().width 
             << " x " << dst.size().height << " / channel: " << dst.channels() << " / type " << dst.type() << std::endl;

            cv::imshow("ORB", src->image);//ORB特徴点検出結果を表示

            std::cout << "特徴点数:" << descriptor1.rows << " / 次元数:" << descriptor1.cols << std::endl;
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