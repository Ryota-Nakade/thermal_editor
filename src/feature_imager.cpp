#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

// 12/3: uvc_cameraからの/image_rawトピックを受取り，ROSにトピックpublish
//　/image_rawを受取り，特徴点検出してpublishするノード

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
                image_sub = it.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);//Publisherを定義．topic名は/image_raw2
                image_pub = it.advertise("/image_edit", 1);    //Subscriverを定義．
                //topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                cv::namedWindow("src", CV_WINDOW_NORMAL);
                cv::namedWindow("gray", CV_WINDOW_NORMAL);
                cv::resizeWindow("src", 640, 480);//ウィンドウサイズのリサイズ
                cv::resizeWindow("gray", 640, 480);//ウィンドウサイズのリサイズ
            }

        //デストラクタ
        ~ImageConverter() {
            cv::destroyWindow("src");
            cv::destroyWindow("gray");
        }

        //Callback関数
        void imageCb(const sensor_msgs::ImageConstPtr &msg) {
            // cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
            cv_bridge::CvImagePtr cv_ptr, src;
            try {
                cv_ptr  = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
                src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                // cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            //ORB抽出器
            std::vector<cv::KeyPoint> keypoint1;//特徴点を格納する領域を用意
            cv::Mat descriptor1;
            cv::Ptr<cv::ORB> feature = cv::ORB::create(2000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);//ORBオブジェクト生成
            feature->detectAndCompute(src->image, cv::noArray(), keypoint1, descriptor1);

            //特徴点表示のときの四角の大きさ．中心からこの座標値離れた位置に四角の頂点が来る
            cv::Point2f rec {5.0, 5.0};

            for (double i = 0; i < keypoint1.size(); i++) {
                cv::KeyPoint *point = &(keypoint1[i]);
                //cv::circle(src, itk->pt, 6, cv::Scalar(0,255,255), 1);//円で表示
                cv::rectangle(src->image, point->pt - rec, point->pt + rec, cv::Scalar(0, 255, 255), 1);//四角で表示
            }

            cv::Mat gray;
            
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);//grayスケールに変換
            cv::Canny(gray, gray, 15.0, 150.0, 3);//Cannyエッジ検出

            cv::imshow("src", src->image);
            cv::imshow("gray", gray);
            cv::waitKey(1);

            std::cout << "特徴点数:" << descriptor1.rows << " / 次元数:" << descriptor1.cols << std::endl;
            image_pub.publish(src->toImageMsg());//cv::Mat型からImageConstPtr型へtoImageMsg()で変換してpublish
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_imager");//ノード名の設定
    ImageConverter ic;//ImageConverterコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}