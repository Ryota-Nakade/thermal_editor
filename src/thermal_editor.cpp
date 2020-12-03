#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>//GUI modules
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>//arrow you to publish&subscribe images in ROS
#include <sensor_msgs/image_encodings.h>

// 12/3: uvc_cameraからの/image_rawトピックを受取り，ROSにトピックpublish

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
                image_sub = it.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);//Publisherを定義．topic名は/image_raw2
                image_pub = it.advertise("/image_edit", 1);    //Subscriverを定義．
                //topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                cv::namedWindow("hsv", CV_WINDOW_NORMAL);
                cv::namedWindow("gray", CV_WINDOW_NORMAL);
                cv::resizeWindow("hsv", 640, 480);//ウィンドウサイズのリサイズ
                cv::resizeWindow("gray", 640, 480);//ウィンドウサイズのリサイズ
            }

        //デストラクタ
        ~ImageConverter() {
            cv::destroyWindow("hsv");
            cv::destroyWindow("gray");
        }

        //Callback関数
        void imageCb(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr cv_ptr, cv_ptr2, cv_ptr3;
            try {
                cv_ptr  = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
                cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//sensor_msgs::ImageConstPtr型からcv::Mat型に変換
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Mat hsv, gray;

            cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);//hsv形式に変換
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);//grayスケールに変換
            cv::Canny(gray, gray, 15.0, 30.0, 3);//Cannyエッジ検出

            cv::imshow("hsv", hsv);
            cv::imshow("gray", gray);
            cv::waitKey(1);

            image_pub.publish(cv_ptr3->toImageMsg());//cv::Mat型からImageConstPtr型へtoImageMsg()で変換してpublish
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "thermal_editor");//ノード名の設定
    ImageConverter ic;//ImageConverterコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}