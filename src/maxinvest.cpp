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
                image_sub = it.subscribe("/thermal_image", 1, &ImageConverter::imageCb, this);//Publisherを定義．topic名は/image_raw2
                image_pub = it.advertise("/image_edit", 1);    //Subscriverを定義．
                //topic名は/image_raw，第2引数はキューサイズ，第3引数はトピックを受信したときに呼び出すCallback関数の指定
                cv::namedWindow("src", CV_WINDOW_NORMAL);//ウィンドウをリサイズ可能に設定
                cv::resizeWindow("src", 640, 480);//ウィンドウサイズのリサイズ
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

            double minT, maxT;
            cv::minMaxLoc(src->image, &minT, &maxT, NULL, NULL);
            
            cv::Mat dst = src->image;
            // cv::Mat dst2;

            // dst.convertTo(dst2, CV_32FC1);//32bit浮動小数点に変換
            // cv::normalize(dst2, dst2, 0, 1, cv::NORM_MINMAX);//0~1に正規化
            cv::Mat dst2 = src->image;
            cv::normalize(dst2, dst2, 0, 255, cv::NORM_MINMAX);//次のステップで8bit画像にするため，0~255に正規化．
            dst.convertTo(dst2, CV_8UC1);//8bitにconvert
            
            double minT2, maxT2;//最大・最小輝度の格納用
            cv::minMaxLoc(dst2, &minT2, &maxT2, NULL, NULL);//8bit画像の最大値と最小値を得る

            //画像のデータを得る
            std::cout << "Min: " << minT << " //Max: " << maxT << std::endl;
            std::cout << "dims: " << dst.dims << " / depth: " << dst.depth() << " / size: " << dst.size().width 
             << " x " << dst.size().height << " / channel: " << dst.channels() << " / type " << dst.type() << std::endl;
            //.depth() >> 0 = CV_8U : 1 = CV_8S : 2 = CV_16U : 3 = CV_16S : 4 = CV_32S : 5 = CV_32F : 6 = CV_64F 
            //.type() >> 0 = CV_8UC1 : 2 = CV_16UC1 : 16 = CV_8UC3 : 18 = CV_16UC3

            std::cout << "Min: " << minT2 << " //Max: " << maxT2 << std::endl;
            std::cout << "dims: " << dst2.dims << " / depth: " << dst2.depth() << " / size: " << dst2.size().width 
             << " x " << dst2.size().height << " / channel: " << dst2.channels() << " / type " << dst2.type() << std::endl;

            image_pub.publish(src->toImageMsg());//cv::Mat型からImageConstPtr型へtoImageMsg()で変換してpublish
            
            cv::imshow("src", dst2);//8bit画像グレースケールを表示
            cv::waitKey(1);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "maxinvest");//ノード名の設定
    ImageConverter ic;//ImageConverterコンストラクタの呼び出し．引数はなく，変数icはどんな名前でも良い．
    ros::spin();//subscriber用．新しいメッセージが来るたびにCallback関数を呼び出す
    return 0;
}