// サーモカメラで撮影した画像の歪みを補正するプログラム
// /thermal_imageトピックをサブスクして，/image_correctedトピックでパブリッシュ
// PI450用
// PI640で使う時は各値を変更すること

#include <math.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
 
class DistortionCorrect{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;

    public:
    // cv::Mat camMat = (cv::Mat_<double>(3,3) << 299.10463, 0.0, 211.75984, 0.0, 296.1873, 146.49607, 0.0, 0.0, 1.0);//カメラの内部パラメータ．(fx, 0, cx, 0, fy, cy, 0, 0, 1)の順．PI450用の値になっている
    // cv::Mat distCoef = (cv::Mat_<double>(1,4) << -0.494721, 0.174556, -0.007342, -0.017139);//歪みパラメータ{k1, k2, p1, p2}．4or5or8個のパラメータをここで設定する（OpneCV準拠）for PI450

    cv::Mat camMat = (cv::Mat_<double>(3,3) << 449.712082, 0.0, 333.955538, 0.0, 449.424017, 253.167613, 0.0, 0.0, 1.0);//カメラの内部パラメータ．(fx, 0, cx, 0, fy, cy, 0, 0, 1)の順．PI640用の値になっている
    cv::Mat distCoef = (cv::Mat_<double>(1,5) << -0.474444, 0.197384, -0.001877, -0.007171, 0.0);//歪みパラメータ{k1, k2, p1, p2}．4or5or8個のパラメータをここで設定する（OpneCV準拠）for PI640

    double k1 = 1.21456239e-05;
double k2 = 1.96249030e-14;
double k3 = 1.65216912e-05;

    cv::Mat map1, map2;//歪み補正マップの宣言

    DistortionCorrect()//コンストラクタ
        : it(nh){
            image_sub = it.subscribe("/thermal_image", 1, &DistortionCorrect::DistortionCo, this);//Subscriberの定義
            image_pub = it.advertise("thermal_image_corrected", 1);//Publisherの定義

            cv::initUndistortRectifyMap(camMat, distCoef, cv::noArray(), cv::noArray(), cv::Size(640, 480), CV_16SC2, map1, map2);//歪み補正マップの算出．cv::remap()とペアで使用．PI450はcv::Sizeを(382,288)に，PI640は(640,480)に変えること
        }
    ~DistortionCorrect() {//デストラクタ
    }

    void DistortionCo(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr src;
        try {
            src = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        cv::Mat input = (src->image).clone();//深いコピー
        cv::Mat dst;//補正後の画像格納用
        cv::remap(input, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());//initundistortRectifyMap()で計算したMapをつかってリマッピング.cv::initundistortRectifyMap()とペアで使用

        // cv::undistort(input, dst, camMat, distCoef, cv::noArray());//initUndistortRectifyMapとremapを使わずに補正可能．しかし動画の補正には非効率．

        sensor_msgs::ImagePtr mess = cv_bridge::CvImage(std_msgs::Header(), "mono16", dst).toImageMsg();//Mat型をMsg型に変換
        image_pub.publish(mess);//補正した画像のPublish
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "thermal_distortion");//ノードの初期化
    DistortionCorrect dt;
    ros::spin();
    return 0;
}
