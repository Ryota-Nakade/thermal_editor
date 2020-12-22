//　Optris PI640用のディストーション補正プログラム（長谷川さん謹製）
//　/thermal_imageトピックをサブスクして，/image_correctedトピックでパブリッシュ
//　PI450では歪係数が異なるので，これは使えない

#include <math.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
//#include <optris_drivers/Image_Points.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

image_transport::Publisher* _pub;

// Consider y coefficient
double k1 = 1.21456239e-05;
double k2 = 1.96249030e-14;
double k3 = 1.65216912e-05;
double k4 = 1.53712821e-11;
double alpha = 9.88930697e-01;

int width = 938;
int height = 606;

cv::Mat input8(480, 640, CV_8UC1);
cv::Mat img(height, width, CV_8UC1);
uint cnt = 0;

void onDataReceive(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
        int maxVal = 0;
        int minVal = 66000;
        for (int i = 0; i < input.rows; i++) {//画像の最大値と最小値を求める
            for (int j = 0; j < input.cols; j++) {
                int val = input.at<ushort>(i, j);
                maxVal = max(maxVal, val);
                minVal = min(minVal, val);
            }
        }
        for (int i = 0; i < input.rows; i++) {
            for (int j = 0; j < input.cols; j++) {
                input8.at<uchar>(i, j) = 255 * (input.at<ushort>(i, j) - minVal + 1) / (maxVal - minVal);//16bitから8bitに正規化
            }
        }

        for (int i = 0; i < img.rows; i++) {
            uchar *out = img.ptr<uchar>(i);
            for (int j = 0; j < img.cols; j++) {
                double x1 = j - img.cols / 2;
                double y1 = (img.rows / 2 - i) * alpha;
                double r_2 = x1 * x1 + y1 * y1;
                double r_4 = r_2 * r_2;
                double x2 = x1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4);
                double y2 = y1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4);
                int ii = input.rows / 2 - (int)round(y2);
                int jj = input.cols / 2 + (int)round(x2);
                if (0 <= ii && ii < input.rows && 0 <= jj && jj < input.cols) {
                    uchar val = (uchar)((input.at<ushort>(ii, jj) - minVal) * 255 / (maxVal - minVal));
                    out[j] = val;
                }
            }
        }

		_pub->publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, img).toImageMsg());//Mat型からMsg型に変換してPublish
    }
    catch (cv_bridge::Exception &e) {//エラー処理
        ROS_ERROR("cv_beidge exception: %s", e.what());
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "optris_correct_distortion");//初期化して，ノード名の宣言とか・・・

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;
    image_transport::ImageTransport it(n_);
    ros::Subscriber subThermal = n.subscribe("/thermal_image", 1, onDataReceive);//Subscriberの宣言
    // image_transport::Publisher pubt = it.advertise("thermal_corrected", 1);
    image_transport::Publisher pubt = it.advertise("thermal_optris_correct", 1);//Publisherの宣言
    _pub = &pubt;

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
