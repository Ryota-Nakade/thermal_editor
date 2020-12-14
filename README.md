# thermal_editor
ROS drivers for thermal image editing from optris camera

thermal camera : optris piseries PI640
ros topic      : /thermal_image from optris_drivers(http://wiki.ros.org/optris_drivers)

file description
/src
    /correct_distortion_cpp.cpp
        PI640の画像の歪み補正をするノード

    /feature_imager.cpp
        usbカメラの画像からorb特徴点を取得して表示するだけ

    /maxinvest.cpp
        サーモ画像を加工するテスト用

    /thermal_imager.cpp
        サーモの画像から特徴点を検出して表示するだけのノード

    /usb_pub.cpp
        usbカメラの画像をpublishするノード．サブスクライブも可能



