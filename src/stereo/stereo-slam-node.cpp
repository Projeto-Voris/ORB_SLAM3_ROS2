#include "stereo-slam-node.hpp"
#include <opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, const std::string &strSettingsFile, const std::string &strDoRectify)
: Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM), node_(node)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify) {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        // Carrega os parâmetros de calibração
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        // Inicializa os mapas de retificação
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    }

    // Cria os subscritores usando o nó passado
    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(node_, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(node_, "camera/right");

    // Sincroniza os subscritores
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
    tf_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
}

StereoSlamNode::~StereoSlamNode() {
    // Para todas as threads
    m_SLAM->Shutdown();

    // Salva a trajetória da câmera
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight) {
    // Copia a imagem RGB da mensagem ROS para cv::Mat
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copia a imagem de profundidade da mensagem ROS para cv::Mat
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    auto sendmsg = geometry_msgs::msg::TransformStamped();
    Sophus::SE3f SE3;
    if (doRectify) {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        SE3 = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    } else {
        SE3 = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    }

    sendmsg.header.stamp = msgLeft->header.stamp;
    sendmsg.header.frame_id = "map";
    sendmsg.child_frame_id = "orbslam3";

    sendmsg.transform.translation.x = SE3.params()(4);
    sendmsg.transform.translation.y = SE3.params()(5);
    sendmsg.transform.translation.z = SE3.params()(6);

    sendmsg.transform.rotation.x = SE3.params()(0);
    sendmsg.transform.rotation.y = SE3.params()(1);
    sendmsg.transform.rotation.z = SE3.params()(2);
    sendmsg.transform.rotation.w = SE3.params()(3);

    tf_publisher->publish(sendmsg);

}







