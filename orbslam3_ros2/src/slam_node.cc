#include "slam_node.hpp"


namespace orbslam3_ros2
{
// Define static const matrices declared in header
const tf2::Matrix3x3 SlamNode::tf_orb_to_ros_default(
    1.0, 0.0, 0.0,   // row 0
    0.0, 1.0, 0.0,   // row 1
    0.0, 0.0, 1.0);  // row 2

const tf2::Matrix3x3 SlamNode::tf_orb_to_ros_enu(
    0.0, 0.0, 1.0,   // row 0
   -1.0, 0.0, 0.0,   // row 1
    0.0,-1.0, 0.0);  // row 2
    

SlamNode::SlamNode(ORB_SLAM3::System* pSLAM, const rclcpp::NodeOptions & options)
: Node("ORB_SLAM3", options), m_SLAM(pSLAM)
{
    pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    pathpublisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    posepublisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_cov", 10);
    trackedpublisher = this->create_publisher<sensor_msgs::msg::Image>("tracked_image", 10);
    status_publisher = this->create_publisher<orbslam3_msgs::msg::SlamStatus>("slam_status", 10);
    resetservice = this->create_service<std_srvs::srv::Trigger>("reset", std::bind(&SlamNode::handleReset, this, std::placeholders::_1, std::placeholders::_2));
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    this->declare_parameter("frame_id", "orbslam3");
    this->declare_parameter("parent_frame_id", "SM2/base_link");
    this->declare_parameter("child_frame_id", "SM2/left_camera_link");
    this->declare_parameter("tracked_points", false);
    this->declare_parameter("ENU_publish", false);
    this->declare_parameter<bool>("tf_publish", true);
    
}
SlamNode::~SlamNode() {
    // Para todas as threads
    SlamNode::SaveData();

}
void SlamNode::handleReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    m_SLAM->Reset();
    m_SLAM->ResetActiveMap();
    response->success = true;
    response->message = "SLAM reseted";
}
void SlamNode::Update(){
    current_frame_time_ = now();
    if (!tf_static_cached_) {
        std::string base_frame_ = this->get_parameter("parent_frame_id").as_string();
        std::string cam_frame_ = this->get_parameter("child_frame_id").as_string();
        try {
            auto tf_base_cam = tf_buffer_->lookupTransform(
                base_frame_, cam_frame_, tf2::TimePointZero);
            tf2::fromMsg(tf_base_cam.transform, T_base_cam_);
            tf_static_cached_ = true;
            RCLCPP_INFO(this->get_logger(), "Static transform [%s -> %s] successfully cached!", base_frame_.c_str(), cam_frame_.c_str());
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Wait static TF to be available: %s", ex.what());
            return; // Retorna cedo pois não podemos publicar path/poses corretos sem essa TF
        }
    }
    int state_num = m_SLAM->GetTrackingState();
    int map_id = m_SLAM->GetMapID();
    bool map_changed = m_SLAM->MapChanged();

    orbslam3_msgs::msg::SlamStatus status_msg;
    status_msg.header.stamp = current_frame_time_;
    status_msg.header.frame_id = this->get_parameter("frame_id").as_string();
    status_msg.tracking_state = static_cast<int8_t>(state_num);
    status_msg.map_id = map_id;
    status_msg.map_changed = map_changed;
    status_publisher->publish(status_msg);
    
    PublishTransform();
    PublishTrackedPointCloud();
    PublishPose();
    PublishPath();

}

void SlamNode::TrackedImage(const cv::Mat image) {
    // 1. INPUT CHECK: If the source image is empty, stop immediately.
    if (image.empty()) {
        return; 
    }

    bool tracked_image = this->get_parameter("tracked_points").as_bool();

    if (tracked_image) {
        std::vector<cv::KeyPoint> keypoints = m_SLAM->GetTrackedKeyPointsUn();
        
        cv::Mat out_image;
        
        // Draw keypoints. If 'image' is Mono8, 'out_image' becomes BGR automatically.
        cv::drawKeypoints(image, keypoints, out_image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

        // 2. OUTPUT CHECK: Ensure the result has data before publishing
        if (!out_image.empty()) {
            auto imgmsg = std::make_unique<sensor_msgs::msg::Image>();
            // Note: Ideally, pass the original message header here so timestamps match
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", out_image).toImageMsg(*imgmsg);
            trackedpublisher->publish(std::move(imgmsg));
        } else {
             RCLCPP_WARN(this->get_logger(), "TrackedImage: Resulting image was empty.");
        }
    }
}

void SlamNode::PublishTrackedPointCloud(){
    std::vector<int> indexes;
    std::vector<ORB_SLAM3::MapPoint*> points = m_SLAM->GetAllMapPoints();
    
    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();

    int count = 0;
    
    for (size_t i = 0; i < points.size(); i++)
    {
        if(points[i] != 0){
            count++;
            indexes.push_back(i);

        }
    }
    
    pointcloudmsg.header.stamp = current_frame_time_;
    pointcloudmsg.header.frame_id = this->get_parameter("frame_id").as_string();
    pointcloudmsg.height = 1;
    pointcloudmsg.width = count;
    pointcloudmsg.is_dense = true;
    pointcloudmsg.fields.resize(3);

    // Populate the fields
    pointcloudmsg.fields[0].name = "x";
    pointcloudmsg.fields[0].offset = 0;
    pointcloudmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[0].count = 1;

    pointcloudmsg.fields[1].name = "y";
    pointcloudmsg.fields[1].offset = 4;
    pointcloudmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[1].count = 1;

    pointcloudmsg.fields[2].name = "z";
    pointcloudmsg.fields[2].offset = 8;
    pointcloudmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[2].count = 1;

    pointcloudmsg.point_step = 12; // Size of a single point in bytes (3 floats * 4 bytes/float)
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
    pointcloudmsg.is_bigendian = false;
    pointcloudmsg.data.resize(pointcloudmsg.point_step*count);

   tf2::Vector3 cam_offset = T_base_cam_.getOrigin(); 
    
    tf2::Matrix3x3 tf_orb_to_ros = this->get_parameter("ENU_publish").as_bool() ? tf_orb_to_ros_enu : tf_orb_to_ros_default;

    // Se houver offset inicial, calculamos a translação dele fora do loop para economizar CPU
    tf2::Vector3 initial_offset_translation(0, 0, 0);
    if (initial_offset_set_) {
        // Ordem correta: Matriz * Vetor
        initial_offset_translation = tf_orb_to_ros * initial_map_base_offset_.inverse().getOrigin();
    }

    for (size_t i = 0; i < count; i++)
    {
        // 1. Extrai o ponto no referencial do ORB-SLAM (Z frente, X direita, Y baixo)
        float x = points[indexes[i]]->GetWorldPos()(0);
        float y = points[indexes[i]]->GetWorldPos()(1);
        float z = points[indexes[i]]->GetWorldPos()(2);
        tf2::Vector3 pt_orb(x, y, z);
        
        // 2. Gira o ponto para o padrão ROS (X frente, Y esquerda, Z cima)
        tf2::Vector3 pt_ros = tf_orb_to_ros * pt_orb;

        // 3. Adiciona o deslocamento da câmera para a base
        // (Move o "0,0,0" do mapa da lente da câmera para o chão/centro do robô)
        tf2::Vector3 pt_base = pt_ros + cam_offset;

        // 5. Copia os bytes para a mensagem PointCloud2
        float final_x = pt_base.x();
        float final_y = pt_base.y();
        float final_z = pt_base.z();
        
        memcpy(&pointcloudmsg.data[i*12], &final_x, 4);
        memcpy(&pointcloudmsg.data[i*12 + 4], &final_y, 4);
        memcpy(&pointcloudmsg.data[i*12 + 8], &final_z, 4);
    }
    
    pclpublisher->publish(pointcloudmsg);
}

void SlamNode::PublishCurrentPointCloud(){
    std::vector<int> indexes;
    std::vector<ORB_SLAM3::MapPoint*> points = m_SLAM->GetTrackedMapPoints();
    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();
 

    int count = 0;
    
    for (size_t i = 0; i < points.size(); i++)
    {
        if(points[i] != 0){
            count++;
            indexes.push_back(i);

        }
    }
    
    pointcloudmsg.header.stamp = current_frame_time_;
    pointcloudmsg.header.frame_id = "orbslam3";//this->get_parameter("frame_id").as_string();
    pointcloudmsg.height = 1;
    pointcloudmsg.width = count;
    pointcloudmsg.is_dense = true;
    pointcloudmsg.fields.resize(3);

    // Populate the fields
    pointcloudmsg.fields[0].name = "x";
    pointcloudmsg.fields[0].offset = 0;
    pointcloudmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[0].count = 1;

    pointcloudmsg.fields[1].name = "y";
    pointcloudmsg.fields[1].offset = 4;
    pointcloudmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[1].count = 1;

    pointcloudmsg.fields[2].name = "z";
    pointcloudmsg.fields[2].offset = 8;
    pointcloudmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[2].count = 1;

    pointcloudmsg.point_step = 12; // Size of a single point in bytes (3 floats * 4 bytes/float)
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
    pointcloudmsg.is_bigendian = false;
    pointcloudmsg.data.resize(pointcloudmsg.point_step*count);

    for (size_t i = 0; i < count; i++)
    {
        float x = points[indexes[i]]->GetWorldPos()(0);
        float y = points[indexes[i]]->GetWorldPos()(1);
        float z = points[indexes[i]]->GetWorldPos()(2);

        memcpy(&pointcloudmsg.data[i*12], &x, 4);
        memcpy(&pointcloudmsg.data[i*12 + 4], &y, 4);
        memcpy(&pointcloudmsg.data[i*12 + 8], &z, 4);
    }
    pclpublisher->publish(pointcloudmsg);
}

void SlamNode::PublishPath(){
    std::vector<ORB_SLAM3::KeyFrame*> trajectory = m_SLAM->GetTrajectory();
    auto path_msg = nav_msgs::msg::Path();

    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = this->get_parameter("frame_id").as_string();

    tf2::Transform T_cam_base = T_base_cam_.inverse();

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        Sophus::SE3f SE3_kf = trajectory[i]->GetPose();
        pose.header.stamp = current_frame_time_;
        pose.header.frame_id = path_msg.header.frame_id;

        // 1. SLAM output: map → camera_link
        Sophus::SE3f T_map_t_cam = SE3_kf.inverse();
        tf2::Transform T_map_cam = TransformFromSophus(T_map_t_cam);

        // 2. Compose: map → base (usando o cache direto)
        tf2::Transform T_map_base = T_map_cam * T_cam_base;
        
        // 3. Aplica o offset inicial
        tf2::Transform T_map_base_zeroed = initial_map_base_offset_.inverse() * T_map_base;
        tf2::toMsg(T_map_base_zeroed, pose.pose);

        path_msg.poses.push_back(pose);
    }
    pathpublisher->publish(path_msg);
}

void SlamNode::PublishPose() {
    // 1. SLAM output: map → camera_link
    Sophus::SE3f T_map_t_cam = SE3.inverse();
    tf2::Transform T_map_cam = TransformFromSophus(T_map_t_cam);

    // 2. Usa a transformada do cache
    tf2::Transform T_cam_base = T_base_cam_.inverse();

    // 3. Compose: map → base
    tf2::Transform T_map_base = T_map_cam * T_cam_base;
    tf2::Transform T_map_base_zeroed = initial_map_base_offset_.inverse() * T_map_base;

    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_msg.header.stamp = current_frame_time_;
    pose_msg.header.frame_id = this->get_parameter("frame_id").as_string();
    tf2::toMsg(T_map_base_zeroed, pose_msg.pose.pose);

    int state = m_SLAM->GetTrackingState();
    std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);

    if(state == 2 || state == 5){
        // Tracking, covariance ok (slam is not the best pose estimator)
        pose_msg.pose.covariance[0] = 0.05;
        pose_msg.pose.covariance[7] = 0.05;
        pose_msg.pose.covariance[14] = 0.05;
        pose_msg.pose.covariance[21] = 0.1;
        pose_msg.pose.covariance[28] = 0.1;
        pose_msg.pose.covariance[35] = 0.1;
    }
    if(state == 3){
        // Recently lost, may find itself again soon
        pose_msg.pose.covariance[0] = 1.0;
        pose_msg.pose.covariance[7] = 1.0;
        pose_msg.pose.covariance[14] = 1.0;
        pose_msg.pose.covariance[21] = 1.0;
        pose_msg.pose.covariance[28] = 1.0;
        pose_msg.pose.covariance[35] = 1.0;
    }
    if(state == -1 || state == 0 || state == 1 || state == 4){
        // If slam is lost
        pose_msg.pose.covariance[0] = -1;
        pose_msg.pose.covariance[7] = -1;
        pose_msg.pose.covariance[14] = -1;
        pose_msg.pose.covariance[21] = -1;
        pose_msg.pose.covariance[28] = -1;
        pose_msg.pose.covariance[35] = -1;
        pose_msg.pose.covariance[35] = -1;
    }

    posepublisher->publish(pose_msg);
}   

void SlamNode::PublishTransform(){
    try {
            // 1. SLAM output: map → camera_link
            Sophus::SE3f T_map_t_cam = SE3.inverse();
            tf2::Transform T_map_cam = TransformFromSophus(T_map_t_cam);

            // 2. Usa a transformada do cache
            tf2::Transform T_cam_base = T_base_cam_.inverse();

            // 3. Compose: map → base
            tf2::Transform T_map_base = T_map_cam * T_cam_base;

            // Set initial offset if not set
            if (!initial_offset_set_) {
                initial_map_base_offset_.setIdentity();
                initial_map_base_offset_.setOrigin(T_map_base.getOrigin());
                initial_offset_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Publishing transform as ENU: %s", 
                            this->get_parameter("ENU_publish").as_bool() ? "true" : "false");
            }

            // Apply the offset so that base_link starts at (0,0,0) in map
            tf2::Transform T_map_base_zeroed = initial_map_base_offset_.inverse() * T_map_base;

            // 4. Broadcast: map → base
            if (this->get_parameter("tf_publish").as_bool()) {
                geometry_msgs::msg::TransformStamped sendmsg;
                sendmsg.header.stamp = current_frame_time_;
                sendmsg.header.frame_id = this->get_parameter("frame_id").as_string();
                sendmsg.child_frame_id = this->get_parameter("parent_frame_id").as_string();
                tf2::toMsg(T_map_base_zeroed, sendmsg.transform);

                tf_broadcaster_->sendTransform(sendmsg);
            }

        } catch (const std::exception& ex) {
            // Mudamos para std::exception pois não estamos mais fazendo chamadas exclusivas de TF que lançam TransformException aqui
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Erro ao calcular a transformação map → base: %s", ex.what());
        }
}

tf2::Transform SlamNode::TransformFromSophus(Sophus::SE3f &pose)
{
    // Convert pose to double precision for tf2 compatibility
    Eigen::Matrix3d rotation = pose.rotationMatrix().cast<double>();
    Eigen::Vector3d translation = pose.translation().cast<double>();

    bool enu_publish = this->get_parameter("ENU_publish").as_bool();

    tf2::Matrix3x3 tf_orb_to_ros = enu_publish ? tf_orb_to_ros_enu : tf_orb_to_ros_default;
    
    tf2::Matrix3x3 tf_camera_rotation(
        rotation(0, 0), rotation(0, 1), rotation(0, 2),
        rotation(1, 0), rotation(1, 1), rotation(1, 2),
        rotation(2, 0), rotation(2, 1), rotation(2, 2));
        tf2::Vector3 tf_camera_translation(
            translation(0), translation(1), translation(2));

    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Return the final tf2::Transform
    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

void SlamNode::CreateDirectoryIfNotExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        mkdir(path.c_str(), 0777);
    }
}

void SlamNode::SaveData() {
    if(!m_SLAM) return;
    
    const std::string output_dir = std::string(getenv("HOME")) + "/Documents/results/";
    CreateDirectoryIfNotExists(output_dir);
    
    // Salva todos os dados implementados
    m_SLAM->SaveMapPoints(output_dir + "map_points.ply");
    m_SLAM->SaveKeyFrameTrajectory(output_dir + "keyframe_trajectory.txt");
    m_SLAM->SaveTrajectoryKITTI(output_dir + "trajectory_kitti.txt");
}
}