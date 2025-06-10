#include "slam_node.hpp"

SlamNode::SlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node)
: Node("ORB_SLAM3_Inertial"), m_SLAM(pSLAM), node_(node)
{
    // tf_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
    pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    pathpublisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    posepublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    statepublisher = this->create_publisher<std_msgs::msg::String>("state", 10);
    flagpublisher = this->create_publisher<std_msgs::msg::Bool>("flag", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    this->declare_parameter("frame_id", "orbslam3");
    this->declare_parameter("parent_frame_id", "SM2/base_link");
    this->declare_parameter("child_frame_id", "SM2/left_camera_link");

}
SlamNode::~SlamNode() {
    // Para todas as threads
    m_SLAM->Shutdown();
}
void SlamNode::Update(){
    current_frame_time_ = now();
    int state_num = m_SLAM->GetTrackingState();
    
    auto statemsg = std_msgs::msg::String();
    switch (state_num)
    {
    case -1:
        statemsg.data = "SYSTEM_NOT_READY";
        break;
    case 0:
        statemsg.data = "NO_IMAGES_YET";
        break;
    case 1:
        statemsg.data = "NOT_INITIALIZED";
        break;
    case 2:
        statemsg.data = "OK";
        break;
    case 3:
        statemsg.data = "RECENTLY_LOST";
        break;
    case 4:
        statemsg.data = "LOST";
        break;
    case 5:
        statemsg.data = "OK_KLT";
        break;
    }
    
    auto flagmsg = std_msgs::msg::Bool();
    flagmsg.data = m_SLAM->MapChanged();

    flagpublisher->publish(flagmsg);
    statepublisher->publish(statemsg);
    PublishTransform();
    PublishTrackedPointCloud();
    PublishPose();
    PublishPath();
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
    pointcloudmsg.header.frame_id = "orbslam3";
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
    path_msg.header.frame_id = this->get_parameter("frame_id").as_string();;

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        Sophus::SE3f SE3 =  trajectory[i]->GetPose();
        pose.header.stamp = current_frame_time_;
        pose.header.frame_id = this->get_parameter("frame_id").as_string();;

        // Transform to ROS coordinates
        // 1. SLAM output: map → camera_link
        Sophus::SE3f T_cam_map = SE3;
        Sophus::SE3f T_map_t_cam = T_cam_map.inverse();

        tf2::Transform T_map_cam = TransformFromSophus(T_map_t_cam);
        // T_map_cam = T_map_cam.inverse();

        // 2. Static transform: base_link → camera_link from TF tree
        std::string base_frame_ = this->get_parameter("parent_frame_id").as_string();
        std::string cam_frame_ = this->get_parameter("child_frame_id").as_string();
        std::string map_frame_ = this->get_parameter("frame_id").as_string();
        auto tf_base_to_cam = tf_buffer_->lookupTransform(
            base_frame_, cam_frame_, tf2::TimePointZero);
        tf2::Transform T_base_cam;
        tf2::fromMsg(tf_base_to_cam.transform, T_base_cam);

        // 3. Compute inverse: camera → base
        tf2::Transform T_cam_base = T_base_cam.inverse();

        // 4. Compose: map → base
        // Apply the offset so that base_link starts at (0,0,0) in map
        tf2::Transform T_map_base = T_map_cam * T_cam_base;
        tf2::Transform T_map_base_zeroed = initial_map_base_offset_.inverse() * T_map_base;
        tf2::toMsg(T_map_base_zeroed, pose.pose);

        path_msg.poses.push_back(pose);

    }
    pathpublisher->publish(path_msg);
    
}

void SlamNode::PublishPose() {

        // 1. SLAM output: map → camera_link
        Sophus::SE3f T_cam_map = SE3;
        Sophus::SE3f T_map_t_cam = T_cam_map.inverse();

        tf2::Transform T_map_cam = TransformFromSophus(T_map_t_cam);
        // T_map_cam = T_map_cam.inverse();

        // 2. Static transform: base_link → camera_link from TF tree
        std::string base_frame_ = this->get_parameter("parent_frame_id").as_string();
        std::string cam_frame_ = this->get_parameter("child_frame_id").as_string();
        std::string map_frame_ = this->get_parameter("frame_id").as_string();
        auto tf_base_to_cam = tf_buffer_->lookupTransform(
            base_frame_, cam_frame_, tf2::TimePointZero);
        tf2::Transform T_base_cam;
        tf2::fromMsg(tf_base_to_cam.transform, T_base_cam);

        // 3. Compute inverse: camera → base
        tf2::Transform T_cam_base = T_base_cam.inverse();

        // 4. Compose: map → base
        tf2::Transform T_map_base = T_map_cam * T_cam_base;
        tf2::Transform T_map_base_zeroed = initial_map_base_offset_.inverse() * T_map_base;

    auto pose_msg = geometry_msgs::msg::PoseStamped();

    pose_msg.header.stamp = current_frame_time_;
    pose_msg.header.frame_id = this->get_parameter("frame_id").as_string();;
    tf2::toMsg(T_map_base_zeroed, pose_msg.pose);

    posepublisher->publish(pose_msg);

}

// void SlamNode::PublishTransform(){
//     auto sendmsg = geometry_msgs::msg::TransformStamped();
//     tf2::Transform grasp_tf = TransformFromSophus(SE3);

//     sendmsg.header.stamp = current_frame_time_;
//     sendmsg.header.frame_id = this->get_parameter("frame_id").as_string();;
//     sendmsg.child_frame_id = this->get_parameter("child_frame_id").as_string();;
//     tf2::toMsg(grasp_tf, sendmsg.transform);

//     // tf_publisher->publish(sendmsg);
//     tf_broadcaster_->sendTransform(sendmsg);
// }

void SlamNode::PublishTransform()
{
    try {
        // 1. SLAM output: map → camera_link
        Sophus::SE3f T_cam_map = SE3;
        Sophus::SE3f T_map_t_cam = T_cam_map.inverse();

        tf2::Transform T_map_cam = TransformFromSophus(T_map_t_cam);
        // T_map_cam = T_map_cam.inverse();



        // 2. Static transform: base_link → camera_link from TF tree
        std::string base_frame_ = this->get_parameter("parent_frame_id").as_string();
        std::string cam_frame_ = this->get_parameter("child_frame_id").as_string();
        std::string map_frame_ = this->get_parameter("frame_id").as_string();
        auto tf_base_to_cam = tf_buffer_->lookupTransform(
            base_frame_, cam_frame_, tf2::TimePointZero);
        tf2::Transform T_base_cam;
        tf2::fromMsg(tf_base_to_cam.transform, T_base_cam);

        // 3. Compute inverse: camera → base
        tf2::Transform T_cam_base = T_base_cam.inverse();

        // 4. Compose: map → base
        tf2::Transform T_map_base = T_map_cam * T_cam_base;

                        // Set initial offset if not set
        if (!initial_offset_set_) {
            initial_map_base_offset_.setIdentity();
            initial_map_base_offset_.setOrigin(T_cam_base.getOrigin());
            initial_offset_set_ = true;
        }

        // Apply the offset so that base_link starts at (0,0,0) in map
        tf2::Transform T_map_base_zeroed = initial_map_base_offset_.inverse() * T_map_base;


        // 5. Broadcast: map → base
        geometry_msgs::msg::TransformStamped sendmsg;
        sendmsg.header.stamp = current_frame_time_;
        sendmsg.header.frame_id = map_frame_;
        sendmsg.child_frame_id = base_frame_;
        tf2::toMsg(T_map_base_zeroed, sendmsg.transform);

        tf_broadcaster_->sendTransform(sendmsg);

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "TF error computing map → base: %s", ex.what());
    }
}

tf2::Transform SlamNode::TransformFromSophus(Sophus::SE3f &pose)
{
    // Convert pose to double precision for tf2 compatibility
    Eigen::Matrix3d rotation = pose.rotationMatrix().cast<double>();
    Eigen::Vector3d translation = pose.translation().cast<double>();

    tf2::Matrix3x3 tf_camera_rotation(
        rotation(0, 0), rotation(0, 1), rotation(0, 2),
        rotation(1, 0), rotation(1, 1), rotation(1, 2),
        rotation(2, 0), rotation(2, 1), rotation(2, 2));
    tf2::Vector3 tf_camera_translation(
        translation(0), translation(1), translation(2));

    // Coordinate transformation matrix: ORB to ROS
    // static const tf2::Matrix3x3 tf_orb_to_ros( // For orbslam3 frame [0 pi/2 0]
    //     0, 0, 1,  // ORB X -> ROS Y
    //     1, 0, 0, // ORB -Y -> ROS Y
    //     0, 1, 0  // ORB Z -> ROS Z
    // ); 
   // Coordinate transformation matrix: ORB to ROS
    static const tf2::Matrix3x3 tf_orb_to_ros( // For orbslam3 frame [0 pi/2 0]
        1, 0, 0,  // ORB X -> ROS Y
        0, 1, 0, // ORB -Y -> ROS Y
        0, 0, 1  // ORB Z -> ROS Z
    ); 

    // Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Return the final tf2::Transform
    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}