#include "stereo_image_saver.hpp"

namespace slam
{
    ImageSaver::ImageSaver(const rclcpp::NodeOptions & options)
        : Node("image_saver", options), save_images(false), counter_(0) {
        // Declare and get the path parameter from the launch file
        this->declare_parameter("saving_path", "~/Pictures/images");
        this->declare_parameter<bool>("apply_clahe", true);
        this->declare_parameter("clahe_tiles", 5);
        this->declare_parameter("clahe_climp", 2.0);

        this->get_parameter("saving_path", path_);
        this->get_parameter("apply_clahe", apply_clahe_);

        // Verify or create the path directories
        if (!verify_path(path_, 0)) {
            RCLCPP_ERROR(this->get_logger(), "Error creating directory");
            rclcpp::shutdown();
        }

        if(apply_clahe_){
            double tile = this->get_parameter("clahe_tile").as_double();
            double climp = this->get_parameter("clahe_climp").as_double();
            clahe_->setClipLimit(climp);
            clahe_->setTilesGridSize(cv::Size(int(tile), int(tile)));
        }


        auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group;
            
        // Initialize the subscribers using the node's interface directly
        left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera_1/image_raw", rmw_qos_profile_sensor_data, sub_options);
        right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera_2/image_raw", rmw_qos_profile_sensor_data, sub_options);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>( SyncPolicy(10), *left_sub, *right_sub);
        sync_->registerCallback(&ImageSaver::images_cb, this);
        
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS(), std::bind(&ImageSaver::odom_cb, this, std::placeholders::_1));
        // Initialize service
        save_image_srv = this->create_service<std_srvs::srv::Trigger>("save_images", std::bind(&ImageSaver::save_image_cb, this, std::placeholders::_1, std::placeholders::_2));

    }

    ImageSaver::~ImageSaver() {
        if (odom_file_.is_open()) {
            odom_file_.close();
            RCLCPP_INFO(this->get_logger(), "Odometry data saved successfully.");
        }
    }

    cv::Mat ImageSaver::appyCLAHEtoColor(const cv::Mat& input_bgr)
    {
        cv::Mat lab_image;
        cv::cvtColor(input_bgr, lab_image, cv::COLOR_BGR2Lab);

        // 2. Separar a imagem nos 3 canais (L, A, B)
        std::vector<cv::Mat> lab_channels(3);
        cv::split(lab_image, lab_channels);

        // Aplicar o CLAHE exclusivamente no canal L (lab_channels[0])
        clahe_->apply(lab_channels[0], lab_channels[0]);

        // 4. Juntar os canais modificados de volta em uma única imagem LAB
        cv::Mat processed_lab;
        cv::merge(lab_channels, processed_lab);

        // 5. Converter de volta para o padrão BGR
        cv::Mat output_bgr;
        cv::cvtColor(processed_lab, output_bgr, cv::COLOR_Lab2BGR);

        return output_bgr;
    }

    void ImageSaver::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
        this->get_parameter("apply_clahe", apply_clahe_);
        if(apply_clahe_){
            double tile = this->get_parameter("clahe_tile").as_double();
            double climp = this->get_parameter("clahe_climp").as_double();
            clahe_->setClipLimit(climp);
            clahe_->setTilesGridSize(cv::Size(tile, tile));
        }
        lastOdom_ = msg;
    }

    bool ImageSaver::verify_path(const std::string &path_string, int counter = 0) {
        // Check if the path exists
        if (!std::filesystem::exists(path_string)) {
            // Path does not exist, create the directory
            if (std::filesystem::create_directory(path_string)) {
                RCLCPP_INFO(this->get_logger(), "Directory created successfully.");
                if (std::filesystem::create_directory(path_string + "/left") &&
                    std::filesystem::create_directory(path_string + "/right")) {
                    RCLCPP_INFO(this->get_logger(), "Left and Right folders created successfully.");
                    std::string odom_file_path = path_string + "/odom_data.txt";
                    odom_file_.open(odom_file_path, std::ios::out);
                if (!odom_file_.is_open()) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot Open file");
                } else {
                    // Escreve um cabeçalho opcional no arquivo para organizar os dados
                    odom_file_ << "Timestamp, x, y, z, w, x, y, z\n";
                }
                    path_ = path_string;
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Error creating Left and Right directories.");
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Error creating main directory.");
                return false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Directory '%s' already exists.", path_string.c_str());
            counter++;
            RCLCPP_INFO(this->get_logger(), "Increase counter to %i", counter);

            // Create a new path by appending the counter
            std::string new_path = path_string + "_" + std::to_string(counter);

            return verify_path(new_path, counter);
        }
    }

    void ImageSaver::images_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                            const sensor_msgs::msg::Image::ConstSharedPtr &msgRight) {
        msgRight_ = msgRight;
        msgLeft_ = msgLeft;
    }

    bool ImageSaver::save_image_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> &req,
                                const std::shared_ptr<std_srvs::srv::Trigger::Response> &res) {

        cv::Mat img_left, img_right;
        try {
            img_left = cv_bridge::toCvCopy(msgLeft_, msgLeft_->encoding)->image;
            img_right = cv_bridge::toCvCopy(msgRight_, msgRight_->encoding)->image;
            cv::cvtColor(img_left, img_left, cv::COLOR_BayerBG2BGR);
            cv::cvtColor(img_right, img_right, cv::COLOR_BayerBG2BGR);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return false;
        }
        //


        // Save images
        std::string img_counter_;
        if (counter_ < 10) {
            img_counter_ = "00";
        } else if (counter_ >= 10 && counter_ < 100) {
            img_counter_ = "0";
        }else if(counter_ >= 100 && counter_ < 1000) {
            img_counter_ = "";
        }
        if(odom_file_.is_open()){
            odom_file_ << lastOdom_->header.stamp.sec << "." << lastOdom_->header.stamp.nanosec << ","
                    << lastOdom_->pose.pose.position.x << ","
                    << lastOdom_->pose.pose.position.y << ","
                    << lastOdom_->pose.pose.position.z << ","
                    << lastOdom_->pose.pose.orientation.w << ","
                    << lastOdom_->pose.pose.orientation.x << ","
                    << lastOdom_->pose.pose.orientation.y << ","
                    << lastOdom_->pose.pose.orientation.z << "\n";

            odom_file_.flush();   
        } else {
            RCLCPP_ERROR(this->get_logger(), "File is not open");
        }

        std::string path_L = path_ + "/left/L" + img_counter_ + std::to_string(counter_) + ".png";
        std::string path_R = path_ + "/right/R" + img_counter_ + std::to_string(counter_) + ".png";
        if(apply_clahe_){
            cv::Mat clahe_left, clahe_right;
            clahe_left = appyCLAHEtoColor(img_left);
            clahe_right = appyCLAHEtoColor(img_right);
            cv::imwrite(path_L, clahe_left);
            cv::imwrite(path_R, clahe_right);
        }
        else{
            cv::imwrite(path_L, img_left);
            cv::imwrite(path_R, img_right);
        }
        RCLCPP_INFO(this->get_logger(), "Saved images %s and %s", path_L.c_str(), path_R.c_str());
        counter_++;
        return true;
    }
} 
RCLCPP_COMPONENTS_REGISTER_NODE(slam::ImageSaver)