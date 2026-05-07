#include <sys/stat.h>
#include <sys/types.h>

#include <cstdio>
#include <cmath>
#include <string>
#include <cassert>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.hpp>

class ShoreFollowerObserve2 : public rclcpp::Node
{
protected:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    image_transport::Subscriber image_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    std::string base_frame_;
    std::string analysis_frame_;

    std::string outdir_;
    std::string image_transport_type_;
    double rotation_threshold_;    
    double min_displacement_xy_;  
    double min_z_displacement_;    
    int max_image_per_type_;
    unsigned long image_counter_;
    unsigned long type_counter_[3]; 
    int joystick_button_;
    bool learning_;

    double prediction_up_threshold_;   
    double prediction_down_threshold_;   
    double prediction_level_threshold_;  

    geometry_msgs::msg::Twist last_command_;
    rclcpp::Time last_joy_time_;
    rclcpp::Time last_command_time_;

    struct Pose3 {
        double x;
        double y;
        double z;
    };

    Pose3 last_pose_;       
    bool last_pose_received_;

protected:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if ((joystick_button_ < int(msg->buttons.size())) && msg->buttons[joystick_button_]) {
            rclcpp::Time now = this->get_clock()->now();
            if ((now - last_joy_time_).seconds() < 0.5) {
                return;
            }
            last_joy_time_ = now;
            learning_ = !learning_;
            if (learning_) {
                RCLCPP_INFO(this->get_logger(), "Learning started, recording images and labels");
            } else {
                RCLCPP_INFO(this->get_logger(), "Learning stopped");
            }
        }
    }

    void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_command_time_ = this->get_clock()->now();
        last_command_ = *msg;
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
    {
        if (!learning_) {
            return;
        }

        if ((rclcpp::Time(img_msg->header.stamp) - last_command_time_).seconds() > 0.1) {
            return;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        std::string errStr;
        if (!tf_buffer_->canTransform(
                analysis_frame_, base_frame_, img_msg->header.stamp,
                rclcpp::Duration(std::chrono::duration<double>(1.0)), &errStr))
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot transform current pose to analysis frame: %s", errStr.c_str());
            return;
        }

        try {
            transformStamped = tf_buffer_->lookupTransform(
                analysis_frame_, base_frame_, img_msg->header.stamp);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
            return;
        }

        Pose3 new_pose;
        new_pose.x = transformStamped.transform.translation.x;
        new_pose.y = transformStamped.transform.translation.y;
        new_pose.z = transformStamped.transform.translation.z;

        if (last_pose_received_) {
            double dx = new_pose.x - last_pose_.x;
            double dy = new_pose.y - last_pose_.y;
            double dist_xy = std::hypot(dx, dy);
            double delta_z = new_pose.z - last_pose_.z;

            if ((dist_xy < min_displacement_xy_) && (std::abs(delta_z) < min_z_displacement_)) {
                return;
            }

            int label = 1; 

            if (delta_z > prediction_up_threshold_) {
                label = 0; 
            } else if (delta_z < prediction_down_threshold_) {
                label = 2; 
            } else {
                if (std::abs(delta_z) < prediction_level_threshold_) {
                    label = 1; 
                } else {
                    label = 1;
                }
            }

            bool save_it = (type_counter_[label] < static_cast<unsigned>(max_image_per_type_));

            if (save_it) {
                cv::Mat img(cv_bridge::toCvShare(img_msg, "bgr8")->image);

                char dirname[1024], filename[1024], labelname[1024];
                sprintf(dirname, "%s/%04ld", outdir_.c_str(), image_counter_ / 1000);
                mkdir(dirname, 0700); 

                sprintf(filename, "%s/%04ld/%04ld.png",
                        outdir_.c_str(), image_counter_ / 1000, image_counter_ % 1000);
                cv::imwrite(filename, img);

                sprintf(labelname, "%s/labels.txt", outdir_.c_str());
                FILE * fp = fopen(labelname, "a");
                if (fp) {
                    fprintf(fp, "%04ld/%04ld.png %d\n", image_counter_ / 1000, image_counter_ % 1000, label);
                    fclose(fp);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Cannot open label file %s for appending", labelname);
                }

                type_counter_[label]++;
                image_counter_++;

                const char *label_str = (label == 0) ? "UP" : (label == 1) ? "LEVEL" : "DOWN";
                RCLCPP_INFO(this->get_logger(),
                            "Saved image %s (label=%s), Δz=%.4f, xy_dist=%.4f",
                            filename, label_str, (new_pose.z - last_pose_.z), dist_xy);
            } else {
                const char *label_str = (label == 0) ? "UP" : (label == 1) ? "LEVEL" : "DOWN";
                RCLCPP_INFO(this->get_logger(),
                            "Skipping save for label=%s (count=%lu >= max=%d)",
                            label_str, type_counter_[label], max_image_per_type_);
            }

        } else {
            RCLCPP_DEBUG(this->get_logger(), "Initial pose received in analysis frame");
            last_pose_received_ = true;
        }
        last_pose_ = new_pose;
    }

public:
    ShoreFollowerObserve2()
    : rclcpp::Node("shore_follower_observe2"),
      image_counter_(0),
      joystick_button_(3),
      learning_(true),
      last_pose_received_(false)
    {
        this->declare_parameter<std::string>("~/image_transport", "raw");
        this->declare_parameter<std::string>("~/base_frame", "body");
        this->declare_parameter<std::string>("~/analysis_frame", "map");
        this->declare_parameter<std::string>("~/out_dir", ".");
        this->declare_parameter<double>("~/rotation_threshold", 0.4);
        this->declare_parameter<double>("~/min_displacement_xy", 0.01); 
        this->declare_parameter<double>("~/min_z_displacement", 0.005); 
        this->declare_parameter<int>("~/max_image_per_type", 1000);
        this->declare_parameter<int>("~/joystick_button", 3);

        this->declare_parameter<double>("~/prediction_up_threshold", 0.02);  
        this->declare_parameter<double>("~/prediction_down_threshold", -0.02); 
        this->declare_parameter<double>("~/prediction_level_threshold", 0.01); 


        image_transport_type_ = this->get_parameter("~/image_transport").as_string();
        base_frame_ = this->get_parameter("~/base_frame").as_string();
        analysis_frame_ = this->get_parameter("~/analysis_frame").as_string();
        outdir_ = this->get_parameter("~/out_dir").as_string();
        rotation_threshold_ = this->get_parameter("~/rotation_threshold").as_double();
        min_displacement_xy_ = this->get_parameter("~/min_displacement_xy").as_double();
        min_z_displacement_ = this->get_parameter("~/min_z_displacement").as_double();
        max_image_per_type_ = this->get_parameter("~/max_image_per_type").as_int();
        joystick_button_ = this->get_parameter("~/joystick_button").as_int();

        prediction_up_threshold_ = this->get_parameter("~/prediction_up_threshold").as_double();
        prediction_down_threshold_ = this->get_parameter("~/prediction_down_threshold").as_double();
        prediction_level_threshold_ = this->get_parameter("~/prediction_level_threshold").as_double();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        char labelname[1024];
        sprintf(labelname, "%s/labels.txt", outdir_.c_str());
        FILE * fp = fopen(labelname, "w");
        if (!fp) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open label file %s", labelname);
        } else {
            fclose(fp);
        }

        type_counter_[0] = type_counter_[1] = type_counter_[2] = 0;

        last_command_time_ = last_joy_time_ = this->get_clock()->now();

        std::string errStr;
        geometry_msgs::msg::TransformStamped transformStamped;
        if (!tf_buffer_->canTransform(analysis_frame_, base_frame_, rclcpp::Time(0),
                rclcpp::Duration(std::chrono::duration<double>(1.0)), &errStr))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Initial transform unavailable from '%s' to '%s': %s. Node will still run and wait for transforms.",
                        base_frame_.c_str(), analysis_frame_.c_str(), errStr.c_str());
        } else {
            try {
                transformStamped = tf_buffer_->lookupTransform(analysis_frame_, base_frame_, rclcpp::Time(0));
                last_pose_.x = transformStamped.transform.translation.x;
                last_pose_.y = transformStamped.transform.translation.y;
                last_pose_.z = transformStamped.transform.translation.z;
                last_pose_received_ = true;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Initial transform lookup failed: %s", ex.what());
            }
        }

        image_sub_ = image_transport::create_subscription(
            this, "~/image",
            std::bind(&ShoreFollowerObserve2::image_callback, this, std::placeholders::_1),
            image_transport_type_, rmw_qos_profile_sensor_data);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "~/joy", 1, std::bind(&ShoreFollowerObserve2::joy_callback, this, std::placeholders::_1));

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/arm_ik/twist", 1, std::bind(&ShoreFollowerObserve2::twist_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "ShoreFollowerObserve2 started: base_frame=%s, analysis_frame=%s, outdir=%s",
                    base_frame_.c_str(), analysis_frame_.c_str(), outdir_.c_str());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShoreFollowerObserve2>());
    rclcpp::shutdown();
    return 0;
}
