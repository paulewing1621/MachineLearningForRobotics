#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MetalDetectionNode : public rclcpp::Node
{
protected:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr detector_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string world_frame_;
    std::string sensor_frame_;
    double threshold_;             
    double min_weight_to_publish_; 
    double cluster_timeout_;       
    double zone_radius_;           


    double sum_w_;
    double sum_x_;
    double sum_y_;
    double sum_z_;

    
    bool in_detection_;
    rclcpp::Time last_strong_time_;  

    
    struct Treasure
    {
        int id;
        double sum_w; 
        double sum_x; 
        double sum_y;  
        double sum_z; 

        double x() const { return sum_x / sum_w; }
        double y() const { return sum_y / sum_w; }
        double z() const { return sum_z / sum_w; }
    };

    std::vector<Treasure> treasures_;
    int next_marker_id_;

    void resetAccumulators()
    {
        sum_w_ = 0.0;
        sum_x_ = 0.0;
        sum_y_ = 0.0;
        sum_z_ = 0.0;
    }

    void publishTreasureMarker(const Treasure &T, const std::string &prefix)
    {
        const double cx = T.x();
        const double cy = T.y();
        const double cz = T.z();

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = world_frame_;

        marker.ns = "metal_detection";
        marker.id = T.id;                    
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = cx;
        marker.pose.position.y = cy;
        marker.pose.position.z = cz;
        marker.pose.orientation.w = 1.0;      

        // Marker plus gros
        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 0.8;

        marker.color.a = 0.9;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.lifetime = rclcpp::Duration(0, 0); 

        marker_pub_->publish(marker);

        RCLCPP_INFO(this->get_logger(),
                    "%s treasure id=%d at (%.2f, %.2f, %.2f), total_weight=%.2f",
                    prefix.c_str(), T.id, cx, cy, cz, T.sum_w);
    }

    void finalizeCluster()
    {
        if (!in_detection_) {
            return;
        }

        in_detection_ = false;

        if (sum_w_ < min_weight_to_publish_) {
            RCLCPP_INFO(this->get_logger(),
                        "Detection cluster ignored: weight %.2f < min_weight_to_publish %.2f",
                        sum_w_, min_weight_to_publish_);
            resetAccumulators();
            return;
        }

       
        const double cx = sum_x_ / sum_w_;
        const double cy = sum_y_ / sum_w_;
        const double cz = sum_z_ / sum_w_;

        const double zone_r2 = zone_radius_ * zone_radius_;

        int best_idx = -1;
        double best_dist2 = zone_r2;

        for (std::size_t i = 0; i < treasures_.size(); ++i) {
            const double dx = cx - treasures_[i].x();
            const double dy = cy - treasures_[i].y();
            const double dist2 = dx * dx + dy * dy;
            if (dist2 <= best_dist2) {
                best_dist2 = dist2;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx >= 0) {
            Treasure &T = treasures_[best_idx];
            T.sum_w += sum_w_;
            T.sum_x += sum_w_ * cx;
            T.sum_y += sum_w_ * cy;
            T.sum_z += sum_w_ * cz;
            publishTreasureMarker(T, "Updated");
        } else {
            Treasure T;
            T.id    = next_marker_id_++;
            T.sum_w = sum_w_;
            T.sum_x = sum_w_ * cx;
            T.sum_y = sum_w_ * cy;
            T.sum_z = sum_w_ * cz;
            treasures_.push_back(T);
            publishTreasureMarker(treasures_.back(), "New");
        }

        resetAccumulators();
    }

protected:
    void detectorCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const double s = static_cast<double>(msg->data);
        const rclcpp::Time now = this->get_clock()->now();

        if (s >= threshold_) {
            if (!in_detection_) {
                RCLCPP_INFO(this->get_logger(),
                            "Starting new detection cluster (s=%.3f)", s);
                resetAccumulators();
                in_detection_ = true;
            }

            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                transformStamped = tf_buffer_->lookupTransform(
                    world_frame_,   
                    sensor_frame_,  
                    tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(),
                            "Could not transform %s to %s: %s",
                            sensor_frame_.c_str(),
                            world_frame_.c_str(),
                            ex.what());
                return;
            }

            const double px = transformStamped.transform.translation.x;
            const double py = transformStamped.transform.translation.y;
            const double pz = transformStamped.transform.translation.z;

            const double w = s;

            sum_w_  += w;
            sum_x_  += w * px;
            sum_y_  += w * py;
            sum_z_  += w * pz;

            last_strong_time_ = now;
            return;
        }

        if (in_detection_) {
            const double dt = (now - last_strong_time_).seconds();
            if (dt > cluster_timeout_) {
                RCLCPP_INFO(this->get_logger(),
                            "Ending detection cluster after %.2fs below threshold", dt);
                finalizeCluster();
            } else {
            }
        }
    }

public:
    MetalDetectionNode() :
        rclcpp::Node("metal_detection_node"),
        sum_w_(0.0),
        sum_x_(0.0),
        sum_y_(0.0),
        sum_z_(0.0),
        in_detection_(false),
        last_strong_time_(this->get_clock()->now()),
        next_marker_id_(0)
    {
        this->declare_parameter<std::string>("world_frame", "world");
        this->declare_parameter<std::string>("sensor_frame", "VSV/Kision_sensor");
        this->declare_parameter<double>("threshold", 0.7);
        this->declare_parameter<double>("min_weight_to_publish", 5.0);
        this->declare_parameter<double>("cluster_timeout", 0.5); 
        this->declare_parameter<double>("zone_radius", 5.0);    

        world_frame_           = this->get_parameter("world_frame").as_string();
        sensor_frame_          = this->get_parameter("sensor_frame").as_string();
        threshold_             = this->get_parameter("threshold").as_double();
        min_weight_to_publish_ = this->get_parameter("min_weight_to_publish").as_double();
        cluster_timeout_       = this->get_parameter("cluster_timeout").as_double();
        zone_radius_           = this->get_parameter("zone_radius").as_double();

        RCLCPP_INFO(this->get_logger(),
                    "MetalDetectionNode: world_frame=%s, sensor_frame=%s, "
                    "threshold=%.2f, min_weight_to_publish=%.2f, "
                    "cluster_timeout=%.2f, zone_radius=%.2f",
                    world_frame_.c_str(),
                    sensor_frame_.c_str(),
                    threshold_,
                    min_weight_to_publish_,
                    cluster_timeout_,
                    zone_radius_);

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        detector_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vrep/metalDetector",
            qos,
            std::bind(&MetalDetectionNode::detectorCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "~/detected_object", 1);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetalDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
