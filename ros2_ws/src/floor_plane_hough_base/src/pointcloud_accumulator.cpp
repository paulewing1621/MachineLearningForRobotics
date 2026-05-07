#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class PointCloudAccumulator : public rclcpp::Node {
public:
    PointCloudAccumulator() : Node("pointcloud_accumulator") {
        this->declare_parameter<std::string>("base_frame", "bubbleRob");
        this->declare_parameter<double>("max_range", 5.0);

        base_frame_ = this->get_parameter("base_frame").as_string();
        max_range_ = this->get_parameter("max_range").as_double();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(10).best_effort();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "scan", qos, std::bind(&PointCloudAccumulator::pointCloudCallback, this, std::placeholders::_1));

        accumulated_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("accumulated_cloud", 10);

        // Initialize accumulated cloud
        accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> pc_sensor;

        // Transform to base frame if needed
        sensor_msgs::msg::PointCloud2 pc_transformed;
        if (msg->header.frame_id != base_frame_) {
            try {
                auto transformStamped = tf_buffer_->lookupTransform(
                    base_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
                tf2::doTransform(*msg, pc_transformed, transformStamped);
                pcl_conversions::toPCL(pc_transformed, pc_sensor);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 Transform failed: %s", ex.what());
                return;
            }
        } else {
            pcl_conversions::toPCL(*msg, pc_sensor);
        }

        // Filter points by max range and remove NaNs
        for (const auto &pt : pc_sensor.points) {
            double dist = std::hypot(pt.x, pt.y);
            if (dist < 1e-3 || dist > max_range_) continue;
            accumulated_cloud_->points.push_back(pt);
        }

        accumulated_cloud_->width = accumulated_cloud_->points.size();
        accumulated_cloud_->height = 1;
        accumulated_cloud_->is_dense = false;

        // Publish accumulated cloud
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl_conversions::fromPCL(*accumulated_cloud_, out_msg);
        out_msg.header.stamp = this->now();
        out_msg.header.frame_id = base_frame_;
        accumulated_pub_->publish(out_msg);
    }

    std::string base_frame_;
    double max_range_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_pub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudAccumulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
