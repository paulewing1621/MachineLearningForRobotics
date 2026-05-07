#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

class CollisionAvoidance : public rclcpp::Node {
    protected:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;

        bool only_forward;
        double max_velocity;
        double safety_diameter;
        double ignore_diameter;

        pcl::PointCloud<pcl::PointXYZ> lastpc;

        void velocity_filter(geometry_msgs::msg::Twist::SharedPtr msg) {
            geometry_msgs::msg::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub->publish(filtered);
        }

        void pc_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
#if 0
            pcl::PCLPointCloud2 cloud2;
            // ROS2 Pointcloud2 to PCL Pointcloud2
            pcl_conversions::toPCL(*msg,cloud2);    
            // PCL Pointcloud2 to templated form
            pcl::fromPCLPointCloud2(cloud2,lastpc);
            // RCLCPP_INFO(this->get_logger(),"New point cloud: %d points",int(lastpc.size()));

            // This is an example of using point cloud data
            unsigned int n = lastpc.size();
            for (unsigned int i=0;i<n;i++) {
                float x = lastpc[i].x;
                float y = lastpc[i].y;
                float z = lastpc[i].z;
                // RCLCPP_INFO(this->get_logger(),"%d %.3f %.3f %.3f",i,x,y,z);
            }
#endif
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            lastpc.clear();
#if 1            
            const float fov = 0.3;
            unsigned int n = msg->ranges.size();
            for (unsigned int i=0;i<n;i++) {
				float theta = msg->angle_min + i * msg->angle_increment;
				if (fabs(theta) <= fov) {
					float x = msg->ranges[i]*cos(theta);
					float y = msg->ranges[i]*sin(theta);
					float z = 0;
					lastpc.push_back(pcl::PointXYZ(x,y,z));
				}
                // RCLCPP_INFO(this->get_logger(),"%d %.3f %.3f %.3f",i,x,y,z);
            }
#endif
            // RCLCPP_INFO(this->get_logger(),"New laser scan: %d points",int(lastpc.size()));
        }

        geometry_msgs::msg::Twist findClosestAcceptableVelocity(const geometry_msgs::msg::Twist & desired) {
            geometry_msgs::msg::Twist res = desired;
            if (only_forward && desired.linear.x <= 0){
                    return res;
                }
                
            double limit = max_velocity;
            double safety_radius = safety_diameter / 2.0;
            double ignore_radius = ignore_diameter / 2.0;
            
            unsigned int n = lastpc.size();
            for (unsigned int i=0;i<n;i++) {
                double d = hypot(lastpc[i].x, lastpc[i].y);
                if (d < 3e-2) {
                    continue;
                }
                if (d > ignore_radius) {
                    continue;
                }
                if (d < safety_radius) 
                {
				limit = 0.0;
                break;
				}
                double scale = (d - safety_radius) / (ignore_radius - safety_radius);
                double v = scale * max_velocity;
                limit = std::min(limit, v);

            }
            
            if(res.linear.x>limit){
				res.linear.x = limit;
			}
            if(std::abs(res.linear.x) < 0.05){
				res.linear.x = 0.0;
			}			
            RCLCPP_INFO(this->get_logger(),"Speed limiter: desired %.2f controlled %.2f",desired.linear.x,res.linear.x);
            return res;
        }
    public:
        CollisionAvoidance() : rclcpp::Node("collision_avoidance") {
            this->declare_parameter("only_forward", true);
            this->declare_parameter("max_velocity", 1.0);
            this->declare_parameter("safety_diameter", 0.7);
            this->declare_parameter("ignore_diameter", 1.7);
            only_forward = this->get_parameter("only_forward").as_bool();
            max_velocity = this->get_parameter("max_velocity").as_double();
            safety_diameter = this->get_parameter("safety_diameter").as_double();
            ignore_diameter = this->get_parameter("ignore_diameter").as_double();

            scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>("~/scans",1,
                    std::bind(&CollisionAvoidance::scan_callback,this,std::placeholders::_1));
            cloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("~/clouds",1,
                    std::bind(&CollisionAvoidance::pc_callback,this,std::placeholders::_1));
            velSub = this->create_subscription<geometry_msgs::msg::Twist>("~/vel_input",1,
                    std::bind(&CollisionAvoidance::velocity_filter,this,std::placeholders::_1));
            velPub = this->create_publisher<geometry_msgs::msg::Twist>("~/vel_output",1);

        }

};
int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionAvoidance>());
    rclcpp::shutdown();
}


