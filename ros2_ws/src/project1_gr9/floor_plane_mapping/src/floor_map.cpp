#include <cstdio>
#include <cmath>
#include <map>
#include <list>
#include <vector>
#include <random>
#include <string>
#include <cstring>   
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

// Labels for OpenCV image
static constexpr uint8_t LABEL_TRAV = 0;    // traversable
static constexpr uint8_t LABEL_OCC  = 100;  // non-traversable
static constexpr uint8_t LABEL_UNK  = 255;  // unknown

// Build a sensor_msgs::Image (BGR8)
static sensor_msgs::msg::Image makeBgr8Image(const cv::Mat& bgr,
                                             const std::string& frame_id,
                                             const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.frame_id = frame_id;
  img.header.stamp    = stamp;
  img.height   = static_cast<uint32_t>(bgr.rows);
  img.width    = static_cast<uint32_t>(bgr.cols);
  img.encoding = "bgr8";
  img.is_bigendian = false;
  img.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.cols * 3);
  img.data.resize(static_cast<size_t>(bgr.rows) * bgr.cols * 3);
  std::memcpy(img.data.data(), bgr.data, img.data.size());
  return img;
}

class FloorMapNode : public rclcpp::Node {
public:
  using PointList = std::list<pcl::PointXYZ>;
  using Cell      = cv::Point; // (ix, iy)
  using BucketMap = std::map<Cell, PointList, bool(*)(const Cell&, const Cell&)>;

  FloorMapNode()
  : rclcpp::Node("floor_map"),
    rd_(),
    gen_(rd_()),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    buckets_(cellCmp)
  {
    // Parameters
    this->declare_parameter<std::string>("~/base_frame",   "bubbleRob");
    this->declare_parameter<std::string>("~/world_frame",  "world");
    this->declare_parameter<double>("~/max_range", 4.0);
    this->declare_parameter<int>("~/n_samples", 400);
    this->declare_parameter<double>("~/tolerance", 0.05);  
    this->declare_parameter<double>("~/tilt_deg_max", 20.0);
    this->declare_parameter<double>("~/xmin", -5.0);
    this->declare_parameter<double>("~/xmax", 5.0);
    this->declare_parameter<double>("~/ymin", -5.0);
    this->declare_parameter<double>("~/ymax", 5.0);
    this->declare_parameter<double>("~/res", 0.10);
    this->declare_parameter<double>("~/z_floor_max", 1e9);  
    this->declare_parameter<int>("~/min_points_trav",5);
    this->declare_parameter<bool>("~/flip_vertical", false);
    // Bayesian / log-odds
    this->declare_parameter<double>("~/prior_trav", 0.5);   
    this->declare_parameter<double>("~/p_hit_trav", 0.75);  // P(T|z=TRAV)
    this->declare_parameter<double>("~/p_hit_occ",0.25);  // P(T|z=OCC)
    this->declare_parameter<double>("~/range_sigma",2.0);   // distance weight
    this->declare_parameter<double>("~/min_weight", 0.2);   // min weight
    this->declare_parameter<double>("~/L_clip",6.0);   // log-odds clip
    // Traversability prob
    this->declare_parameter<double>("~/thr_trav",0.6);
    this->declare_parameter<double>("~/thr_occ",0.4);

    base_frame_ = this->get_parameter("~/base_frame").as_string();
    world_frame_ = this->get_parameter("~/world_frame").as_string();
    max_range_ = this->get_parameter("~/max_range").as_double();
    ransac_n_samples_ = this->get_parameter("~/n_samples").as_int();
    ransac_tol_ = this->get_parameter("~/tolerance").as_double();
    tilt_deg_max_ = this->get_parameter("~/tilt_deg_max").as_double();

    xmin_ = this->get_parameter("~/xmin").as_double();
    xmax_ = this->get_parameter("~/xmax").as_double();
    ymin_ = this->get_parameter("~/ymin").as_double();
    ymax_ = this->get_parameter("~/ymax").as_double();
    res_  = this->get_parameter("~/res").as_double();
    z_floor_max_ = this->get_parameter("~/z_floor_max").as_double();
    min_points_trav_= this->get_parameter("~/min_points_trav").as_int();
    flip_vertical_= this->get_parameter("~/flip_vertical").as_bool();

    prior_trav_= this->get_parameter("~/prior_trav").as_double();
    p_hit_trav_= this->get_parameter("~/p_hit_trav").as_double();
    p_hit_occ_ = this->get_parameter("~/p_hit_occ").as_double();
    range_sigma_ = this->get_parameter("~/range_sigma").as_double();
    min_weight_= this->get_parameter("~/min_weight").as_double();
    L_clip_ = this->get_parameter("~/L_clip").as_double();
    thr_trav_ = this->get_parameter("~/thr_trav").as_double();
    thr_occ_ = this->get_parameter("~/thr_occ").as_double();

    // Grid size
    width_  = static_cast<int>(std::ceil((xmax_ - xmin_) / res_));
    height_ = static_cast<int>(std::ceil((ymax_ - ymin_) / res_));
    if (width_ <= 0 || height_ <= 0) {
      throw std::runtime_error("invalid grid");
    }

    // OccupancyGrid 
    info_.resolution = res_;
    info_.width  = static_cast<uint32_t>(width_);
    info_.height = static_cast<uint32_t>(height_);
    info_.origin.position.x = xmin_;
    info_.origin.position.y = ymin_;
    info_.origin.position.z = 0.0;
    info_.origin.orientation.w = 1.0;

    og_header_.frame_id = world_frame_;

    // Bayesian state
    L0_ = std::log(prior_trav_ / std::max(1e-6, 1.0 - prior_trav_));
    lz_trav_ = std::log(p_hit_trav_ / std::max(1e-6, 1.0 - p_hit_trav_));
    lz_occ_  = std::log(p_hit_occ_  / std::max(1e-6, 1.0 - p_hit_occ_));
    L_map_   = cv::Mat(height_, width_, CV_32FC1, cv::Scalar(L0_));
    obs_count_ = cv::Mat(height_, width_, CV_16UC1, cv::Scalar(0));

    // I/O
    auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/scans", qos, std::bind(&FloorMapNode::cloudCb, this, std::placeholders::_1));
    pub_og_  = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/traversability_map", 1);
    pub_img_ = this->create_publisher<sensor_msgs::msg::Image>("~/traversability_image", 1);
  }

private:
  // Convertion from prob map to OccupancyGrid 
  void mat_to_og_from_prob(const cv::Mat& prob_trav, nav_msgs::msg::OccupancyGrid& og) {
    og.info   = info_;
    og.header = og_header_;
    og.header.stamp = this->now();
    og.data.resize(prob_trav.cols * prob_trav.rows, -1);

    for (int r = 0; r < prob_trav.rows; ++r) {
      for (int c = 0; c < prob_trav.cols; ++c) {
        float p = prob_trav.at<float>(r,c);
        int8_t out = -1;
        if (obs_count_.at<uint16_t>(r,c) == 0) {
          out = -1; // unknown
        } else if (p >= static_cast<float>(thr_trav_)) {
          out = 0;  // traversable
        } else if (p <= static_cast<float>(thr_occ_)) {
          out = 100; // non-traversable
        } else {
          out = -1; // uncertain
        }
        og.data[r * prob_trav.cols + c] = out;
      }
    }
  }

  // Bucket indexing
  static bool cellCmp(const Cell& a, const Cell& b) {
    return (a.y < b.y) || (a.y == b.y && a.x < b.x);
  }
  inline bool worldToCell(double x, double y, Cell& out) const {
    int ix = static_cast<int>(std::floor((x - xmin_) / res_));
    int iy = static_cast<int>(std::floor((y - ymin_) / res_));
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) return false;
    out.x = ix; out.y = iy;
    return true;
  }
  inline void cellToWorldCenter(int ix, int iy, double& x, double& y) const {
    x = xmin_ + (ix + 0.5) * res_;
    y = ymin_ + (iy + 0.5) * res_;
  }

  // RANSAC plane fit per cell
  bool fitPlaneRansacCell(const PointList& L,
                          int n_samples,
                          double inlier_tol_m,
                          Eigen::Vector3f& best_normal)
  {
    const size_t n = L.size();
    if (n < 3) return false;

    std::vector<Eigen::Vector3f> P; P.reserve(n);
    for (const auto& p : L) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
        P.emplace_back(p.x, p.y, p.z);
    }
    if (P.size() < 3) return false;

    std::uniform_int_distribution<> dsample(0, static_cast<int>(P.size()) - 1);

    size_t best = 0;
    best_normal = Eigen::Vector3f(0,0,1);

    for (int it = 0; it < n_samples; ++it) {
      int j1 = dsample(gen_);
      int j2 = dsample(gen_);
      int j3 = dsample(gen_);
      while ((j2 == j1) || (j3 == j1) || (j3 == j2)) {
        j2 = dsample(gen_);
        j3 = dsample(gen_);
      }

      const Eigen::Vector3f& P1 = P[(size_t)j1];
      const Eigen::Vector3f& P2 = P[(size_t)j2];
      const Eigen::Vector3f& P3 = P[(size_t)j3];

      Eigen::Vector3f v1 = P2 - P1;
      Eigen::Vector3f v2 = P3 - P1;
      Eigen::Vector3f nrm = v1.cross(v2);
      float nrm_norm = nrm.norm();
      if (nrm_norm < 1e-9f) continue;
      nrm /= nrm_norm;

      // skip near vertical planes
      if (std::fabs(nrm(2)) < 1e-6f) continue;

      const double a = nrm(0);
      const double b = nrm(1);
      const double c = nrm(2);
      const double d = -(a * P1(0) + b * P1(1) + c * P1(2));

      size_t ninliers = 0;
      for (const auto& q : P) {
        const double dz = std::fabs(a*q(0) + b*q(1) + c*q(2) + d);
        if (dz < inlier_tol_m) ++ninliers;
      }

      if (ninliers > best) {
        if (std::fabs(nrm(2)) < 1e-6f) continue;
        best = ninliers;
        best_normal = nrm;
      }
    }
    return (best > 0);
  }

  // Classify a cell with the RANSAC normal tilt
  bool classifyCellRansac(const PointList& L,
                          int n_samples,
                          double inlier_tol_m,
                          double tilt_deg_max,
                          int min_pts,
                          uint8_t& label)
  {
    if ((int)L.size() < std::max(3, min_pts)) { label = LABEL_UNK; return true; }

    Eigen::Vector3f normal;
    if (!fitPlaneRansacCell(L, n_samples, inlier_tol_m, normal)) {
      label = LABEL_UNK;
      return true; 
    }

    const float c = std::clamp(std::fabs(normal.dot(Eigen::Vector3f(0,0,1))), 0.f, 1.f);
    const float theta_deg = std::acos(c) * 180.0f / float(M_PI);
    label = (theta_deg <= tilt_deg_max) ? LABEL_TRAV : LABEL_OCC;
    return true;
  }


  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf_robot;
    try {
      tf_robot = tf_buffer_.lookupTransform(
        world_frame_, base_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
    } catch (...) {
      return;
    }
    const double rx = tf_robot.transform.translation.x;
    const double ry = tf_robot.transform.translation.y;

    // Cloud to world_frame
    sensor_msgs::msg::PointCloud2 cloud_world;
    try {
      auto tf = tf_buffer_.lookupTransform(
        world_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
      tf2::doTransform(*msg, cloud_world, tf);
    } catch (...) {
      return;
    }

    // ROS to PCL
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(cloud_world, pc);

    // Buckets and filters 
    buckets_.clear();
    const bool use_range = (max_range_ > 0.0);
    const double r2max = max_range_ * max_range_;
    for (const auto& P : pc.points) {
      if (!std::isfinite(P.x) || !std::isfinite(P.y) || !std::isfinite(P.z)) continue;

      if (use_range) {
        const double dx = P.x - rx, dy = P.y - ry;
        if (dx*dx + dy*dy > r2max) continue;
      }
      if (P.z > z_floor_max_) continue; 

      Cell c;
      if (worldToCell(P.x, P.y, c)) {
        buckets_[c].push_back(P);
      }
    }

    // Local measurements and Bayesian log-odds update
    for (const auto& kv : buckets_) {
      const Cell& coord = kv.first;
      const PointList& L = kv.second;

      uint8_t mlabel = LABEL_UNK;
      classifyCellRansac(L, ransac_n_samples_, ransac_tol_, tilt_deg_max_, min_points_trav_, mlabel);
      if (mlabel == LABEL_UNK) continue; 

      // distance weight
      double cx, cy; cellToWorldCenter(coord.x, coord.y, cx, cy);
      const double dx = cx - rx, dy = cy - ry;
      const double d  = std::sqrt(dx*dx + dy*dy);
      double w = std::exp( - (d*d) / (2.0 * range_sigma_ * range_sigma_) );
      w = std::max(min_weight_, std::min(1.0, w));

      // log-odds update: L += w * (logit(P(T|z)) - logit(p0))
      float& Lc = L_map_.at<float>(coord.y, coord.x);
      const double lz = (mlabel == LABEL_TRAV) ? lz_trav_ : lz_occ_;
      Lc = static_cast<float>( std::clamp( double(Lc) + w * (lz - L0_), -L_clip_, L_clip_) );

      // observation counter
      uint16_t& cnt = obs_count_.at<uint16_t>(coord.y, coord.x);
      if (cnt < std::numeric_limits<uint16_t>::max()) ++cnt;
    }

    // Convertion from log-odds to traversability probability
    cv::Mat prob(height_, width_, CV_32FC1);
    for (int r = 0; r < height_; ++r) {
      for (int c = 0; c < width_; ++c) {
        float Lc = L_map_.at<float>(r,c);
        float p  = 1.0f / (1.0f + std::exp(-Lc));
        prob.at<float>(r,c) = p;
      }
    }

    // Publish OccupancyGrid 
    nav_msgs::msg::OccupancyGrid og_out;
    mat_to_og_from_prob(prob, og_out);
    pub_og_->publish(og_out);

    // Color image: blue=TRAV, red=OCC, gray=UNK
    cv::Mat color(height_, width_, CV_8UC3);
    for (int r = 0; r < height_; ++r) {
      for (int c = 0; c < width_; ++c) {
        uint16_t cnt = obs_count_.at<uint16_t>(r,c);
        cv::Vec3b pix(128,128,128); 
        if (cnt > 0) {
          float p = prob.at<float>(r,c);
          if (p >= static_cast<float>(thr_trav_))      pix = cv::Vec3b(255, 0,   0); // blue = TRAV
          else if (p <= static_cast<float>(thr_occ_))  pix = cv::Vec3b(  0, 0, 255); // red  = OCC
        }
        color.at<cv::Vec3b>(r,c) = pix;
      }
    }
    cv::Mat vis;
    if (flip_vertical_) cv::flip(color, vis, 0); else vis = color;

    auto img_msg = makeBgr8Image(vis, world_frame_, this->now());
    pub_img_->publish(img_msg);
  }

  // Members and parameters
  std::string base_frame_{"bubbleRob"};
  std::string world_frame_{"world"};
  double max_range_{4.0};
  int    ransac_n_samples_{400};
  double ransac_tol_{0.05};
  double tilt_deg_max_{20.0};
  double xmin_{-5}, xmax_{5}, ymin_{-5}, ymax_{5}, res_{0.1};
  int width_{0}, height_{0};
  int min_points_trav_{5};
  double z_floor_max_{1e9};
  bool  flip_vertical_{false};
  double prior_trav_{0.5}, p_hit_trav_{0.75}, p_hit_occ_{0.25};
  double range_sigma_{2.0}, min_weight_{0.2}, L_clip_{6.0};
  double thr_trav_{0.6}, thr_occ_{0.4};
  double L0_{0.0}, lz_trav_{0.0}, lz_occ_{0.0};
  cv::Mat L_map_;      
  cv::Mat obs_count_;   
  std::random_device rd_;
  std::mt19937 gen_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_og_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       pub_img_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  nav_msgs::msg::MapMetaData info_;
  std_msgs::msg::Header og_header_;

  BucketMap buckets_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloorMapNode>());
  rclcpp::shutdown();
  return 0;
}
