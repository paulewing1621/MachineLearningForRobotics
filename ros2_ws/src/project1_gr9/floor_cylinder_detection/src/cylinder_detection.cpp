#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

class CylinderMapNode : public rclcpp::Node {
public:
  CylinderMapNode()
  : rclcpp::Node("cylinder_map"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    gen_(rd_())
  {
    // Parameters
    declare_parameter<std::string>("~/world_frame", "world");
    declare_parameter<std::string>("~/base_frame",  "bubbleRob");
    declare_parameter<double>("~/max_range",4.0);   
    declare_parameter<double>("~/voxel_leaf",0.015); 
    declare_parameter<int>("~/ground_ransac", 400);
    declare_parameter<double>("~/ground_tol", 0.02);  
    declare_parameter<double>("~/belt_min",0.10);  
    declare_parameter<double>("~/belt_max",0.60); 
    declare_parameter<int>("~/n_samples", 1000);
    declare_parameter<double>("~/dist_tol",0.02);  
    declare_parameter<int>("~/min_inliers",70);
    declare_parameter<double>("~/min_cov_deg",200.0); 
    declare_parameter<double>("~/r_min",0.11);
    declare_parameter<double>("~/r_max",0.14);
    declare_parameter<double>("~/merge_center_tol",0.08);
    declare_parameter<double>("~/merge_radius_tol",0.02);
    declare_parameter<double>("~/normal_radius", 0.06);   
    declare_parameter<double>("~/nz_abs_max",0.30);   
    declare_parameter<double>("~/agree_cos_min",0.80);   
    declare_parameter<double>("~/agree_ratio", 0.60);   
    declare_parameter<double>("~/rstd_max", 0.010);  
    declare_parameter<double>("~/inside_ratio",0.08);   
    declare_parameter<int>("~/normal_dir_bins",36);   
    declare_parameter<int>("~/normal_dir_min_bins",12);   
    declare_parameter<double>("~/min_height",0.45); 
    declare_parameter<double>("~/center_max_shift", 0.015);

    world_frame_ = get_parameter("~/world_frame").as_string();
    base_frame_ = get_parameter("~/base_frame").as_string();
    max_range_ = get_parameter("~/max_range").as_double();
    voxel_leaf_ = get_parameter("~/voxel_leaf").as_double();
    ground_ransac_ = get_parameter("~/ground_ransac").as_int();
    ground_tol_= get_parameter("~/ground_tol").as_double();
    belt_min_ = get_parameter("~/belt_min").as_double();
    belt_max_ = get_parameter("~/belt_max").as_double();
    n_samples_ = get_parameter("~/n_samples").as_int();
    dist_tol_ = get_parameter("~/dist_tol").as_double();
    min_inliers_ = get_parameter("~/min_inliers").as_int();
    min_cov_rad_ = get_parameter("~/min_cov_deg").as_double() * M_PI / 180.0;
    r_min_= get_parameter("~/r_min").as_double();
    r_max_= get_parameter("~/r_max").as_double();
    merge_center_tol_ = get_parameter("~/merge_center_tol").as_double();
    merge_radius_tol_ = get_parameter("~/merge_radius_tol").as_double();

    normal_radius_= get_parameter("~/normal_radius").as_double();
    nz_abs_max_ = get_parameter("~/nz_abs_max").as_double();
    agree_cos_min_ = get_parameter("~/agree_cos_min").as_double();
    agree_ratio_min_ = get_parameter("~/agree_ratio").as_double();
    rstd_max_ = get_parameter("~/rstd_max").as_double();
    inside_ratio_max_ = get_parameter("~/inside_ratio").as_double();

    normal_dir_bins_= get_parameter("~/normal_dir_bins").as_int();
    normal_dir_min_bins_ = get_parameter("~/normal_dir_min_bins").as_int();
    min_height_ = get_parameter("~/min_height").as_double();
    center_max_shift_ = get_parameter("~/center_max_shift").as_double();

    auto qos = rclcpp::SensorDataQoS();
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/scans", qos, std::bind(&CylinderMapNode::cloudCb, this, std::placeholders::_1));
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/cylinders", 1);
  }

private:
  struct Plane { double a{0}, b{0}, c{1}, d{0}; bool ok{false}; };
  struct Cylinder {
    double cx{0}, cy{0}, r{0};
    double zmin{0}, zmax{0};
    int hits{0};
    std::array<uint8_t,36> bins{}; 
    double coverage{0.0};          
  };

  bool lookupTfSafe(const std::string& target,const std::string& source,
                    const rclcpp::Time& stamp, geometry_msgs::msg::TransformStamped& out) {
    try {
      out = tf_buffer_.lookupTransform(target, source, stamp, rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::ExtrapolationException&) {
      try {
        out = tf_buffer_.lookupTransform(target, source, rclcpp::Time(0),
                                         rclcpp::Duration::from_seconds(0.2));
        return true;
      } catch (...) {
        return false;
      }
    } catch (...) {
      return false;
    }
  }

  // RANSAC for the ground
  Plane ransacPlane(const pcl::PointCloud<pcl::PointXYZ>& pc,
                    const std::vector<int>& idx, int iters, double tol) {
    if ((int)idx.size() < 3) return {};
    std::uniform_int_distribution<int> d(0, (int)idx.size()-1);
    size_t best=0; Plane Pbest;
    for (int t=0;t<iters;++t) {
      int i1=d(gen_), i2=d(gen_), i3=d(gen_);
      while (i2==i1 || i3==i1 || i3==i2) { i2=d(gen_); i3=d(gen_); }
      const auto& A = pc[idx[i1]];
      const auto& B = pc[idx[i2]];
      const auto& C = pc[idx[i3]];
      double ux=B.x-A.x, uy=B.y-A.y, uz=B.z-A.z;
      double vx=C.x-A.x, vy=C.y-A.y, vz=C.z-A.z;
      double nx = uy*vz - uz*vy;
      double ny = uz*vx - ux*vz;
      double nz = ux*vy - uy*vx;
      double nn = std::sqrt(nx*nx+ny*ny+nz*nz);
      if (nn < 1e-9) continue;
      nx/=nn; ny/=nn; nz/=nn;
      if (std::fabs(nz) < 0.5) continue; 
      double a=nx,b=ny,c=nz,d0=-(a*A.x + b*A.y + c*A.z);
      size_t inl=0;
      for (int id: idx) {
        const auto& Q = pc[id];
        double dist = std::fabs(a*Q.x + b*Q.y + c*Q.z + d0);
        if (dist < tol) ++inl;
      }
      if (inl>best){ best=inl; Pbest={a,b,c,d0,true}; }
    }
    return Pbest;
  }

  // Circle from 3 points
  static bool circleFrom3(double x1,double y1,double x2,double y2,double x3,double y3,
                          double &cx,double &cy,double &r) {
    const double a = x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2;
    if (std::fabs(a) < 1e-12) return false;
    const double x1_2 = x1*x1 + y1*y1;
    const double x2_2 = x2*x2 + y2*y2;
    const double x3_2 = x3*x3 + y3*y3;
    cx = (x1_2*(y2 - y3) + x2_2*(y3 - y1) + x3_2*(y1 - y2)) / (2.0*a);
    cy = (x1_2*(x3 - x2) + x2_2*(x1 - x3) + x3_2*(x2 - x1)) / (2.0*a);
    r  = std::hypot(x1 - cx, y1 - cy);
    return std::isfinite(cx) && std::isfinite(cy) && std::isfinite(r);
  }

  // Angular coverage 
  static double angularCoverage(const std::vector<std::pair<double,double>>& xy,
                                double cx,double cy,double r,double tol) {
    std::vector<double> ang; ang.reserve(xy.size());
    for (auto& p: xy) {
      double d = std::fabs(std::hypot(p.first - cx, p.second - cy) - r);
      if (d < tol) ang.push_back(std::atan2(p.second - cy, p.first - cx));
    }
    if (ang.size() < 3) return 0.0;
    std::sort(ang.begin(), ang.end());
    double max_gap=0.0;
    for (size_t i=0;i<ang.size();++i) {
      double a = ang[(i+1)%ang.size()] - ang[i];
      if (a<0) a+=2*M_PI;
      max_gap = std::max(max_gap,a);
    }
    return 2*M_PI - max_gap;
  }

  // Update coverage bins
  void updateBins(Cylinder& c, const std::vector<std::pair<double,double>>& xy) {
    for (auto& p: xy) {
      double d = std::fabs(std::hypot(p.first - c.cx, p.second - c.cy) - c.r);
      if (d < dist_tol_) {
        double a = std::atan2(p.second - c.cy, p.first - c.cx);
        if (a<0) a+=2*M_PI;
        int b = std::clamp(int(a/(2*M_PI)*36.0),0,35);
        c.bins[(size_t)b]=1;
      }
    }
    int cnt=0; for (auto v:c.bins) cnt+= (v?1:0);
    c.coverage = (double)cnt/36.0 * 2.0 * M_PI;
  }

  // Nmber of non-empty bins
  int normalDirectionCoverage(
    const std::vector<int>& ids,
    const pcl::PointCloud<pcl::PointXYZ>& pc,
    const pcl::PointCloud<pcl::Normal>& normals,
    const std::unordered_map<int,int>& id2local)
  {
    std::vector<uint8_t> bins((size_t)normal_dir_bins_, 0);
    for (int id : ids){
      auto it = id2local.find(id);
      if (it==id2local.end()) continue;
      const auto& n = normals.at(it->second);
      if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y)) continue;
      double ang = std::atan2(n.normal_y, n.normal_x);
      if (ang < 0) ang += 2.0*M_PI;
      int b = (int)std::floor(ang / (2.0*M_PI) * normal_dir_bins_);
      b = std::clamp(b, 0, normal_dir_bins_-1);
      bins[(size_t)b] = 1;
    }
    int cnt=0; for (auto v: bins) cnt += (v?1:0);
    return cnt;
  }


  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // TF: robot and cloud to world
    geometry_msgs::msg::TransformStamped tf_rb, tf_cloud;
    if (!lookupTfSafe(world_frame_, base_frame_, msg->header.stamp, tf_rb)) return;
    if (!lookupTfSafe(world_frame_, msg->header.frame_id, msg->header.stamp, tf_cloud)) return;

    sensor_msgs::msg::PointCloud2 cloud_world;
    tf2::doTransform(*msg, cloud_world, tf_cloud);

    // PCL and voxel
    pcl::PointCloud<pcl::PointXYZ> pc_raw;
    pcl::fromROSMsg(cloud_world, pc_raw);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(pc_raw.makeShared());
    vg.setLeafSize((float)voxel_leaf_, (float)voxel_leaf_, (float)voxel_leaf_);
    pcl::PointCloud<pcl::PointXYZ> pc;
    vg.filter(pc);

    const double rx = tf_rb.transform.translation.x;
    const double ry = tf_rb.transform.translation.y;

    // Distance ROI
    std::vector<int> idx; idx.reserve(pc.size());
    for (int i=0;i<(int)pc.size();++i) {
      const auto& P=pc[i];
      if (!std::isfinite(P.x)||!std::isfinite(P.y)||!std::isfinite(P.z)) continue;
      const double dx=P.x-rx, dy=P.y-ry;
      if (max_range_>0.0 && (dx*dx+dy*dy)>max_range_*max_range_) continue;
      idx.push_back(i);
    }
    if ((int)idx.size()<3){ publishMarkers(msg->header.stamp); return; }

    // Ground RANSAC
    Plane G = ransacPlane(pc, idx, ground_ransac_, ground_tol_);
    if (!G.ok){ publishMarkers(msg->header.stamp); return; }

    // Belt above ground
    std::vector<int> belt; belt.reserve(idx.size());
    for (int id: idx){
      const auto& P=pc[id];
      double delta = G.a*P.x + G.b*P.y + G.c*P.z + G.d; 
      if (delta>=belt_min_ && delta<=belt_max_) belt.push_back(id);
    }
    if ((int)belt.size()<std::max(min_inliers_,3)){ publishMarkers(msg->header.stamp); return; }

    // Normals and filter by |n.z|
    pcl::PointCloud<pcl::PointXYZ>::Ptr belt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    belt_cloud->reserve(belt.size());
    for (int id: belt) belt_cloud->push_back(pc[id]);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(belt_cloud);
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_radius_);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    std::vector<int> belt_horiz; belt_horiz.reserve(belt.size());
    std::vector<std::pair<double,double>> xy_all; xy_all.reserve(belt.size());
    std::unordered_map<int,int> id2local; id2local.reserve(belt.size());
    for (size_t i=0;i<belt_cloud->size();++i){
      const auto& n = normals->at(i);
      if (std::isfinite(n.normal_x) && std::isfinite(n.normal_y) && std::isfinite(n.normal_z)) {
        if (std::fabs(n.normal_z) < nz_abs_max_) {
          int gid = belt[(int)i];
          id2local[gid] = (int)i;
          belt_horiz.push_back(gid);
          const auto& P = pc[gid];
          xy_all.emplace_back(P.x,P.y);
        }
      }
    }
    if ((int)belt_horiz.size() < std::max(min_inliers_, 30)) { publishMarkers(msg->header.stamp); return; }

    std::vector<Cylinder> detections;
    std::vector<int> active = belt_horiz;

    while ((int)active.size()>=std::max(min_inliers_,30)) {
      const int N=(int)active.size();
      if (N<3) break;
      int best_inl=0; double bcx=0,bcy=0,br=0;

      for (int it=0; it<n_samples_; ++it) {
        std::uniform_int_distribution<int> dS(0, (int)active.size()-1);
        if ((int)active.size()<3) break;
        int i1=dS(gen_), i2=dS(gen_), i3=dS(gen_);
        int guard=0; while((i2==i1||i3==i1||i3==i2)&&guard++<20){i2=dS(gen_); i3=dS(gen_);}
        if (i1>=N||i2>=N||i3>=N) continue;
        const auto& P1=pc[active[i1]];
        const auto& P2=pc[active[i2]];
        const auto& P3=pc[active[i3]];
        double cx,cy,r;
        if(!circleFrom3(P1.x,P1.y,P2.x,P2.y,P3.x,P3.y,cx,cy,r)) continue;
        if (r<r_min_||r>r_max_) continue;

        // Count inliers and normal radial agreement
        int ninl=0, nagree=0;
        for (int id: active){
          const auto& Q=pc[id];
          const double rr = std::hypot(Q.x-cx,Q.y-cy);
          const double dr = std::fabs(rr - r);
          if (dr < dist_tol_) {
            ++ninl;
            auto itloc = id2local.find(id);
            if (itloc!=id2local.end()){
              const auto& n = normals->at(itloc->second);
              const double nx=n.normal_x, ny=n.normal_y;
              const double nxy = std::sqrt(nx*nx+ny*ny);
              if (nxy>1e-6 && rr>1e-6){
                const double cosang = (nx*(Q.x-cx) + ny*(Q.y-cy)) / (nxy * rr);
                if (cosang > agree_cos_min_) ++nagree;
              }
            }
          }
        }
        if (ninl > best_inl) {
          double agree_ratio = ninl>0 ? (double)nagree/ninl : 0.0;
          if (agree_ratio >= agree_ratio_min_) {
            best_inl=ninl; bcx=cx; bcy=cy; br=r;
          }
        }
      }

      if (best_inl<min_inliers_) break;

      // Angular coverage on belt points
      const double cov = angularCoverage(xy_all, bcx, bcy, br, dist_tol_);
      if (cov < min_cov_rad_) {
        std::vector<std::pair<double,int>> dists; dists.reserve(active.size());
        for (int id: active){
          const auto& Q=pc[id];
          dists.emplace_back(std::fabs(std::hypot(Q.x-bcx,Q.y-bcy)-br), id);
        }
        std::sort(dists.begin(), dists.end(),
                  [](auto&a,auto&b){return a.first<b.first;});
        int rm = std::max(1,(int)(0.1*dists.size()));
        std::vector<int> next; next.reserve(active.size());
        for (size_t i=rm;i<dists.size();++i) next.push_back(dists[i].second);
        active.swap(next);
        continue;
      }

      // Collect inliers, z stats and inside ratio
      std::vector<int> inlier_ids; inlier_ids.reserve(active.size());
      std::vector<int> next; next.reserve(active.size());
      double zmin=+1e9, zmax=-1e9;
      int inside=0, ninl=0;
      for (int id: active){
        const auto& Q=pc[id];
        const double rr = std::hypot(Q.x-bcx,Q.y-bcy);
        const double dr = std::fabs(rr - br);
        if (dr<dist_tol_){
          inlier_ids.push_back(id);
          zmin = std::min(zmin, (double)Q.z);
          zmax = std::max(zmax, (double)Q.z);
          ++ninl;
          if (rr < br - dist_tol_) ++inside;
        } else next.push_back(id);
      }

      if (ninl < min_inliers_) { active.swap(next); continue; }
      const double inside_ratio = (double)inside / (double)ninl;
      if (inside_ratio > inside_ratio_max_) { active.swap(next); continue; }

      if (std::isfinite(zmin) && std::isfinite(zmax)){
        if ((zmax - zmin) < min_height_) { active.swap(next); continue; }
      }

      // Normal direction coverage to reject prisms
      int ndirs = normalDirectionCoverage(inlier_ids, pc, *normals, id2local);
      if (ndirs < normal_dir_min_bins_) { active.swap(next); continue; }

      // Radius constancy vs z (3 slices)
      if (std::isfinite(zmin) && std::isfinite(zmax) && (zmax - zmin) >= 0.05){
        auto r_stats = [&](double a,double b){
          double m=0; int k=0; double var=0;
          for(int id: inlier_ids){
            const auto& V=pc[id];
            if (V.z>=a && V.z<b){ const double rr = std::hypot(V.x-bcx,V.y-bcy); m += rr; ++k; }
          }
          if (k<10) return std::pair<bool,std::pair<double,double>>(false,{0,0});
          m/=k;
          for(int id: inlier_ids){
            const auto& V=pc[id];
            if (V.z>=a && V.z<b){
              const double rr = std::hypot(V.x-bcx,V.y-bcy);
              var += (rr-m)*(rr-m);
            }
          }
          var/=k;
          return std::pair<bool,std::pair<double,double>>(true,{m,std::sqrt(var)});
        };
        double dz=(zmax - zmin)/3.0;
        auto s0=r_stats(zmin, zmin+dz);
        auto s1=r_stats(zmin+dz, zmin+2*dz);
        auto s2=r_stats(zmin+2*dz, zmax);
        if (!(s0.first&&s1.first&&s2.first)) { active.swap(next); continue; }
        double rstd = std::max({s0.second.second, s1.second.second, s2.second.second});
        if (rstd > rstd_max_) { active.swap(next); continue; }
      }

      // Center stability vs z (coaxiality)
      auto center_stats = [&](double a,double b){
        double sx=0, sy=0; int k=0;
        for(int id: inlier_ids){
          const auto& V=pc[id];
          if (V.z>=a && V.z<b){ sx += V.x; sy += V.y; ++k; }
        }
        if (k<8) return std::pair<bool,std::pair<double,double>>(false,{0,0});
        return std::pair<bool,std::pair<double,double>>(true,{sx/k, sy/k});
      };
      if (std::isfinite(zmin) && std::isfinite(zmax)){
        double dz = (zmax - zmin)/3.0;
        auto c0 = center_stats(zmin, zmin+dz);
        auto c1 = center_stats(zmin+dz, zmin+2*dz);
        auto c2 = center_stats(zmin+2*dz, zmax);
        if (!(c0.first && c1.first && c2.first)) { active.swap(next); continue; }
        double shift01 = std::hypot(c0.second.first - c1.second.first, c0.second.second - c1.second.second);
        double shift12 = std::hypot(c1.second.first - c2.second.first, c1.second.second - c2.second.second);
        double shift02 = std::hypot(c0.second.first - c2.second.first, c0.second.second - c2.second.second);
        double max_shift = std::max({shift01, shift12, shift02});
        if (max_shift > center_max_shift_) { active.swap(next); continue; }
      }

      // Save detection and remove inliers
      Cylinder cyl; cyl.cx=bcx; cyl.cy=bcy; cyl.r=br;
      cyl.zmin=zmin; cyl.zmax=zmax;
      detections.push_back(cyl);
      active.swap(next);
    }

    // Merge and accumulate coverage
    for (auto& d: detections){
      bool merged=false;
      for (auto& c: map_){
        const double dc = std::hypot(c.cx-d.cx, c.cy-d.cy);
        if (dc<merge_center_tol_ && std::fabs(c.r-d.r)<merge_radius_tol_){
          const double a=0.4;
          c.cx=(1-a)*c.cx + a*d.cx;
          c.cy=(1-a)*c.cy + a*d.cy;
          c.r =(1-a)*c.r  + a*d.r;
          c.zmin=std::min(c.zmin,d.zmin);
          c.zmax=std::max(c.zmax,d.zmax);
          c.hits+=1;
          updateBins(c, xy_all);
          merged=true; break;
        }
      }
      if (!merged){ auto nc=d; nc.hits=1; updateBins(nc, xy_all); map_.push_back(nc); }
    }

    publishMarkers(msg->header.stamp);
  }

  void publishMarkers(const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray arr;
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id=world_frame_; m.header.stamp=stamp;
      m.ns="cylinders"; m.id=0; m.action=visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(m);
    }

    int id=1;
    for (const auto& c: map_){
      visualization_msgs::msg::Marker m;
      m.header.frame_id=world_frame_; m.header.stamp=stamp;
      m.ns="cylinders"; m.id=id++;
      m.type=visualization_msgs::msg::Marker::CYLINDER;
      m.action=visualization_msgs::msg::Marker::ADD;

      const double zc=0.5*(c.zmin+c.zmax);
      const double h=std::max(0.05, c.zmax-c.zmin);
      m.pose.position.x=c.cx; m.pose.position.y=c.cy; m.pose.position.z=zc;
      m.pose.orientation.w=1.0;
      m.scale.x=2.0*c.r; m.scale.y=2.0*c.r; m.scale.z=h;

      double alpha = 0.2 + 0.6 * std::min(1.0, c.coverage / min_cov_rad_);
      m.color.r=0.1f; m.color.g=0.9f; m.color.b=0.1f; m.color.a=(float)alpha;
      arr.markers.push_back(m);
    }
    pub_markers_->publish(arr);
  }

  // Members
  std::string world_frame_, base_frame_;
  double max_range_{4.0}, voxel_leaf_{0.015};
  int ground_ransac_{400}; double ground_tol_{0.02};
  double belt_min_{0.10}, belt_max_{0.60};

  int n_samples_{1000}; double dist_tol_{0.02};
  int min_inliers_{70}; double min_cov_rad_{200.0*M_PI/180.0};
  double r_min_{0.11}, r_max_{0.14};
  double merge_center_tol_{0.08}, merge_radius_tol_{0.02};

  double normal_radius_{0.06};
  double nz_abs_max_{0.30};
  double agree_cos_min_{0.80};
  double agree_ratio_min_{0.60};
  double rstd_max_{0.010};
  double inside_ratio_max_{0.08};

  int normal_dir_bins_{36};
  int normal_dir_min_bins_{12};
  double min_height_{0.45};
  double center_max_shift_{0.015};

  std::vector<Cylinder> map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::random_device rd_; std::mt19937 gen_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<CylinderMapNode>());
  rclcpp::shutdown();
  return 0;
}
