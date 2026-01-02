/**
 * @file   global_relocation.cpp
 * @brief  全局重定位节点 - 使用激光与地图匹配搜索最佳位姿
 *
 * 订阅: /initialpose_ori, /map, /scan
 * 发布: /initialpose (优化后的位姿)
 */

#include <cmath>
#include <ros/ros.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/utils.h"

namespace seed::slam
{

#define degree2rad (M_PI / 180.0)

/** @brief 全局重定位类，通过激光-地图匹配在给定初始位姿附近搜索最佳位姿 */
class GlobalRelocation
{
public:
  GlobalRelocation();
  ~GlobalRelocation() = default;

private:
  /** @brief 初始位姿接收回调 */
  void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

  /** @brief 地图接收回调 */
  void mapReceived(const nav_msgs::OccupancyGridConstPtr &msg);

  /** @brief 激光数据接收回调 */
  void laserReceived(const sensor_msgs::LaserScanConstPtr &msg);

  /** @brief 范围搜索重定位，返回最佳位姿的匹配得分 */
  auto rangeRelocate(geometry_msgs::PoseWithCovarianceStamped &best_pose,
                     double dist_range = 2.0, double angle_range = 60.0 * degree2rad,
                     double dist_reso = 0.05, double angle_reso = 1.0 * degree2rad) -> int;

private:
  ros::Publisher initial_pose_pub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber laser_sub_;

  ros::ServiceClient finish_traj_srv_;
  ros::ServiceClient start_traj_srv_;

  nav_msgs::OccupancyGrid map_{};
  sensor_msgs::LaserScan laser_{};

  std::vector<std::pair<double, double>> cos_sin_table_{};

  std::atomic_bool on_going_{false};
  std::atomic_bool got_laser_info_{false};
};

GlobalRelocation::GlobalRelocation()
{
  ros::NodeHandle nh("~");

  initial_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  initial_pose_sub_ = nh.subscribe("/initialpose_ori", 1, &GlobalRelocation::initialPoseReceived, this);
  map_sub_ = nh.subscribe("/map", 1, &GlobalRelocation::mapReceived, this);
  laser_sub_ = nh.subscribe("/scan", 1, &GlobalRelocation::laserReceived, this);
}

void GlobalRelocation::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  static int32_t traj_id = 0;
  on_going_ = true;
  auto best_pose = *msg;
  ROS_INFO("Receive original initial pose for amcl node [%.3f, %.3f, %.3f]",
           msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
  auto start_time = ros::Time::now();
  auto score = rangeRelocate(best_pose);
  ROS_WARN("Get new best pose [%.3f, %.3f, %.3f] score [%d], time go %.3f",
           best_pose.pose.pose.position.x, best_pose.pose.pose.position.y, tf2::getYaw(best_pose.pose.pose.orientation),
           score, (ros::Time::now() - start_time).toSec());
  initial_pose_pub_.publish(best_pose);
  on_going_ = false;
}

void GlobalRelocation::mapReceived(const nav_msgs::OccupancyGridConstPtr &msg)
{
  if (on_going_)
    return;
  map_ = *msg;
}

void GlobalRelocation::laserReceived(const sensor_msgs::LaserScanConstPtr &msg)
{
  if (on_going_)
    return;
  laser_ = *msg;
  if (got_laser_info_)
    return;
  auto start_time = ros::Time::now();
  got_laser_info_ = true;
  cos_sin_table_.resize(laser_.ranges.size());
  for (size_t i = 0; i < laser_.ranges.size(); ++i)
  {
    cos_sin_table_[i].first = std::cos(laser_.angle_min + i * laser_.angle_increment);
    cos_sin_table_[i].second = std::sin(laser_.angle_min + i * laser_.angle_increment);
  }
  ROS_WARN("Calculate table size %lu time go %.3f",
           cos_sin_table_.size(), (ros::Time::now() - start_time).toSec());
}

auto GlobalRelocation::rangeRelocate(geometry_msgs::PoseWithCovarianceStamped &best_pose,
                                     double dist_range, double angle_range, double dist_reso, double angle_reso) -> int
{
  auto mapValid = [&](double x, double y)
  {
    auto i = std::floor((x - map_.info.origin.position.x) / map_.info.resolution + 0.5);
    auto j = std::floor((y - map_.info.origin.position.y) / map_.info.resolution + 0.5);
    return (i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height);
  };
  auto mapObstacle = [&](double x, double y)
  {
    auto i = std::floor((x - map_.info.origin.position.x) / map_.info.resolution + 0.5);
    auto j = std::floor((y - map_.info.origin.position.y) / map_.info.resolution + 0.5);
    auto idx = static_cast<size_t>(i + j * map_.info.width);
    return map_.data[idx] > 50;
  };
  auto calcuScore = [&](double x, double y, double cos_val, double sin_val)
  {
    const double laser2base = 0.29;
    auto score = 0;
    // transform to laser frame
    auto laser_x = x + laser2base * cos_val;
    auto laser_y = y + laser2base * sin_val;
    for (size_t i = 0; i < laser_.ranges.size(); i += 10)
    {
      if (laser_.ranges[i] < laser_.range_min || laser_.ranges[i] >= laser_.range_max)
        continue;
      auto cos_pth = cos_val * cos_sin_table_[i].first - sin_val * cos_sin_table_[i].second;
      auto sin_pth = sin_val * cos_sin_table_[i].first + cos_val * cos_sin_table_[i].second;
      auto px = laser_x + laser_.ranges[i] * cos_pth;
      auto py = laser_y + laser_.ranges[i] * sin_pth;
      if (!mapValid(px, py))
        continue;
      if (mapObstacle(px, py))
        ++score;
    }

    return score;
  };

  auto min_x = best_pose.pose.pose.position.x - dist_range / 2.0;
  auto max_x = best_pose.pose.pose.position.x + dist_range / 2.0;
  auto min_y = best_pose.pose.pose.position.y - dist_range / 2.0;
  auto max_y = best_pose.pose.pose.position.y + dist_range / 2.0;
  auto min_th = tf2::getYaw(best_pose.pose.pose.orientation) - angle_range / 2.0;
  auto max_th = tf2::getYaw(best_pose.pose.pose.orientation) + angle_range / 2.0;

  auto score = 0;
  double target_x = best_pose.pose.pose.position.x;
  double target_y = best_pose.pose.pose.position.y;
  double target_th = tf2::getYaw(best_pose.pose.pose.orientation);

  for (auto th = min_th; th < max_th; th += angle_reso)
  {
    auto cos_th = std::cos(th);
    auto sin_th = std::sin(th);
    for (auto x = min_x; x <= max_x; x += dist_reso)
    {
      for (auto y = min_y; y <= max_y; y += dist_reso)
      {
        if (!mapValid(x, y) || mapObstacle(x, y))
          continue;
        auto temp = calcuScore(x, y, cos_th, sin_th);
        if (temp > score)
        {
          score = temp;
          target_x = x;
          target_y = y;
          target_th = th;
        }
      }
    }
  }

  best_pose.pose.pose.position.x = target_x;
  best_pose.pose.pose.position.y = target_y;
  best_pose.pose.pose.orientation.z = std::sin(target_th / 2.0);
  best_pose.pose.pose.orientation.w = std::cos(target_th / 2.0);
  return score;
}

} // namespace seed::slam

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_relocation");

  seed::slam::GlobalRelocation global_relocation;
  ros::spin();

  return 0;
}
