/**
 * @file   angle_controller.cpp
 * @brief  角度控制器节点 - 控制机器人旋转到指定角度
 *
 * 订阅: odom (nav_msgs/Odometry), target_angle (std_msgs/Float64)
 * 发布: cmd_vel (geometry_msgs/Twist), current_angle (std_msgs/Float64)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <deque>
#include <algorithm>
#include <cmath>
#include <numeric>

/** @brief 角度控制器类，使用 PID 算法控制机器人旋转到目标角度 */
class AngleController
{
public:
  /** @brief 构造函数，初始化订阅/发布器及 PID 参数 */
  AngleController() : is_angle_control_active_(false), previous_error_(0.0), integral_(0.0), last_filtered_angle_(0.0)
  {
    odom_sub_ = nh_.subscribe("odom", 10, &AngleController::odomCallback, this);
    angle_sub_ = nh_.subscribe("target_angle", 10, &AngleController::angleCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    current_angle_pub_ = nh_.advertise<std_msgs::Float64>("current_angle", 10);

    ros::NodeHandle pnh("~");
    pnh.param("Kp", Kp_, 0.5);
    pnh.param("Ki", Ki_, 0.01);
    pnh.param("Kd", Kd_, 0.1);
    pnh.param("max_angular_vel", max_angular_vel_, 0.1);
    pnh.param("threshold", threshold_, 0.05);
  }

  /** @brief 里程计回调，获取当前角度并执行控制 */
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double current_angle = filterAngle(yaw);
    last_filtered_angle_ = current_angle;

    if (is_angle_control_active_)
    {
      angleControlLoop(current_angle);
    }

    std_msgs::Float64 current_angle_msg;
    current_angle_msg.data = current_angle;
    current_angle_pub_.publish(current_angle_msg);
  }

  /** @brief 目标角度回调，接收旋转指令并激活控制 */
  void angleCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    target_angle_ = msg->data;
    target_angle_ = std::atan2(std::sin(target_angle_), std::cos(target_angle_)); // 归一化到 [-pi, pi]

    double err = std::atan2(std::sin(target_angle_ - last_filtered_angle_), std::cos(target_angle_ - last_filtered_angle_));
    if (std::fabs(err) < threshold_)
    {
      ROS_INFO("AngleController: target angle within threshold, no rotation needed.");
      geometry_msgs::Twist stop_msg;
      stop_msg.angular.z = 0.0;
      cmd_vel_pub_.publish(stop_msg);
      return;
    }

    is_angle_control_active_ = true;
    integral_ = 0.0;
  }

  /** @brief PID 角度控制主循环 */
  void angleControlLoop(double current_angle)
  {
    double error = target_angle_ - current_angle;
    error = atan2(sin(error), cos(error)); // 归一化误差

    integral_ += error;
    double derivative = error - previous_error_;
    double angular_velocity = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    angular_velocity = std::max(std::min(angular_velocity, max_angular_vel_), -max_angular_vel_);

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = angular_velocity;
    cmd_vel_pub_.publish(cmd_vel_msg);

    previous_error_ = error;

    if (std::fabs(error) < threshold_)
    {
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_.publish(cmd_vel_msg);
      is_angle_control_active_ = false;
      integral_ = 0.0;
    }
  }

  /** @brief 圆形平均滤波，处理角度跨越周期问题 */
  double filterAngle(double new_angle)
  {
    const int buffer_size = 10;
    static std::deque<double> sin_buffer;
    static std::deque<double> cos_buffer;

    sin_buffer.push_back(std::sin(new_angle));
    cos_buffer.push_back(std::cos(new_angle));
    if (sin_buffer.size() > buffer_size)
    {
      sin_buffer.pop_front();
      cos_buffer.pop_front();
    }

    double sum_sin = std::accumulate(sin_buffer.begin(), sin_buffer.end(), 0.0);
    double sum_cos = std::accumulate(cos_buffer.begin(), cos_buffer.end(), 0.0);

    return std::atan2(sum_sin / sin_buffer.size(), sum_cos / cos_buffer.size());
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber angle_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher current_angle_pub_;
  bool is_angle_control_active_;
  double target_angle_;
  double previous_error_;
  double integral_;
  double Kp_, Ki_, Kd_;
  double max_angular_vel_;
  double threshold_;
  double last_filtered_angle_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angle_controller");
  AngleController controller;
  ros::spin();
  return 0;
}
