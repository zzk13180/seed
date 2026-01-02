/**
 * @file   robot_map_pose.cpp
 * @brief  机器人地图位姿发布节点
 *
 * 发布: /robot_map_pose (geometry_msgs/PoseStamped)
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_map_pose");
    ros::NodeHandle nh("~");

    std::string map_frame;
    std::string robot_base_frame;
    double rate_value;

    nh.param<std::string>("map_frame", map_frame, "map");
    nh.param<std::string>("robot_base_frame", robot_base_frame, "base_footprint");
    nh.param<double>("rate", rate_value, 10.0);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_map_pose", 100);
    tf::TransformListener listener;
    ros::Rate rate(rate_value);

    ROS_INFO("Robot map pose publisher started");
    ROS_INFO("  map_frame: %s", map_frame.c_str());
    ROS_INFO("  robot_base_frame: %s", robot_base_frame.c_str());
    ROS_INFO("  rate: %.1f Hz", rate_value);

    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform(map_frame, robot_base_frame, ros::Time(0), ros::Duration(15.0));
            listener.lookupTransform(map_frame, robot_base_frame, ros::Time(0), transform);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = map_frame;
            pose_msg.pose.position.x = transform.getOrigin().x();
            pose_msg.pose.position.y = transform.getOrigin().y();
            pose_msg.pose.orientation.z = transform.getRotation().z();
            pose_msg.pose.orientation.w = transform.getRotation().w();

            pose_pub.publish(pose_msg);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }
    return 0;
}
