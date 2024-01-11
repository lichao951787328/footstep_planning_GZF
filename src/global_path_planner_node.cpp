#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <footstep_planning_GZF/global_plan.h>
#include <Eigen/Geometry>
#include "humanoid_msgs/global_path_plan.h"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
double getYaw(geometry_msgs::Quaternion & q)
{
    Eigen::Quaterniond q_eigen;
    q_eigen.x() = q.x;
    q_eigen.y() = q.y;
    q_eigen.z() = q.z;
    q_eigen.w() = q.w;
    Eigen::Vector3d v_r = q_eigen.toRotationMatrix() * Eigen::Vector3d::UnitX();
    return atan(v_r.y()/v_r.x());
}

bool path_callback(humanoid_msgs::global_path_plan::Request &req, humanoid_msgs::global_path_plan::Response & res)
{
    ROS_INFO("Received request to global path planner");
    // 这个地方实际上是省略了roll，ptch，和z， 只保留了x，y，yaw
    // 将四元数投影到x-y平面，求yaw角
    // geometry_msgs::Quaternion start_q = ;
    Eigen::Vector3d start(req.start.position.x, req.start.position.y, getYaw(req.start.orientation));
    Eigen::Vector3d end(req.goal.position.x, req.goal.position.y, getYaw(req.goal.orientation));
    grid_map::GridMap global_map;
    grid_map::GridMapRosConverter::fromMessage(req.global_map, global_map);
    hybridAstar a_star(start, end, global_map);
    if (a_star.plan())
    {
        // 只存储x, y, yaw
        for (auto & p : a_star.getPath())
        {
            geometry_msgs::Pose point;
            point.position.x = p.x();
            point.position.y = p.y();
            point.position.z = p.z();// 注意，这里的z不是高度，而是yaw
            res.global_path.emplace_back(point);
        }
        ROS_INFO("get global path");
        return true;
    }
    else
    {
        ROS_INFO("can not search to the goal");
        return false;
    }
    
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planner_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("global_path_plan", path_callback);
    ROS_INFO("global path planning service is ready.");

    ros::spin();
    return 0;
}
