#include <footstep_planning_GZF/global_plan.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <iostream>
#include <glog/logging.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <grid_map_core/iterators/LineIterator.hpp>
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "footstep_palnning");
    ros::NodeHandle nh("~");
    ros::Publisher rawMapPub = nh.advertise<grid_map_msgs::GridMap>("raw_map", 1);
    ros::Publisher feasibleMapPub = nh.advertise<grid_map_msgs::GridMap>("feasible_map", 1);
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("path", 1);
    std::string rawMap_topic = "/raw_map";
    std::string feasibleMap_topic = "/feasible_map";
    std::string file_path = "/home/lichao/catkin_GZF/src/footstep_planning_GZF/map_bag/";
    grid_map::GridMap rawMap, feasibleMap;
    grid_map::GridMapRosConverter::loadFromBag(file_path + "raw_map.bag", rawMap_topic, rawMap);
    grid_map::GridMapRosConverter::loadFromBag(file_path + "feasible_map.bag", feasibleMap_topic, feasibleMap);

    Eigen::Vector3d s(0.1, 0, -10/57.3);
    Eigen::Vector3d e(3.5, 0.1, 10/57.3);
    hybridAstar a_star(s, e, rawMap);
    std::vector<Eigen::Vector3d> path;
    if (a_star.plan())
    {
        path = a_star.getPath();
        std::cout<<"get path success"<<std::endl;
    }
    else
    {
        std::cout<<"plan error"<<std::endl;
    }
    nav_msgs::Path path_3D;
    path_3D.header.frame_id  = "map";
    path_3D.header.stamp = ros::Time::now();
    bool not_last = false;
    Eigen::Vector3d last_pose;
    for (auto & p : path)
    {
        if (not_last) // 需要考虑前一次的插入多个点
        {
            if (rawMap.isInside(grid_map::Position(last_pose.x(), last_pose.y())) && rawMap.isInside(grid_map::Position(p.x(), p.y())))
            {
                int total_num = 0;
                nav_msgs::Path tmp_path;
                for (grid_map::LineIterator iterator(rawMap, grid_map::Position(last_pose.x(), last_pose.y()), grid_map::Position(p.x(), p.y())); !iterator.isPastEnd(); ++iterator)
                {
                    total_num++;
                }
                double delta = (p.z() - last_pose.z())/total_num;//递增变化的yaw角
                double index_n = 0;
                for (grid_map::LineIterator iterator(rawMap, grid_map::Position(last_pose.x(), last_pose.y()), grid_map::Position(p.x(), p.y())); !iterator.isPastEnd(); ++iterator)
                {
                    grid_map::Index index(*iterator);
                    grid_map::Position3 p3;
                    if (rawMap.getPosition3("elevation", index, p3))
                    {
                        Eigen::AngleAxisd ad(last_pose.z() + index_n * delta, Eigen::Vector3d::UnitZ());
                        Eigen::Quaterniond qd(ad.toRotationMatrix());
                        geometry_msgs::Pose tmp_p;
                        tmp_p.position.x = p3.x();
                        tmp_p.position.y = p3.y();
                        tmp_p.position.z = p3.z();
                        tmp_p.orientation.w = qd.w();
                        tmp_p.orientation.x = qd.x();
                        tmp_p.orientation.y = qd.y();
                        tmp_p.orientation.z = qd.z();
                        geometry_msgs::PoseStamped tmp_ps;
                        tmp_ps.pose = tmp_p;
                        tmp_path.poses.emplace_back(tmp_ps);
                    }
                    index_n++;
                }
                path_3D.poses.insert(path_3D.poses.end(), tmp_path.poses.begin(), tmp_path.poses.end());
            }
            last_pose = p;
        }
        else// 插入当前点
        {
            grid_map::Index index;
            if (rawMap.getIndex(grid_map::Position(p.x(), p.y()), index))
            {
                grid_map::Position3 p3;
                if (rawMap.getPosition3("elevation", index, p3))
                {
                    Eigen::AngleAxisd ad(p.z(), Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond qd(ad.toRotationMatrix());
                    geometry_msgs::Pose tmp_p;
                    tmp_p.position.x = p3.x();
                    tmp_p.position.y = p3.y();
                    tmp_p.position.z = p3.z();
                    tmp_p.orientation.w = qd.w();
                    tmp_p.orientation.x = qd.x();
                    tmp_p.orientation.y = qd.y();
                    tmp_p.orientation.z = qd.z();
                    geometry_msgs::PoseStamped tmp_ps;
                    tmp_ps.pose = tmp_p;
                    path_3D.poses.emplace_back(tmp_ps);
                }
                
            }
        }
        not_last = true;
        last_pose = p;
    }


    grid_map_msgs::GridMap rawMap_msg;
    grid_map::GridMapRosConverter::toMessage(rawMap, rawMap_msg);
    rawMap_msg.info.header.frame_id = "map";

    grid_map_msgs::GridMap feasibleMap_msg;
    grid_map::GridMapRosConverter::toMessage(feasibleMap, feasibleMap_msg);
    feasibleMap_msg.info.header.frame_id = "map";
    ros::Rate rate(100);
    while (ros::ok())
    {
        rawMapPub.publish(rawMap_msg);
        pub_path.publish(path_3D);
        feasibleMapPub.publish(feasibleMap_msg);
        rate.sleep();
    }

    return 0;
}