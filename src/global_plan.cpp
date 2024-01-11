#include "footstep_planning_GZF/global_plan.h"
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
hybridAstar::hybridAstar(Eigen::Vector3d & start, Eigen::Vector3d & end, grid_map::GridMap & map)
{
    start_P = std::make_shared<node>(start);
    goal_P = std::make_shared<node>(end);
    map_ = map;
    close_set_.clear();
    while (!p_queue_.empty())
    {
        p_queue_.pop();
    }
    
    // grid_map::Index start_index, goal_inedx;
    // map_.getIndex(grid_map::Position(start_P->x, start_P->y), start_index);
    // map_.getIndex(grid_map::Position(goal_P->x, goal_P->y), goal_inedx);
    // LOG(INFO)<<start_index.transpose()<<" "<<goal_inedx.transpose();

}

bool hybridAstar::GetHumanoidGuidence(NodePtr n)
{
    if (n == nullptr)
    {
        return false;
    }
    if (goal_P == nullptr)
    {
        return false;
    }
    
    if (std::isnan(n->x) || std::isnan(n->y) || std::isnan(n->yaw) || std::isnan(goal_P->x) || std::isnan(goal_P->y) || std::isnan(goal_P->yaw))
    {
        LOG(INFO)<<"error current node or goal";
        return false;
    }
    // if (std::isnan(end_.x) || std::isnan(end_.y) || std::isnan(end_.yaw))
    // {
    //     return false;
    // }
    
    if (std::sqrt(std::pow(n->x - goal_P->x, 2) + std::pow(n->y - goal_P->y, 2)) < 0.05 && std::sqrt(n->yaw - goal_P->yaw) < 2/57.3)
    {
        return true;
    }
    return false;
}

std::string hybridAstar::get_node_indexS(NodePtr n)
{
    grid_map::Index node_index;
    map_.getIndex(grid_map::Position(n->x, n->y), node_index);
    std::string index = std::to_string(node_index.x()) + "_" + std::to_string(node_index.y())+"_"+std::to_string((int)(n->yaw * 57.3));
    return index;
}

std::vector<NodePtr> hybridAstar::getNextNodes(NodePtr current_node)
{
    std::vector<NodePtr> next_nodes;
    for (auto & angle : sample_angle)
    {
        for (auto & length: sample_lengths)
        {
            NodePtr next_node = getNodeFrom(current_node, angle, length);
            if (map_.isInside(grid_map::Position(next_node->x, next_node->y)) && isValid(next_node))
            {
                next_node->cost = computeCost(next_node);
                next_nodes.emplace_back(next_node);
            }
        }
    }
    // LOG(INFO)<<"nodes size "<<next_nodes.size();
    // LOG(INFO)<<"GET NEXT NODES FINISH";
    return next_nodes;
}

bool hybridAstar::isValid(NodePtr n)
{
    grid_map::Index n_index;
    map_.getIndex(grid_map::Position(n->x, n->y), n_index);
    // LOG(INFO)<<"tmp index: "<<n_index.transpose();
    grid_map::Position3 p3;
    return map_.getPosition3("elevation", n_index, p3);
}


// 误差项分为三个部分：垂直误差、平行误差、角度误差
double hybridAstar::computeCost(NodePtr n)
{
    Eigen::AngleAxisd aa(goal_P->yaw + 3.1415926/2, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d normal = aa.toRotationMatrix() * Eigen::Vector3d::UnitX();
    Eigen::Vector3d v(n->x - goal_P->x, n->y - goal_P->y, 0);
    // 这个3是一个系数
    double cost_c = std::abs(normal.dot(v)) * 3;
    Eigen::AngleAxisd ap(goal_P->yaw, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d normal_p = ap.toRotationMatrix()*Eigen::Vector3d::UnitX();
    double cost_p = std::abs(normal_p.dot(v));
    // 这个后面的0.07也是一个系数
    double cost_r = std::abs(n->yaw - goal_P->yaw) * 57.3 * 0.07;// 1度对应0.01m的误差
    return cost_c + cost_r + cost_p;
}


NodePtr hybridAstar::getNodeFrom(NodePtr current_node, double angle, double length)
{
    double dx = length * std::cos(angle + current_node->yaw);
    double dy = length * std::sin(angle + current_node->yaw);
    // LOG(INFO)<<"dx = "<<dx<<" dy = "<<dy;
    Eigen::Vector3d node_para(current_node->x + dx, current_node->y + dy, current_node->yaw + angle);
    
    NodePtr next_node = std::make_shared<node>(node_para);
    next_node->prenode = current_node;
    return next_node;
}

bool hybridAstar::plan()
{
    // debug
    // cv::Mat debug_image(map_.getSize().x(), map_.getSize().y(), CV_8UC3);
    // LOG(INFO)<<"grid map size: "<<map_.getSize().transpose();
    // double scale = 4.0;
    // cv::namedWindow("debug window", cv::WINDOW_NORMAL);
    // cv::resizeWindow("debug window", static_cast<int>(debug_image.cols * scale), static_cast<int>(debug_image.rows * scale));
    // for (int i = 0; i < debug_image.rows; i++)
    // {
    //   for (int j = 0; j < debug_image.cols; j++)
    //   {
    //     grid_map::Index index(i, j);
    //     grid_map::Position3 p3;
    //     map_.getPosition3("elevation", index, p3);
    //     if (!std::isnan(p3.z()))
    //     {
    //         debug_image.at<cv::Vec3b>(i,j) = cv::Vec3b(255, 255, 255);
    //     }
    //   }
    // }
    // cv::imshow("debug window", debug_image);
    // cv::waitKey(0);
    
    p_queue_.push(start_P);
    int iter = 0;
    while (!p_queue_.empty())
    {
        // debug
        // LOG(INFO)<<"in iterator...";
        // for (int i = 0; i < debug_image.rows; i++)
        // {
        //     for (int j = 0; j < debug_image.cols; j++)
        //     {
        //         grid_map::Index index(i, j);
        //         grid_map::Position3 p3;
        //         map_.getPosition3("elevation", index, p3);
        //         if (!std::isnan(p3.z()))
        //         {
        //             debug_image.at<cv::Vec3b>(i,j) = cv::Vec3b(255, 255, 255);
        //         }
        //     }
        // }
        // auto tmp_queue = p_queue_;
        // while (!tmp_queue.empty())
        // {
        //     auto n = tmp_queue.top();
        //     tmp_queue.pop();
        //     grid_map::Position p2(n->x, n->y);
        //     grid_map::Index index;
        //     map_.getIndex(p2, index);
        //     // LOG(INFO)<<index.transpose();
        //     debug_image.at<cv::Vec3b>(index.x() ,index.y()) = cv::Vec3b(255, 0, 0);
        // }
        // cv::imshow("debug window", debug_image);
        // cv::waitKey(0);

        auto current_node = p_queue_.top();

        // debug
        // {
        //     grid_map::Position p2(current_node->x, current_node->y);
        //     grid_map::Index index;
        //     map_.getIndex(p2, index);
        //     // LOG(INFO)<<index.transpose();
        //     // LOG(INFO)<<current_node->x<<" "<<current_node->y<<" "<<current_node->yaw * 57.3;
        //     debug_image.at<cv::Vec3b>(index.x() ,index.y()) = cv::Vec3b(0, 255, 0);
        // }
        // cv::imshow("debug window", debug_image);
        // cv::waitKey(0);

        p_queue_.pop();
        std::string Sindex = get_node_indexS(current_node);
        if (close_set_.find(Sindex) != close_set_.end())
        {
            // LOG(INFO)<<"has detect";
            continue;
        }
        else
        {
            // LOG(INFO)<<"INSERT";
            close_set_.insert(Sindex);
        }
        if (GetHumanoidGuidence(current_node))
        {
            LOG(INFO)<<"arrive goal";
            end_P = current_node;
            break;
        }
        // LOG(INFO)<<"GET NEXT NODES";
        for (auto & next_node : getNextNodes(current_node))
        {
            if (map_.isInside(grid_map::Position(next_node->x, next_node->y)))
            {
                if (isValid(next_node))
                {
                    if (close_set_.find(get_node_indexS(next_node)) == close_set_.end())
                    {
                        p_queue_.push(next_node);
                        // debug
                        // {
                        //     grid_map::Position p2(next_node->x, next_node->y);
                        //     grid_map::Index index;
                        //     map_.getIndex(p2, index);
                        //     debug_image.at<cv::Vec3b>(index.x() ,index.y()) = cv::Vec3b(0, 0, 255);
                        // }
                        
                    }
                }
            }
            // cv::imshow("debug window", debug_image);
            // cv::waitKey(0);
            // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/publish_feasible_region/debug/debug_image.png", debug_image);
        }
    }
    if (!GetResult(end_P)) 
    {
        std::cout << "get res fail" << std::endl;
        return false;
    }
    return true;
}

bool hybridAstar::GetResult(NodePtr n)
{
    // LOG(INFO)<<"ENTER RESULT";
    auto iter_P = n;
    while (iter_P)
    {
        grid_map::Position p2(iter_P->x, iter_P->y);
        grid_map::Index index;
        map_.getIndex(p2, index);
        // LOG(INFO)<<"current index: "<<index.transpose();
        // sleep(1);
        path.emplace_back(Eigen::Vector3d(iter_P->x, iter_P->y, iter_P->yaw));
        if (iter_P == start_P)
        {
            std::reverse(path.begin(), path.end());
            return true;
        }
        // current_node_P = current_node_P->prenode;
        iter_P = iter_P->prenode;
    }
    return false;
}


hybridAstar::~hybridAstar()
{
}