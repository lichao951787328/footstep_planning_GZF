#include <footstep_planning_GZF/footstep_planner.h>
#include <Eigen/Geometry>
#include <grid_map_core/iterators/SpiralIterator.hpp>
#include <glog/logging.h>
#define DEBUG;
namespace footstep
{
/*
function: 设置机器人硬件参数
input   : 最大步长，最大向内侧方向转动角度，最大向内侧方向转动角度，步宽
output  : 无
*/
// 还是给局部坐标系下的path更合适
footStepPlan::footStepPlan(grid_map::GridMap & feasiblemap,std::vector<Eigen::Vector3d> path)
{
    feasibleMap_ = feasiblemap;
    path_ = path;

#ifdef DEBUG
    debug_image = cv::Mat::zeros(feasibleMap_.getSize().x(), feasibleMap_.getSize().y(), CV_8UC3);
    for (int i = 0; i < feasibleMap_.getSize().x(); i++)
    {
        for (int j = 0; j < feasibleMap_.getSize().y(); j++)
        {
            grid_map::Position3 p3;
            if (feasibleMap_.getPosition3("z", grid_map::Index(i, j), p3))
            {
                if (!std::isnan(p3.z()))
                {
                    debug_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                }
            }

        }
    }
    
    for (auto & path_p : path_)
    {
        grid_map::Index index;
        if (feasibleMap_.getIndex(grid_map::Position(path_p.x(), path_p.y()), index))
        {
            debug_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(255, 0, 0);
        }
    }
    double scale = 4;
    cv::namedWindow("debug window", cv::WINDOW_NORMAL);
    cv::resizeWindow("debug window", static_cast<int>(debug_image.cols * scale), static_cast<int>(debug_image.rows * scale));
    cv::imshow("debug window", debug_image);
    cv::waitKey(0);
#endif
}

void footStepPlan::setBHRhard(const double _cdmaxstep,const double _cdmaxstepthetaInside,const double _dmaxstepthetaOutside,const double _dstepwidth,double _detalegthH,double _detalegthL)
{
    dmaxstep = _cdmaxstep;
    dmaxstepthetaInside = _cdmaxstepthetaInside;
    dmaxstepthetaOutside = _dmaxstepthetaOutside;
    dstepwidth = _dstepwidth;
    detalegth.first = _detalegthH;
    detalegth.second = _detalegthL;
}


footStepPlan::~footStepPlan()
{
    feasibleMap_.clearAll();
    capturepath.clear();
    path_.clear();
};

/*
function: 从路径上选择捕获点
input   : path 和 feasibleMap
output  : 
*/
bool footStepPlan::CapturePathPlan()
{
    //1. 如果是起点，捕获点由小逐渐增大到最大容许步长，但是如果指针对上斜坡和上台阶，则不要增加到最大
    // 从起点到终点遍历路径
    grid_map::Position3 p3_last;
    capturepath.emplace_back(*(path_.begin())); // 将路径的第一个点放入捕获点第一个点
    grid_map::Index start_index;
    if (feasibleMap_.getIndex(grid_map::Position(path_.begin()->x(), path_.begin()->y()), start_index))
    {
        if (feasibleMap_.getPosition3("z", start_index, p3_last))
        {
            // 需要对起点进行单独处理
            if (std::isnan(p3_last.z()))
            {
                LOG(INFO)<<"some error ocurr";
            }
        }
    }
    
    int steptag = 0; // 上台阶标志，不用管
    double resolution = feasibleMap_.getResolution();
    int max_move = (int)(detalegth.first/resolution);
    int min_move = (int)(detalegth.second/resolution);
    int mid_move = (max_move + min_move)/2;

#ifdef DEBUG
    LOG(INFO)<<"start index: "<<start_index.transpose();
    LOG(INFO)<<"max_move: "<<max_move;
    LOG(INFO)<<"min_move: "<<min_move;
    LOG(INFO)<<"mid_move: "<<mid_move;
#endif
    bool second_half = true;
    // 这样会不会原地多走一步
    std::vector<Eigen::Vector3d>::iterator p_iter = path_.begin(); 
    capturepath.emplace_back(*p_iter);
    // 保证起点在地图的有效区域内
    bool is_first = true;
    while (p_iter < path_.end())
    {
        LOG(INFO)<<(p_iter - path_.begin());
        // 如果到终点的距离小于mid_move
        if (p_iter + mid_move >= path_.end())
        {
            capturepath.emplace_back(*(path_.end() - 1));
            break;
        }
        // 这里的path点全部都是在地图上的，不是在可通行区域上
        // 区域分段聚类
        // x y z yaw
        std::vector<std::vector<std::pair<Eigen::Vector4d, int>>> clusters;
        std::vector<std::pair<Eigen::Vector4d, int>> cluster;
        cluster.clear();
        for (int i = mid_move; i <= max_move; i++)
        {
            LOG(INFO)<<"i = "<<i;
            if (p_iter + i < path_.end())
            {
                grid_map::Position position((p_iter + i)->x(), (p_iter + i)->y());
                grid_map::Index index;
                if (feasibleMap_.getIndex(position, index))
                {
                    grid_map::Position3 position3;
                    if (feasibleMap_.getPosition3("z", index, position3))
                    {
                        if (!std::isnan(position3.z()))
                        {
                            Eigen::Vector4d tmp4;
                            tmp4.head(3) = position3;
                            tmp4(3) = (p_iter + i)->z();
                            LOG(INFO)<<"emplace i "<<i;
                            cluster.emplace_back(std::make_pair(tmp4, i));
                        }
                        else
                        {
                            LOG(INFO)<<"is nan";
                            if (!cluster.empty())
                            {
                                clusters.emplace_back(cluster);
                                cluster.clear(); 
                            }
                        }
                    }
                    else
                    {
                        LOG(INFO)<<"get z error";
                    }
                    
                }
                else
                {
                    LOG(INFO)<<"get index error";
                }
                
            }
            else
            {
                LOG(INFO)<<"not in path";
            }
            
            
        }

        if (!cluster.empty())
        {
            clusters.emplace_back(cluster);
        }
        second_half = false;
        if (clusters.empty())// 证明按这种方式没有找到合适的段，需要按照最近到中间再进行一次遍历
        {
            
            for (int i = min_move; i < mid_move; i++)
            {
                if (p_iter + i < path_.end())
                {
                    grid_map::Position position((p_iter + i)->x(), (p_iter + i)->y());
                    grid_map::Index index;
                    if (feasibleMap_.getIndex(position, index))
                    {
                        grid_map::Position3 position3;
                        if (feasibleMap_.getPosition3("z", index, position3))
                        {
                            if (!std::isnan(position3.z()))
                            {
                                Eigen::Vector4d tmp4;
                                tmp4.head(3) = position3;
                                tmp4(3) = (p_iter + i)->z();
                                LOG(INFO)<<"emplace i "<<i;
                                cluster.emplace_back(std::make_pair(tmp4, i));
                            }
                            else
                            {
                                LOG(INFO)<<"is nan";
                                if (!cluster.empty())
                                {
                                    clusters.emplace_back(cluster);
                                    cluster.clear(); 
                                }
                            }
                        }
                        else
                        {
                            LOG(INFO)<<"get z error";
                        }
                        
                    }
                    else
                    {
                        LOG(INFO)<<"get index error";
                    }
                    
                }
                else
                {
                    LOG(INFO)<<"not in path";
                }
            }
        }

#ifdef DEBUG
        LOG(INFO)<<"clusters size is: "<<clusters.size();
        if (!clusters.empty())
        {
            LOG(INFO)<<"first cluster size is: "<<clusters.at(0).size();
            LOG(INFO)<<"last cluster size is: "<<clusters.at(clusters.size() - 1).size();
        }
        else
        {
            LOG(INFO)<<"cluster is empty";
            break;
        }
        
        
        cv::Mat tmp_image = debug_image.clone();
        grid_map::Index iter_index;
        if (feasibleMap_.getIndex(grid_map::Position(p_iter->x(), p_iter->y()), iter_index))
        {
            tmp_image.at<cv::Vec3b>(iter_index.x(), iter_index.y()) = cv::Vec3b(0, 0, 255);
        }
        for (auto & c : clusters)
        {
            for (auto & p : c)
            {
                grid_map::Index index;
                if (feasibleMap_.getIndex(grid_map::Position(p.first.x(), p.first.y()), index))
                {
                    tmp_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 255, 255);
                }
            }
        }
        double scale = 4;
        cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
        cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
        cv::imshow("tmp window", tmp_image);
        cv::waitKey(0);
#endif
        // 只将第一个聚类的最后一个数作为判断
        if (second_half)
        {
            if (abs(clusters.at(0).begin()->first.z() - p3_last.z()) > 0.08)
            {
                if (clusters.at(0).size() >= 3)
                {
                    Eigen::Vector3d tmp3((clusters.at(0).begin() + 1)->first.x(), (clusters.at(0).begin() + 1)->first.y(), (clusters.at(0).begin() + 1)->first.w());
                    capturepath.emplace_back(tmp3);
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + (clusters.at(0).begin() + 1)->second;

#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
                else
                {
                    Eigen::Vector3d tmp3((clusters.at(0).begin())->first.x(), (clusters.at(0).begin())->first.y(), (clusters.at(0).begin())->first.w());
                    capturepath.emplace_back(tmp3);
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + (clusters.at(0).begin())->second;
#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
            }
            else
            {
                if (clusters.at(0).size() >= 3)
                {
                    Eigen::Vector3d tmp3((clusters.at(0).begin() + 1)->first.x(), (clusters.at(0).begin() + 1)->first.y(), (clusters.at(0).begin() + 1)->first.w());
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + (clusters.at(0).begin() + 1)->second;
#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
                else
                {
                    Eigen::Vector3d tmp3((clusters.at(0).begin())->first.x(), (clusters.at(0).begin())->first.y(), (clusters.at(0).begin())->first.w());
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + (clusters.at(0).begin())->second;
#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
            }

        }
        else
        {
            std::vector<std::pair<Eigen::Vector4d, int>> last_cluster = *(clusters.end() - 1);
            if (abs((last_cluster.end() - 1)->first.z() - p3_last.z()) > 0.08)
            {
                if (last_cluster.size() >= 3)
                {
                    Eigen::Vector3d tmp3(last_cluster.at(last_cluster.size() -2).first.x(), last_cluster.at(last_cluster.size() -2).first.y(), last_cluster.at(last_cluster.size() -2).first.w());
                    capturepath.emplace_back(tmp3);
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + last_cluster.at(last_cluster.size() -2).second;
#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
                else
                {
                    Eigen::Vector3d tmp3(last_cluster.at(last_cluster.size() - 1).first.x(), last_cluster.at(last_cluster.size() - 1).first.y(), last_cluster.at(last_cluster.size() - 1).first.w());
                    capturepath.emplace_back(tmp3);
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + last_cluster.at(last_cluster.size() - 1).second;
#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
            }
            else
            {
                if (last_cluster.size() >= 3)
                {
                    Eigen::Vector3d tmp3(last_cluster.at(last_cluster.size() -2).first.x(), last_cluster.at(last_cluster.size() -2).first.y(), last_cluster.at(last_cluster.size() -2).first.w());
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + last_cluster.at(last_cluster.size() -2).second;
#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
                else
                {
                    Eigen::Vector3d tmp3(last_cluster.at(last_cluster.size() - 1).first.x(), last_cluster.at(last_cluster.size() - 1).first.y(), last_cluster.at(last_cluster.size() - 1).first.w());
                    capturepath.emplace_back(tmp3);
                    p_iter = p_iter + last_cluster.at(last_cluster.size() - 1).second;
#ifdef DEBUG
                    grid_map::Index add_index;
                    if (feasibleMap_.getIndex(grid_map::Position(tmp3.x(), tmp3.y()), add_index))
                    {
                        tmp_image.at<cv::Vec3b>(add_index.x(), add_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                    double scale = 4;
                    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
                    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
                    cv::imshow("tmp window", tmp_image);
                    cv::waitKey(0);
#endif
                }
            }


        }
        
        
        
    }
        

    //     grid_map::Index index;
    //     if (feasibleMap_.getIndex(grid_map::Position(p_iter->x(), p_iter->y()), index))
    //     {
    //         grid_map::Position3 point_3;
    //         if (feasibleMap_.getPosition3("z", index, point_3))
    //         {
    //             if (!std::isnan(point_3.z()))
    //             {
    //                 if (is_first)// 第一次把初始点加入
    //                 {
    //                     capturepath.emplace_back(*p_iter);
    //                     p3_last = point_3;
    //                     // 移动中间步长，如果发现超出，则补齐一步后停止
    //                     if (p_iter + mid_move < path_.end())
    //                     {
    //                         p_iter = p_iter + mid_move;
    //                     }
    //                     else
    //                     {
    //                         capturepath.emplace_back(*(path_.end() - 1));
    //                         break;
    //                     }
    //                 }
    //                 else
    //                 {
    //                     if (!isUp(index))// 位于斜坡
    //                     {
    //                         capturepath.emplace_back(*p_iter);
    //                         if (p_iter + mid_move < path_.end())
    //                         {
    //                             p_iter = p_iter + mid_move;
    //                         }
    //                         else
    //                         {
    //                             capturepath.emplace_back(*(path_.end() - 1));
    //                             break;
    //                         }
    //                     }   
    //                     else
    //                     {
    //                         if (std::abs(point_3.z() - p3_last.z()) > 0.08)
    //                         {
    //                             if (steptag == 0)
    //                             {
    //                                 grid_map::Position3 tmp_p3 = point_3;
    //                                 while (std::abs(tmp_p3.z() - p3_last.z()) > 0.08)
    //                                 {
    //                                     p_iter --;
    //                                     grid_map::Index tmp_index;
    //                                     if (feasibleMap_.getIndex(grid_map::Position(p_iter->x(), p_iter->y()), tmp_index))
    //                                     {
    //                                         if (feasibleMap_.getPosition3("z", tmp_index, tmp_p3))
    //                                         {
    //                                             if (std::isnan(tmp_p3.z()))
    //                                             {
    //                                                 LOG(INFO)<<"some error ocurr";
    //                                             }
    //                                         }
    //                                         else
    //                                         {
    //                                             LOG(INFO)<<"some error ocurr";
    //                                         }
    //                                     }
    //                                     else
    //                                     {
    //                                         LOG(INFO)<<"some error ocurr";
    //                                     }
    //                                 }
    //                                 // 并步同时加入2个
    //                                 capturepath.emplace_back(*p_iter);
    //                                 capturepath.emplace_back(*p_iter);
    //                                 p_iter = p_iter + max_move;
    //                                 steptag = 1;  
    //                             }
    //                         else
    //                         {
    //                             capturepath.emplace_back(*p_iter);
    //                             p_iter = p_iter + max_move;
    //                         }
    //                     }                  
    //                 }
    //             }
    //         }
    //     }
    // }
    
}

// 一定要保证起点在feasibleMap_内， 考虑角度
bool footStepPlan::CapturePathPlanC()
{
    std::vector<Eigen::Vector3d> cluster;
    std::vector<std::vector<Eigen::Vector3d>> clusters;
    cluster.clear();
    for (int i = 0; i < path_.size(); i++)
    {
        LOG(INFO)<<"I: "<<i;
        grid_map::Index index;
        if (feasibleMap_.getIndex(grid_map::Position(path_.at(i).x(), path_.at(i).y()), index))
        {
            grid_map::Position3 p3;
            if (feasibleMap_.getPosition3("z", index, p3)) // 感觉如果为nan则会返回false
            {
                LOG(INFO)<<p3.transpose();
                if (!std::isnan(p3.z()))
                {
                    cluster.emplace_back(path_.at(i));
                    LOG(INFO)<<path_.at(i).transpose();
                }
                // else
                // {
                //     LOG(INFO)<<"IS NAN";
                //     if (!cluster.empty())
                //     {
                //         LOG(INFO)<<"cluster size is "<<cluster.size();
                //         LOG(INFO)<<"clusters size is "<<clusters.size();
                //         clusters.emplace_back(cluster);
                //         cluster.clear();
                //     }
                // }
            }
            else
            {
                if (!cluster.empty())
                {
                    LOG(INFO)<<"cluster size is "<<cluster.size();
                    LOG(INFO)<<"clusters size is "<<clusters.size();
                    clusters.emplace_back(cluster);
                    cluster.clear();
                }
            }
        }
    }
    if (!cluster.empty())
    {
        clusters.emplace_back(cluster);
    }
    LOG(INFO)<<"clusters size is "<<clusters.size();
#ifdef DEBUG
    cv::Mat tmp_image = debug_image.clone();
    for (auto & cc : clusters)
    {
        for (auto & c : cc)
        {
            grid_map::Index index;
            if (feasibleMap_.getIndex(grid_map::Position(c.x(), c.y()), index))
            {
                tmp_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
            }
        }
    }
    double scale = 4;
    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
    cv::imshow("tmp window", tmp_image);
    cv::waitKey(0);
#endif

    // for (auto & p : path_)
    // {
    //     grid_map::Index index;
    //     if (feasibleMap_.getIndex(grid_map::Position(p.x(), p.y()), index))
    //     {
    //         grid_map::Position3 p3;
    //         if (feasibleMap_.getPosition3("z", index, p3))
    //         {
    //             if (!std::isnan(p3.z()))
    //             {
    //                 cluster.emplace_back(p);
    //             }
    //             else
    //             {
    //                 if (!cluster.empty())
    //                 {
    //                     clusters.emplace_back(cluster);
    //                     cluster.clear();
    //                 }
    //             }
    //         }
    //     }
    // }
    
    // 由于在规划全局轨迹时没有考虑实际落脚点是否会超出可通行区域，所以在进行落脚点分配时需要一个小范围或者角度的变换
    double resolution = feasibleMap_.getResolution();
    for (auto & clu : clusters)
    {
        double length = clu.size() * resolution;
        double angle_sum = computeAngle(clu);
        if (angle_sum < 4/57.3)// 转角很小，不考虑转角对步长分割的影响
        {
            LOG(INFO)<<"length is "<<length;
            if (length >= 0.07)
            {
                capturepath.emplace_back(clu.at(0));
                capturepath.emplace_back(clu.at(0));
                if (length > 0.25)
                {
                    int num = (length/0.25) + 1;
                    LOG(INFO)<<"num: "<<num;
                    double step_length = length/num;
                    LOG(INFO)<<"step_length: "<<step_length;
                    for (int i = 1; i < num; i++)
                    {
                        double l = i * step_length;
                        LOG(INFO)<<"l: "<<l;
                        int index_add = l / resolution;
                        LOG(INFO)<<"index_add: "<<index_add;
                        capturepath.emplace_back(clu.at(index_add));
                    }
                }
                capturepath.emplace_back(*(clu.end() - 1));
                capturepath.emplace_back(*(clu.end() - 1));
            }
            else
            {
                int index = clu.size()/2;
                capturepath.emplace_back(clu.at(index));
                capturepath.emplace_back(clu.at(index));
            }
        }
        else
        {
            if (length >= 0.07 && length <= 0.25)// 只用考虑角度
            {
                // int step_num = angle_sum /(4/57.3) + 1;// 没转一步的角度
                // double step_angle = angle_sum/step_num;
                // if (step_num <= 4)
                // {
                //     Eigen::Vector3d p1 = clu.at(0);
                //     Eigen::Vector3d p2 = clu.at(0);
                //     p2.z() = p1.z() + step_angle;
                //     Eigen::Vector3d p3 = clu.at(clu.size() - 1);
                //     p3.z() = p2.z() + step_angle;
                //     Eigen::Vector3d p4 = clu.at(clu.size() - 1);
                //     p4.z() = p3.z() + step_angle;
                //     capturepath.emplace_back(p1);
                //     capturepath.emplace_back(p2);
                //     capturepath.emplace_back(p3);
                //     capturepath.emplace_back(p4);
                // }
                // else
                // {
                //     // for (int i = 0; i < step_num + 1; i++)   
                //     // {
                //     //     int index = clu.size()/2;
                //     //     Eigen::Vector3d p = clu.at(index);
                //     //     p.z() = i * step_angle + clu.at(0).z();
                //     //     capturepath.emplace_back(p);
                //     // }
                // }

                capturepath.emplace_back(clu.at(0));
                capturepath.emplace_back(clu.at(0));
                double add_angle = 0;
                for (int i = 1; i < clu.size(); i++)
                {
                    add_angle += abs(clu.at(i - 1).z() - clu.at(i).z());
                    if (add_angle >= 3/57.3)// 超过3度就新加一个步态点
                    {
                        capturepath.emplace_back(clu.at(i));
                        add_angle = 0;
                    }
                }
                if (add_angle != 0)// 证明最后还有一些小角度没转，而且要保证转后并脚
                {
                    capturepath.emplace_back(clu.at(clu.size() - 1));
                    capturepath.emplace_back(clu.at(clu.size() - 1));
                }
                else//证明没有剩余的角度要转，则并上最后一步
                {
                    capturepath.emplace_back(clu.at(clu.size() - 1));
                }
                
            }
            else if (length > 0.25)// 角度和长度都需要考虑
            {
                capturepath.emplace_back(clu.at(0));
                capturepath.emplace_back(clu.at(0));
                double add_angle = 0.0;
                double add_dis = 0.0;
                for (int i = 1; i < clu.size(); i++)
                {
                    add_angle += abs(clu.at(i - 1).z() - clu.at(i).z());
                    Eigen::Vector2d v_t(clu.at(i-1).head(2) - clu.at(i).head(2));
                    add_dis += v_t.norm();
                    if (add_angle > 3/57.3 || add_dis > 0.25)
                    {
                        capturepath.emplace_back(clu.at(i));
                        add_angle = 0;
                        add_dis = 0;
                    }
                }
                if (add_angle != 0 || add_dis != 0)
                {
                    capturepath.emplace_back(clu.at(clu.size() - 1));
                    capturepath.emplace_back(clu.at(clu.size() - 1));
                }
                else
                {
                    capturepath.emplace_back(clu.at(clu.size() - 1));
                }
            }
            else// 只用考虑角度
            {
                // int step_num = angle_sum /(4/57.3) + 1;// 没转一步的角度
                // double step_angle = angle_sum/step_num;
                // for (int i = 0; i < step_num + 1; i++)   
                // {
                //     int index = clu.size()/2;
                //     Eigen::Vector3d p = clu.at(index);
                //     p.z() = i * step_angle + clu.at(0).z();
                //     capturepath.emplace_back(p);
                // }
                capturepath.emplace_back(clu.at(0));
                capturepath.emplace_back(clu.at(0));
                double add_angle = 0;
                for (int i = 1; i < clu.size(); i++)
                {
                    add_angle += abs(clu.at(i - 1).z() - clu.at(i).z());
                    if (add_angle >= 3/57.3)// 超过3度就新加一个步态点
                    {
                        capturepath.emplace_back(clu.at(i));
                        add_angle = 0;
                    }
                }
                if (add_angle != 0)
                {
                    capturepath.emplace_back(clu.at(clu.size() - 1));
                }
            }
        }
    }

#ifdef DEBUG
    for (int i = 0; i < capturepath.size(); i++)
    {
        LOG(INFO)<<i<<": "<<capturepath.at(i).head(2).transpose()<<" "<<capturepath.at(i).z() * 57.3;
        grid_map::Index index;
        if (feasibleMap_.getIndex(grid_map::Position(capturepath.at(i).x(), capturepath.at(i).y()), index))
        {
            debug_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
        }
    }
    // doubscale = 4;
    cv::namedWindow("debug window", cv::WINDOW_NORMAL);
    cv::resizeWindow("debug window", static_cast<int>(debug_image.cols * scale), static_cast<int>(debug_image.rows * scale));
    cv::imshow("debug window", debug_image);
    cv::waitKey(0);
#endif
}

double footStepPlan::computeAngle(std::vector<Eigen::Vector3d> points)
{
    double sum = 0.0;
    for (int i = 0; i < points.size() - 1; i++)
    {
        sum += abs(points.at(i).z() - points.at(i + 1).z());
    }
    return sum;
}

// 不考虑角度
bool footStepPlan::CapturePathPlanB()
{
    std::vector<Eigen::Vector3d> cluster;
    std::vector<std::vector<Eigen::Vector3d>> clusters;
    cluster.clear();
    for (int i = 0; i < path_.size(); i++)
    {
        LOG(INFO)<<"I: "<<i;
        grid_map::Index index;
        if (feasibleMap_.getIndex(grid_map::Position(path_.at(i).x(), path_.at(i).y()), index))
        {
            grid_map::Position3 p3;
            if (feasibleMap_.getPosition3("z", index, p3)) // 感觉如果为nan则会返回false
            {
                LOG(INFO)<<p3.transpose();
                if (!std::isnan(p3.z()))
                {
                    cluster.emplace_back(path_.at(i));
                    LOG(INFO)<<path_.at(i).transpose();
                }
                // else
                // {
                //     LOG(INFO)<<"IS NAN";
                //     if (!cluster.empty())
                //     {
                //         LOG(INFO)<<"cluster size is "<<cluster.size();
                //         LOG(INFO)<<"clusters size is "<<clusters.size();
                //         clusters.emplace_back(cluster);
                //         cluster.clear();
                //     }
                // }
            }
            else
            {
                if (!cluster.empty())
                {
                    LOG(INFO)<<"cluster size is "<<cluster.size();
                    LOG(INFO)<<"clusters size is "<<clusters.size();
                    clusters.emplace_back(cluster);
                    cluster.clear();
                }
            }
        }
    }
    if (!cluster.empty())
    {
        clusters.emplace_back(cluster);
    }
    LOG(INFO)<<"clusters size is "<<clusters.size();
#ifdef DEBUG
    cv::Mat tmp_image = debug_image.clone();
    for (auto & cc : clusters)
    {
        for (auto & c : cc)
        {
            grid_map::Index index;
            if (feasibleMap_.getIndex(grid_map::Position(c.x(), c.y()), index))
            {
                tmp_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
            }
        }
        
    }
    double scale = 4;
    cv::namedWindow("tmp window", cv::WINDOW_NORMAL);
    cv::resizeWindow("tmp window", static_cast<int>(tmp_image.cols * scale), static_cast<int>(tmp_image.rows * scale));
    cv::imshow("tmp window", tmp_image);
    cv::waitKey(0);
#endif

    // for (auto & p : path_)
    // {
    //     grid_map::Index index;
    //     if (feasibleMap_.getIndex(grid_map::Position(p.x(), p.y()), index))
    //     {
    //         grid_map::Position3 p3;
    //         if (feasibleMap_.getPosition3("z", index, p3))
    //         {
    //             if (!std::isnan(p3.z()))
    //             {
    //                 cluster.emplace_back(p);
    //             }
    //             else
    //             {
    //                 if (!cluster.empty())
    //                 {
    //                     clusters.emplace_back(cluster);
    //                     cluster.clear();
    //                 }
    //             }
    //         }
    //     }
    // }
    
    double resolution = feasibleMap_.getResolution();
    for (auto & clu : clusters)
    {
        double length = clu.size() * resolution;
        LOG(INFO)<<"length is "<<length;
        if (length >= 0.07)
        {
            capturepath.emplace_back(clu.at(0));
            capturepath.emplace_back(clu.at(0));
            if (length > 0.25)
            {
                int num = (length/0.25) + 1;
                LOG(INFO)<<"num: "<<num;
                double step_length = length/num;
                LOG(INFO)<<"step_length: "<<step_length;
                for (int i = 1; i < num; i++)
                {
                    double l = i * step_length;
                    LOG(INFO)<<"l: "<<l;
                    int index_add = l / resolution;
                    LOG(INFO)<<"index_add: "<<index_add;
                    capturepath.emplace_back(clu.at(index_add));
                }
            }
            capturepath.emplace_back(*(clu.end() - 1));
            capturepath.emplace_back(*(clu.end() - 1));
        }
        else
        {
            int index = clu.size()/2;
            capturepath.emplace_back(clu.at(index));
            capturepath.emplace_back(clu.at(index));
        }
    }

#ifdef DEBUG
    for (int i = 0; i < capturepath.size(); i++)
    {
        grid_map::Index index;
        if (feasibleMap_.getIndex(grid_map::Position(capturepath.at(i).x(), capturepath.at(i).y()), index))
        {
            debug_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
        }
    }
    // doubscale = 4;
    cv::namedWindow("debug window", cv::WINDOW_NORMAL);
    cv::resizeWindow("debug window", static_cast<int>(debug_image.cols * scale), static_cast<int>(debug_image.rows * scale));
    cv::imshow("debug window", debug_image);
    cv::waitKey(0);
#endif
}

// 如何根据机器人当前脚状态继续进行规划
/**
 * @1 获取开始时的capturePoint位置
 * 
 * @return true 
 * @return false 
 */

bool footStepPlan::computefootStep()
{
    foot foot_;
    int leftOrright = - 1; // -1 left, 1 right
    for (auto & point : capturepath)
    {
        foot ff;
        Eigen::Vector2d tmp(0, leftOrright * dstepwidth);
        if (cpTofoot(point, tmp, ff))
        {
            ff.foottag = leftOrright;
            footlist.emplace_back(ff);
            leftOrright = -leftOrright;
        }
        else
        {
            LOG(INFO)<<"point info: "<<point.head(2).transpose()<<" "<<point.z() * 57.3;
            LOG(INFO)<<leftOrright;
            return false;
        }
        
    }
    return true;
}

bool footStepPlan::isUp(grid_map::Index & index_p)
{
    grid_map::Position3 p3_roll, p3_pitch;
    if (feasibleMap_.getPosition3("roll", index_p, p3_roll) && feasibleMap_.getPosition3("pitch", index_p, p3_pitch))
    {
        if (!std::isnan(p3_roll.z()) && !std::isnan(p3_pitch.z()))
        {
            if (std::abs(p3_roll.z()) < 0.08 || std::abs(p3_pitch.z()) < 0.08)
            {
                return true;
            }
        }
    }
    return false;
}

bool footStepPlan::cpTofoot(Eigen::Vector3d & p, Eigen::Vector2d & dd, foot & f)
{
    Eigen::AngleAxisd ad(p.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d v_t(dd.x(), dd.y(), 0);
    Eigen::Vector3d point;
    point.head(2) = p.head(2);
    point.z() = 0;
    Eigen::Vector3d after = ad.toRotationMatrix() * v_t + point;

    // 根据可通行区域选取
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int size = 0;
    for (grid_map::SpiralIterator iter(feasibleMap_, Eigen::Vector2d(after.x(), after.y()), 0.04); !iter.isPastEnd(); ++iter)
    {
        grid_map::Index index(*iter);
        grid_map::Position3 p3;
        if (feasibleMap_.getPosition3("z", index, p3))
        {
            if (!std::isnan(p3.z()))
            {
                sum += p3;
                size ++;
            }
        }
    }
    if (sum.isZero())
    {
        LOG(INFO)<<"sum is zero";
        return false;
    }
    Eigen::Vector3d center = sum/size;
    grid_map::Index center_index;
    if (feasibleMap_.getIndex(center.head(2), center_index))
    {
        grid_map::Position3 center_position, center_start, center_end;
        if (!feasibleMap_.getPosition3("z", center_index, center_position))
        {
            LOG(INFO)<<"no Z";
            return false;
        }
        if (!feasibleMap_.getPosition3("start_angle", center_index, center_start))
        {
            LOG(INFO)<<"no start_angle";
            return false;
        }
        if (!feasibleMap_.getPosition3("end_angle", center_index, center_end))
        {
            LOG(INFO)<<"no end_angle";
            return false;
        }
        if (p.z() >= center_start.z() - 2/57.3 && p.z() <= center_end.z() + 2/57.3)
        {
            f.footpitch = 0;
            f.footroll = 0;
            f.foottheta = p.z();
            f.footx = center_position.x();
            f.footy = center_position.y();
            f.footz = center_position.z();
            return true;
        }
        else
        {
            LOG(INFO)<<"angle out of range";
            LOG(INFO)<<p.z()<<" "<<center_start.z()<<" "<<center_end.z();
            return false;
        }
    }
    return false;
}

}