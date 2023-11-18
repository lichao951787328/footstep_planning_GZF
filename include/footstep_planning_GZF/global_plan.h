#include <queue>
#include <grid_map_core/GridMap.hpp>
#include <set>
#include <unordered_set>
#include <opencv2/opencv.hpp>
// 选取代价最小的点
struct node
{
    double x, y, z, yaw, cost;
    std::shared_ptr<node> prenode = nullptr;
    node(Eigen::Vector3d & input)
    {
        x = input.x();
        y = input.y();
        yaw = input.z();
        z = NAN;
        cost = NAN;
        prenode = nullptr;
    }
    node()
    {
        x = NAN;
        y = NAN;
        z = NAN;
        yaw = NAN;
        cost = NAN;
        prenode = nullptr;
    }
    node & operator=(const node & other)
    {
        if (this == &other)
        {
            return *this;
        }
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        this->yaw = other.yaw;
        this->cost = other.cost;
        this->prenode = other.prenode;
        return *this;
    }
        
};

typedef std::shared_ptr<node> NodePtr;

struct nodeptrCom
{
    bool operator()(const NodePtr n1, const NodePtr n2)
    {
        if (std::isnan(n1->cost) || std::isnan(n2->cost))
        {
            std::cout<<"the cost is nan"<<std::endl;
        }
        
        return n1->cost > n2->cost;
    }
};

class hybridAstar
{
private:
    NodePtr start_P = nullptr;
    NodePtr goal_P = nullptr;
    NodePtr end_P = nullptr;
    grid_map::GridMap map_;
    std::priority_queue<NodePtr, std::vector<NodePtr>, nodeptrCom> p_queue_;
    std::unordered_set<std::string> close_set_;
    std::vector<double> sample_lengths{0.1};
    std::vector<double> sample_angle{-4/57.3, -2/57.3, 0, 2/57.3, 4/57.3};
    std::vector<Eigen::Vector3d> path;

public:
    hybridAstar(Eigen::Vector3d & start, Eigen::Vector3d & end, grid_map::GridMap & map);
    bool GetHumanoidGuidence(NodePtr n);
    std::string get_node_indexS(NodePtr n);
    std::vector<NodePtr> getNextNodes(NodePtr current_node);
    NodePtr getNodeFrom(NodePtr current_node, double angle, double length);
    double computeCost(NodePtr n);
    bool plan();
    bool GetResult(NodePtr n);
    inline std::vector<Eigen::Vector3d> getPath()
    {
        return path;
    }
    bool isValid(NodePtr n);
    ~hybridAstar();
};
