#include <iostream>
#include <queue>
#include <set>
#include <vector>
#include <Eigen/Core>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <opencv2/opencv.hpp>
namespace footstep{

struct foot
{
    double footx, footy, footz, foottheta, footroll, footpitch; // 落脚点的 x,y,z,theta;
    int foottag; // 左脚 foottag = 1 , 右脚 foottag = -1;
    // std::shared_ptr<foot> prenode = nullptr;
    foot()
    {
        footx = NAN;
        footy = NAN;
        footz = NAN;
        foottheta = NAN;
        foottag = NAN;
    }
};

// typedef std::shared_ptr<foot> FootPtr;
class footStepPlan
{
private:
    std::vector<Eigen::Vector3d> capturepath; // x，y, z 或 x,y,theta
    grid_map::GridMap feasibleMap_;
    std::vector<Eigen::Vector3d> path_;
    /* 硬件参数  */
    double dmaxstep; //最大步长
    double dmaxstepthetaInside; // 最大向内侧方向转动角度
    double dmaxstepthetaOutside; // 最大向内侧方向转动角度
    double dstepwidth; // 步宽 党到脚中心0.1
    std::pair<double, double> detalegth; // 计算捕获点时默认增长步长最小和最大值  注意对应关系
    std::vector<foot> footlist;

    cv::Mat debug_image;
public:
    
    
    void setBHRhard(const double _cdmaxstep,const double _cdmaxstepthetaInside,const double _dmaxstepthetaOutside,const double _dstepwidth,double _detalegthH,double _detalegthL);

    footStepPlan(grid_map::GridMap & feasiblemap,std::vector<Eigen::Vector3d> path);
    ~footStepPlan();
    bool CapturePathPlanB();
    bool CapturePathPlanC();
    bool CapturePathPlan();
    bool isUp(grid_map::Index & index_p);
    bool cpTofoot(Eigen::Vector3d & p, Eigen::Vector2d & dd, foot & f);
    double computeAngle(std::vector<Eigen::Vector3d> points);
    // foot cpTofoot(double dlx, double dly, double dltheta, double dllx, double dlly);
    bool computefootStep();

    
};

}
