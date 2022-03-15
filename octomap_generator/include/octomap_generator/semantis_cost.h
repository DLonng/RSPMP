#ifndef SEMANTICS_COST_H_
#define SEMANTICS_COST_H_

#include <octomap/ColorOcTree.h>

namespace semantics_cost {
    // 把语义分割发送的颜色按顺序填入即可，不需要更改 RGB 的顺序
    static const octomap::ColorOcTreeNode::Color kRoadGray(170, 170, 170);      // 灰色地面
    static const octomap::ColorOcTreeNode::Color kLowGrassGreen(0, 255, 0);     // 绿色小草
    static const octomap::ColorOcTreeNode::Color kHighGrassDeepGreen(0, 60, 0); // 深绿色树木
    // 目前只有天空的颜色中 G = 120，如果增加了其他 G = 120 的颜色，要修改 SemanticImageCallback
    static const octomap::ColorOcTreeNode::Color kSkyBlue(0, 120, 255);         // 淡蓝色天空
    static const octomap::ColorOcTreeNode::Color kPeopleRed(60, 20, 220);       // 人
    static const octomap::ColorOcTreeNode::Color kCarBlue(142, 0, 0);           // 车辆
    static const octomap::ColorOcTreeNode::Color kUnknowBlack(0, 0, 0);         // 未知类别黑色
    static const octomap::ColorOcTreeNode::Color kBuildingGray(70, 70, 70);     // 建筑物

    // 1. 可通行区域根据通行难度设置不同的语义代价
    static const unsigned char kRoadCost = 1;
    static const unsigned char kLowGrassCost = 2;
    // 高度比底盘还低的未知类别认为是可通行的
    static const unsigned char kLowUnknowClassCost = 3;


    // 2. 不可通行区域设置同一语义代价 50 ?还是设置不同？
    // 设置的不同：可以在规划的时候区分障碍物类别，但是会导致语义代价函数计算障碍物代价标准不一致
    // 设置的相同：不能区分障碍物，但是语义代价函数对障碍物同等看待
    static const unsigned char kObstacleCost = 254;

    static const unsigned char kHighGrassCost = kObstacleCost;//50
    static const unsigned char kPeopleCost = kObstacleCost;//51;
    static const unsigned char kCarCost = kObstacleCost;//52;
    // 因为误差会把天空的颜色融合到点云中，所以把蓝色的天空点云也当做不可通行的
    static const unsigned char kSkyCost = kObstacleCost;//53;
    static const unsigned char kHighRoadCost = kObstacleCost;//54;
    static const unsigned char kBuildingCost = kObstacleCost;//54;

    // 3. 未知类别语义代价
    static const unsigned char kUnknowClassCost = kObstacleCost;//60;

    static const unsigned char kInflationCost = kObstacleCost - 1;

    // 4. 初始无语义的值，不是代价值
    static const unsigned char kNoSemantic = 255;

    // 用于轨迹语义成本的归一化计算
    static const unsigned char kMinCost = kRoadCost;
    static const unsigned char kMaxCost = kUnknowClassCost;
}

#endif // SEMANTICS_COST_H_
