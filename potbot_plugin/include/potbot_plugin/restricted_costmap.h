#ifndef RESTRICTED_COSTMAP_H
#define RESTRICTED_COSTMAP_H

#include <dynamic_reconfigure/server.h>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <potbot_lib/utility.h>
#include <potbot_lib/utility_ros.h>
#include <potbot_lib/field.h>

class RestrictedCostmap : public costmap_2d::CostmapLayer {
public:
    RestrictedCostmap();

    virtual void onInitialize() override;
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,double* min_x, double* min_y, double* max_x, double* max_y) override;
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
    void publishRestrictedAreaMarker();

    ros::Publisher marker_pub;

private:
    double origin_x_, origin_y_;  // ロボット初期位置
    double width_, height_;       // 活動範囲（矩形）
    bool first_update_;           // 初回更新フラグ

    bool isWithinRestrictedArea(double x, double y);
};

#endif // RESTRICTED_COSTMAP_H