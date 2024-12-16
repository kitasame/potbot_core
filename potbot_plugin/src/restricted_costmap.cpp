#include <potbot_plugin/restricted_costmap.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


PLUGINLIB_EXPORT_CLASS(RestrictedCostmap, costmap_2d::Layer)


RestrictedCostmap::RestrictedCostmap() : origin_x_(0.0), origin_y_(0.0), width_(4.0), height_(10.0), first_update_(true) {}

void RestrictedCostmap::onInitialize() {
    ros::NodeHandle nh("~" + name_);
    nh.param("width", width_, 4.0);
    nh.param("height", height_, 10.0);

    ROS_INFO("RestrictedCostmap initialized with width: %f, height: %f", width_, height_);
    publishRestrictedAreaMarker();
}

void RestrictedCostmap::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                       double* min_x, double* min_y, double* max_x, double* max_y) {
    if (first_update_) {
        origin_x_ = robot_x;  // 初回のロボット位置を基準にする
        origin_y_ = robot_y;
        first_update_ = false;
        ROS_INFO("Origin set to: (%f, %f)", origin_x_, origin_y_);
    }

    *min_x = origin_x_ - width_ / 2.0;
    *max_x = origin_x_ + width_ / 2.0;
    *min_y = origin_y_ - height_ / 2.0;
    *max_y = origin_y_ + height_ / 2.0;
}

void RestrictedCostmap::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            double wx, wy;
            master_grid.mapToWorld(i, j, wx, wy);

            if (!isWithinRestrictedArea(wx, wy)) {
                master_grid.setCost(i, j, costmap_2d::NO_INFORMATION);
            }
        }
    }
}

bool RestrictedCostmap::isWithinRestrictedArea(double x, double y) {
    return (x >= origin_x_ - width_ / 2.0 && x <= origin_x_ + width_ / 2.0 &&
            y >= origin_y_ - height_ / 2.0 && y <= origin_y_ + height_ / 2.0);
}

// 制限エリアをRvizに表示するメソッドを追加
void RestrictedCostmap::publishRestrictedAreaMarker() {
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("restricted_area_marker", 1, true);

    // Markerの初期化
    visualization_msgs::Marker restricted_area_marker;
    restricted_area_marker.header.frame_id = "map";  // 必要に応じてtfのframe_idを変更
    restricted_area_marker.header.stamp = ros::Time::now();
    restricted_area_marker.ns = "restricted_area";
    restricted_area_marker.id = 0;
    restricted_area_marker.type = visualization_msgs::Marker::LINE_STRIP;
    restricted_area_marker.action = visualization_msgs::Marker::ADD;
    restricted_area_marker.pose.orientation.w = 1.0;  // 無回転

    // Markerの色、スケール
    restricted_area_marker.scale.x = 0.1;  // 線の幅
    restricted_area_marker.color.r = 0.0;  // 緑
    restricted_area_marker.color = potbot_lib::color::get_msg("yellow");

    // 制限エリアの4つの頂点を指定
    geometry_msgs::Point p1, p2, p3, p4;
    double half_width = width_ / 2.0;
    double half_height = height_ / 2.0;

    // 各頂点の座標を設定
    p1.x = origin_x_ - half_width;
    p1.y = origin_y_ - half_height;

    p2.x = origin_x_ + half_width;
    p2.y = origin_y_ - half_height;

    p3.x = origin_x_ + half_width;
    p3.y = origin_y_ + half_height;

    p4.x = origin_x_ - half_width;
    p4.y = origin_y_ + half_height;

    // 頂点を順番に追加して矩形を形成
    restricted_area_marker.points.push_back(p1);
    restricted_area_marker.points.push_back(p2);
    restricted_area_marker.points.push_back(p3);
    restricted_area_marker.points.push_back(p4);
    restricted_area_marker.points.push_back(p1);  // 最初の点に戻る

    // Publisherを用いてMarkerを送信
    marker_pub.publish(restricted_area_marker);
}



