/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef POTBOT_NAV_STATE_LAYER_H_
#define POTBOT_NAV_STATE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/footprint.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Polygon.h>

#include <potbot_plugin/StatePluginConfig.h>
#include <potbot_msgs/StateArray.h>
#include <potbot_lib/scan_clustering.h>
#include <potbot_lib/unscented_kalman_filter.h>

using namespace costmap_2d;

namespace potbot_nav
{
    class StateLayer : public costmap_2d::Layer
    {
    public:
        StateLayer()
        {
            // costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
        }

        virtual ~StateLayer();
        virtual void onInitialize();
        virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        /**
         * @brief  A callback to handle buffering LaserScan messages
         * @param message The message returned from a message notifier
         */
        void laserScanCallback(const sensor_msgs::LaserScanConstPtr &message);

        const potbot_msgs::ObstacleArray& getClustersObstacleArray() const;

    protected:
        virtual void setupDynamicReconfigure(ros::NodeHandle &nh);

    private:
        std::string global_frame_;

        potbot_msgs::ObstacleArray clusters_obstaclearray_scan_;

        ros::Subscriber sub_scan_;
        ros::Publisher pub_scan_clustering_, pub_state_marker_, pub_obstacles_scan_estimate_, pub_scan_range_;
        std::vector<potbot_lib::UnscentedKalmanFilter> states_ukf_;

        double kappa_ = -2;
        double sigma_q_ = 0.00001;
        double sigma_r_ = 0.00001;
        double sigma_p_ = 1;
        double apply_cluster_to_localmap_ = 100;
        double max_estimated_linear_velocity_ = 100;
        double max_estimated_angular_velocity_ = 100;
        double prediction_time_ = 2;

        pcl::PointCloud<pcl::PointXYZ> scan_cloud_;

        dynamic_reconfigure::Server<potbot_plugin::StatePluginConfig> *dsrv_;

        void reconfigureCB(potbot_plugin::StatePluginConfig &config, uint32_t level);
    };

} // namespace potbot_nav

#endif // POTBOT_NAV_STATE_LAYER_H_
