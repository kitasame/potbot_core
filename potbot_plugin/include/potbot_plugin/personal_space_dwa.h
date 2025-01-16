/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/
#ifndef PERSONAL_SPACE_DWA_H_
#define PERSONAL_SPACE_DWA_H_

#include <vector>
#include <Eigen/Core>


#include <potbot_plugin/PersonalSpaceDWAConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layer.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

/*追加項目*/
#include <geometry_msgs/Point.h>
#include <potbot_lib/utility.h>
#include <potbot_lib/field.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <potbot_lib/scan_clustering.h>
#include <potbot_plugin/state_layer.h>
#include <unordered_map>

#include <nav_msgs/Path.h>
/*ここまで*/

namespace potbot_nav {

/*追加項目 */
  struct Pedestrian {
    double x;
    double y;
    double yaw; 
    double lf;  // 前方パーソナルスペースの長軸
    double ls;  // パーソナルスペースの短軸
/*ここまで*/
};

  /**
   * @class PersonalSpaceDWA
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class PersonalSpaceDWA {
    
    protected:
        std::vector<potbot_lib::Point> obstacles_;


    public:
      /**
       * @brief  Constructor for the planner
       * @param name The name of the planner 
       * @param costmap_ros A pointer to the costmap instance the planner should use
       * @param global_frame the frame id of the tf frame to use
       */
      PersonalSpaceDWA(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
      /**
       * @brief Reconfigures the trajectory planner
       */
      void reconfigure(potbot_plugin::PersonalSpaceDWAConfig &cfg);

      void getObstacle(double x = 0, double y = 0);
      void setObstacle(double x = 0, double y = 0);

      // void updateObstaclesFromClusters(potbot_nav::StateLayer& state_layer);

      double estimatePedestrianOrientationFromClusters(const potbot_msgs::ObstacleArray& clusters_obstaclearray);

      /**
       * @brief  Check if a trajectory is legal for a position/velocity pair
       * @param pos The robot's position
       * @param vel The robot's velocity
       * @param vel_samples The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner::Trajectory findBestPath(
          const geometry_msgs::PoseStamped& global_pose,
          const geometry_msgs::PoseStamped& global_vel,
          geometry_msgs::PoseStamped& drive_velocities);

      /**
       * @brief  Update the cost functions before planning
       * @param  global_pose The robot's current pose
       * @param  new_plan The new global plan
       * @param  footprint_spec The robot's footprint
       *
       * The obstacle cost function gets the footprint.
       * The path and goal cost functions get the global_plan
       * The alignment cost functions get a version of the global plan
       *   that is modified based on the global_pose 
       */
      void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
          const std::vector<geometry_msgs::PoseStamped>& new_plan,
          const std::vector<geometry_msgs::Point>& footprint_spec);

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }

      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * sets new plan and resets state
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      
      PersonalSpaceDWA();
      double calculateTotalPersonalSpaceInvasion(const geometry_msgs::Point& point);

    private:

      costmap_2d::Costmap2D* costmap;
      std::string pedestrian_layer_name_;
      std::vector<Pedestrian> current_pedestrians_;
      std::unordered_map<int, std::pair<double, double>> previous_positions;

      // 新しいメソッド
      // void pedestrianPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
      void updatePedestrianPositions();
      bool isPedestrianCell(unsigned int mx, unsigned int my) const;

      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;
      
      // Subscribers
      ros::Subscriber robot_pose_sub_;
      ros::Subscriber centroid_sub_;
      double centroid_x_;
      double centroid_y_;
      bool received_centroid_ ;

      void centroidCallback(const geometry_msgs::Point::ConstPtr& msg);
      base_local_planner::SimpleTrajectoryGenerator simple_trajectory_generator_;
      
      // Publishers
      ros::Publisher personal_space_pub_;
      ros::Publisher invasion_pub_;



      geometry_msgs::Point robot_pos_;
      std::vector<Pedestrian> pedestrians_;
      
      // Costmap related
      nav_msgs::OccupancyGrid personal_space_grid_;
      
      void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void pedestrianPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void updatePersonalSpaceGrid();

      // 新しく追加するメンバー関数
      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

      double calculatePersonalSpaceCost(const base_local_planner::Trajectory& traj);
      double calculatePersonalSpaceInvasion(const geometry_msgs::Point& point, const Pedestrian& pedestrian);
      double calculateCostFromInvasion(double invasion);

      // double estimatePedestrianOrientationFromClusters(const potbot_msgs::ObstacleArray& clusters_obstaclearray,double x, double y);
      double calculateTotalPersonalSpaceViolation(const geometry_msgs::Point& robot_pos);

      std::vector<Pedestrian> getPedestrianPositions(const costmap_2d::Costmap2D* costmap);
      // 歩行者情報を更新
      void updatePedestrians(const costmap_2d::Costmap2D* costmap);
      // スケールパラメータのゲッター
      double getLfScale() const { return lf_scale_; }
      double getLsScale() const { return ls_scale_; }


      double calculateDWACostWithPersonalSpace(const std::vector<base_local_planner::Trajectory>& all_explored);
      double calculateDWACostWithPersonalSpace(const base_local_planner::Trajectory& traj);
      double calculateDWACostWithPersonalSpace(std::vector<base_local_planner::Trajectory>& trajectories,
            const Eigen::Vector3f& pedestrian_pos,
            const Eigen::Vector3f& robot_pos);

      double lf_scale_;
      double ls_scale_;
      double ps_weight_;
      //ここまでが追加変数

      base_local_planner::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double path_distance_bias_, goal_distance_bias_, occdist_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      base_local_planner::Trajectory result_traj_;

      double forward_point_distance_;

      std::vector<geometry_msgs::PoseStamped> global_plan_;

      boost::mutex configuration_mutex_;
      std::string frame_id_;
      ros::Publisher traj_cloud_pub_;
      bool publish_cost_grid_pc_; ///< @brief Whether or not to build and publish a PointCloud
      bool publish_traj_pc_;

      double cheat_factor_;

      base_local_planner::MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function

      // see constructor body for explanations
      base_local_planner::SimpleTrajectoryGenerator generator_;
      base_local_planner::OscillationCostFunction oscillation_costs_;
      base_local_planner::ObstacleCostFunction obstacle_costs_;
      base_local_planner::MapGridCostFunction path_costs_;
      base_local_planner::MapGridCostFunction goal_costs_;
      base_local_planner::MapGridCostFunction goal_front_costs_;
      base_local_planner::MapGridCostFunction alignment_costs_;
      base_local_planner::TwirlingCostFunction twirling_costs_;

  };
};
#endif
