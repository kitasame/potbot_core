#include <potbot_plugin/personal_space_dwa.h>
#include <base_local_planner/goal_functions.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <potbot_lib/field.h>
#include <unordered_map>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <potbot_plugin/state_layer.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>

namespace potbot_nav {

  void PersonalSpaceDWA::reconfigure(potbot_plugin::PersonalSpaceDWAConfig &config)  //パラメータを設定しているだけ
  {

    // パーソナルスペースのパラメータを追加
        lf_scale_ = config.lf_scale;
        ls_scale_ = config.ls_scale;
        ps_weight_ = config.ps_weight;
    // ここまで

    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    double resolution = planner_util_->getCostmap()->getResolution();
    path_distance_bias_ = resolution * config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(path_distance_bias_);
    alignment_costs_.setScale(path_distance_bias_);

    goal_distance_bias_ = resolution * config.goal_distance_bias;
    goal_costs_.setScale(goal_distance_bias_);
    goal_front_costs_.setScale(goal_distance_bias_);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(occdist_scale_);

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);
 
    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

    twirling_costs_.setScale(config.twirling_scale);

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;
 
    if (vx_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      vx_samp = 1;
      config.vx_samples = vx_samp;
    }
 
    if (vy_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      vy_samp = 1;
      config.vy_samples = vy_samp;
    }
 
    if (vth_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
      vth_samp = 1;
      config.vth_samples = vth_samp;
    }
 
    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
 

  }

  PersonalSpaceDWA::PersonalSpaceDWA(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())
  {
    
    goal_front_costs_.setStopOnFailure( false );
    alignment_costs_.setStopOnFailure( false );

    // 新しいパラメータの初期化
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("lf_scale", lf_scale_);
    private_nh.param("ls_scale", ls_scale_);
    private_nh.param("ps_weight", ps_weight_);
    centroid_sub_ = nh_.subscribe("centroid", 10, &PersonalSpaceDWA::centroidCallback, this);
    invansion_pub_ = private_nh.advertise<std_msgs::Float64>("invansion", 10);
    // ここまで

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05;
    } else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    oscillation_costs_.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);


    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name,
                        planner_util->getGlobalFrame(),
                        [this](int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost){
                          return getCellCosts(cx, cy, path_cost, goal_cost, occ_cost, total_cost);
                        });

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));

    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&twirling_costs_); // optionally prefer trajectories that don't spin

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);
  }

  void PersonalSpaceDWA::centroidCallback(const geometry_msgs::Point::ConstPtr& msg)
  {
    centroid_x_ = msg->x;
    centroid_y_ = msg->y;
    ROS_INFO("Received centroid: (%f, %f)", centroid_x_, centroid_y_);
  }

  // used for visualization only, total_costs are not really total costs
  bool PersonalSpaceDWA::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    double norm_path_cost = path_cost / 254;
    double norm_goal_cost = goal_cost / 254;
    double norm_occ_cost = occ_cost / 254;

    total_cost =
        path_distance_bias_ * path_cost; +
        goal_distance_bias_ * goal_cost; +
        occdist_scale_ * occ_cost;
        // ROS_INFO("total_cost: %f",total_cost);
        // ROS_INFO("norm_path_cost: %f",norm_path_cost);
        // ROS_INFO("norm_goal_cost: %f",norm_goal_cost);
        // ROS_INFO("norm_occ_cost: %f",norm_occ_cost);
    return true;
  }

  bool PersonalSpaceDWA::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    return planner_util_->setPlan(orig_global_plan);
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool PersonalSpaceDWA::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      // ROS_INFO("cost: %f",cost);
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }


  void PersonalSpaceDWA::updatePlanAndLocalCosts(
      const geometry_msgs::PoseStamped& global_pose,
      const std::vector<geometry_msgs::PoseStamped>& new_plan,
      const std::vector<geometry_msgs::Point>& footprint_spec) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }

    obstacle_costs_.setFootprint(footprint_spec);

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);

    // alignment costs
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
      forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
      sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);
    
    // keeping the nose on the path
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
      alignment_costs_.setScale(path_distance_bias_);
      // costs for robot being aligned with path (nose on path, not ju
      alignment_costs_.setTargetPoses(global_plan_);
    } else {
      // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale(0.0);
    }
  }


  /*
   * given the current state of the robot, find a good trajectory
   */
  base_local_planner::Trajectory PersonalSpaceDWA::findBestPath(  //DWAの評価関数かも？ 
      const geometry_msgs::PoseStamped& global_pose,  //世界座標系でのロボットの位置
      const geometry_msgs::PoseStamped& global_vel,
      geometry_msgs::PoseStamped& drive_velocities)  //評価式を最大化する速度、角速度を代入
      
      {

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    Eigen::Vector3f pedestrian_pos(centroid_x_, centroid_y_, 0.0);
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    std::vector<base_local_planner::Trajectory> all_explored;
    bool found_legal = scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
    // ここまで

    // prepare cost functions and generators for this run
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);

    std::vector<Eigen::Vector3f> sample_vector = { pedestrian_pos };
    simple_trajectory_generator_.initialise(
    pedestrian_pos,        // 初期位置
    pedestrian_pos,        // 初期速度
    pedestrian_pos,        // 初期加速度
    &limits,               // 制限
    pedestrian_pos,        // サンプル位置
    sample_vector,         // サンプルベクトル
    false                  // サンプリングフラグ
    );

    result_traj_.cost_ = -1e9;

    // ROS_INFO("result_traj_.cost_: %f",result_traj_.cost_);

    // find best trajectory by sampling and scoring the samples
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

    //result_traj_は何を格納した変数？
    //calculateDWACostWithPersonalSpace()はresult_traj_を受け取るのでいいの？
    //calculateDWACostWithPersonalSpace()でresult_traj_を受け取ったあとのresult_traj_の使い方は適切？
    //障害物(今回は歩行者)とロボットの相対的な位置があれば大丈夫！
    // double custom_score = calculateDWACostWithPersonalSpace(all_explored);

    // ROS_INFO("custom score: %f", custom_score);

  

    if(publish_traj_pc_)
    {
        sensor_msgs::PointCloud2 traj_cloud;
        traj_cloud.header.frame_id = frame_id_;
        traj_cloud.header.stamp = ros::Time::now();

        sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
        cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "theta", 1, sensor_msgs::PointField::FLOAT32,
                                          "cost", 1, sensor_msgs::PointField::FLOAT32);

        unsigned int num_points = 0;
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if (t->cost_<0)
              continue;
            num_points += t->getPointsSize();
        }

        cloud_mod.resize(num_points);
        sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t) //全探査する速度、角速度を事前に定義済み
        {
            if(t->cost_<0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                iter_x[0] = p_x;
                iter_x[1] = p_y;
                iter_x[2] = 0.0;
                iter_x[3] = p_th;
                iter_x[4] = t->cost_;
                ++iter_x;
            }
        }
        traj_cloud_pub_.publish(traj_cloud);
    }

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
      //we'll publish the visualization of the costs to rviz before returning our best trajectory
      map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans);

    // パーソナルスペースを考慮したコスト計算
    double custom_score = calculateDWACostWithPersonalSpace(all_explored, pedestrian_pos, pos);
    result_traj_.cost_ += custom_score;
    // ROS_INFO("result_traj_.cost_: %f",result_traj_.cost_);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.pose = geometry_msgs::Pose();
      // drive_velocities.pose.position.x = 0;
      // drive_velocities.pose.position.y = 0;
      // drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    } else {
      drive_velocities.pose.position.x = result_traj_.xv_;
      drive_velocities.pose.position.y = result_traj_.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, result_traj_.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
    }
    // ROS_INFO("result_traj_ cost: %f", result_traj_.cost_);
    // ROS_INFO("best_score: %f", custom_score);
    return result_traj_;
  }

  double PersonalSpaceDWA::calculateDWACostWithPersonalSpace(
    std::vector<base_local_planner::Trajectory>& trajectories,
    const Eigen::Vector3f& pedestrian_pos,
    const Eigen::Vector3f& robot_pos) 
  {
      double min_ps_cost = std::numeric_limits<double>::max(); // 初期値は非常に小さな値
      double invansion = 0.0;
      for (const auto& traj : trajectories) 
      {

        base_local_planner::Trajectory modified_traj = traj; // コピーを作成

          if (traj.cost_ < 0) continue; // 無効な軌道はスキップ

          // 軌道のすべてのポイントを確認
          for (unsigned int i = 0; i < traj.getPointsSize(); ++i) 
          {
              double p_x, p_y, p_th;
              traj.getPoint(i, p_x, p_y, p_th);

              // 歩行者の位置と未来の軌道の計算
              double dx = pedestrian_pos[0] - p_x;
              double dy = pedestrian_pos[1] - p_y;

              // 現在のロボットと歩行者の相対ベクトルの計算
              // double dx = pedestrian_pos[0] - robot_pos[0];
              // double dy = pedestrian_pos[1] - robot_pos[1];

              double distance = std::sqrt(dx * dx + dy * dy);
              // ROS_INFO("distance: %f",distance);

              // 角度を0-180度範囲に正規化
              double theta = std::atan2(dy, dx) - p_th;

              while (theta > M_PI) theta -= 2 * M_PI;
              while (theta < -M_PI) theta += 2 * M_PI;

              double personal_space_radius;
              

              if (std::abs(theta) <= M_PI / 2) {
                // 歩行者の前方の楕円形パーソナルスペース
                personal_space_radius = std::sqrt(std::pow(dx / (lf_scale_), 2) + std::pow(dy / (ls_scale_), 2));
              } else {
                // 歩行者の後方の円形パーソナルスペース
                personal_space_radius = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) / (ls_scale_ * ls_scale_ );
              }

              // Avoid division by zero
              if (personal_space_radius < 1e-9) continue;

              double current_invansion = personal_space_radius / distance;
              invansion = std::max(invansion, current_invansion); // 最大侵入値を記録
              // ROS_INFO("invansion:%f",invansion);


              // Calculate cost using the exponential function
              // double ps_cost = std::exp(ps_weight_ * (1.0 / personal_space_radius));

              double current_ps_cost = std::exp(ps_weight_ * (1.0 / personal_space_radius));
              min_ps_cost = std::min(min_ps_cost, current_ps_cost); // 最大コストを記録
              ROS_INFO("ps_cost: %f",min_ps_cost);
              
            }
        }
        std_msgs::Float64 invansion_msg;
        invansion_msg.data = invansion;
        invansion_pub_.publish(invansion_msg);
        return min_ps_cost;
  }
}

//追加したコード
// ここから

// void PersonalSpaceDWA::updateObstaclesFromClusters(potbot_nav::StateLayer& state_layer) 
// {   
//     const auto& clusters_obstaclearray = state_layer.getClustersObstacleArray();
//     ROS_INFO("Number of clusters: %zu", clusters_obstaclearray.data.size());

//     std::vector<Pedestrian> peds;

//     double yaw = PersonalSpaceDWA::estimatePedestrianOrientationFromClusters(clusters_obstaclearray);

//     ROS_INFO("updateObstaclesFromClusters called");
//     ROS_INFO("Estimated pedestrian orientation (yaw): %f", yaw);


//     for (const auto& obstacle : clusters_obstaclearray.data) 
//     {
//         // 各障害物の座標 (x, y) を取得
//         double x = obstacle.pose.position.x;
//         double y = obstacle.pose.position.y;

//         // 歩行者データを生成
//         Pedestrian ped;

//         ped.yaw = yaw;

//         peds.push_back(ped);

//         // 障害物を設置
//         setObstacle(x, y);
//     }
    
//     current_pedestrians_ = peds;
//     ROS_INFO("Adding pedestrian: yaw=%f", yaw);
// }



// double PersonalSpaceDWA::calculatePersonalSpaceCost(const base_local_planner::Trajectory& traj)
// {
//   double max_cost = 0.0;
//   const std::vector<Pedestrian>& pedestrians = current_pedestrians_;

//   // ROS_INFO("Debugging calculatePersonalSpaceCost:");
//   // ROS_INFO("  Number of pedestrians: %zu", current_pedestrians_.size());

//   for (unsigned int i = 0; i < traj.getPointsSize(); ++i)
//   {
//     double x, y, th;
//     traj.getPoint(i, x, y, th); //trajのi番目のポイントを取得
//     // ROS_INFO("  Trajectory point: x=%f, y=%f", x, y);
//     geometry_msgs::Point point;
//     point.x = x;
//     point.y = y;
//     for (const auto& pedestrian : pedestrians) 
//     {
//       ROS_INFO("  Pedestrian position: x=%f, y=%f, yaw=%f", pedestrian.x, pedestrian.y, pedestrian.yaw);
//       double invasion = calculatePersonalSpaceInvasion(point, pedestrian);
//       double cost = calculateCostFromInvasion(invasion);
//       max_cost = std::max(max_cost, cost);
//     }
//   }
//   // ROS_INFO("Maximum personal space cost: %f", max_cost);
//   return max_cost;
// }

// double PersonalSpaceDWA::calculatePersonalSpaceInvasion(const geometry_msgs::Point& point, const Pedestrian& pedestrian)
// { 
//   for(auto& coord_obstacle : obstacles_)
//   {   
//       double x_obstacle = coord_obstacle.x;
//       double y_obstacle = coord_obstacle.y;

//       double dx = point.x - x_obstacle;
//       double dy = point.y - y_obstacle;
//       double theta = std::atan2(dy, dx) - pedestrian.yaw;
//       ROS_INFO("dx: %f", dx);
//       ROS_INFO("dy: %f", dy);
  

//         while (theta > M_PI) theta -= 2 * M_PI;
//         while (theta < -M_PI) theta += 2 * M_PI;

//       if (std::abs(theta) <= M_PI / 2) {
//         // 歩行者の前方の楕円形パーソナルスペース
//         return std::sqrt(std::pow(dx / (lf_scale_ * lf_scale_), 2) + std::pow(dy / (ls_scale_ * ls_scale_), 2));
//       } else {
//         // 歩行者の後方の円形パーソナルスペース
//         return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) / (ls_scale_ * ls_scale_ );
//       }
//   }
// }

// double PersonalSpaceDWA::calculateCostFromInvasion(double invasion)
//  {
//       if (invasion <= 0.5) {
//         return 254.0;  // 最大コスト（障害物と同等）
//       } else {
//         return 254.0 / (invasion+1e-100);  // 距離に応じてコストを減少
//       }
//   }

// double PersonalSpaceDWA::calculateDWACostWithPersonalSpace(const std::vector<base_local_planner::Trajectory>& all_explored)
//  {
//     double best_total_score = -DBL_MAX;
//     base_local_planner::Trajectory* best_traj = nullptr;
    
//     // すべての軌道を評価
//     for (const auto& traj : all_explored) {
//           base_local_planner::Trajectory traj_copy = traj;
//       if (traj.cost_ < 0) continue;
        
//         // パーソナルスペースコストの計算
//         double ps_cost = calculatePersonalSpaceCost(traj);

//         double base_score = scored_sampling_planner_.scoreTrajectory(traj_copy, DBL_MAX);

//         double normalized_ps_cost = ps_cost / 254.0;
        
//         // 総合コストの計算
//         double total_score = (1 - ps_weight_) * base_score - ps_weight_ * ps_cost;

//         if (total_score > best_total_score) {
//             best_total_score = total_score;
//             best_traj = &traj_copy;
        
//         // デバッグ情報の出力
//         // ROS_INFO("  base_score: %f", base_score);
//         // ROS_INFO("  ps_cost: %f", ps_cost);
//         // ROS_INFO("  normalized_ps_cost: %f", normalized_ps_cost);
//         // ROS_INFO("  total_cost: %f", total_score);
//     }
//     }
    
//     if (best_traj != nullptr) {
//         result_traj_ = *best_traj;
//         return best_total_score;
//     }
    
//     return -1.0;
// }

 

// double PersonalSpaceDWA::estimatePedestrianOrientationFromClusters(const potbot_msgs::ObstacleArray& clusters_obstaclearray) 
// {
//     double sum_yaw = 0;
//     int count = 0;

//     // 周辺の障害物を探索し、向きを推定
//     for (const auto& obstacle : clusters_obstaclearray.data) 
//     {
//         // 障害物のIDを取得
//         int id = obstacle.id; // 障害物の識別子 (クラスタリングによってIDが付与される前提)

//         double current_x = obstacle.pose.position.x;
//         double current_y = obstacle.pose.position.y;

//         // 前回の位置が存在するか確認
//         if (previous_positions.find(id) != previous_positions.end()) 
//         {
//           // 前回の位置を取得
//             double previous_x = previous_positions[id].first;
//             double previous_y = previous_positions[id].second;

//             // 差分を計算
//             double dx = current_x - previous_x;
//             double dy = current_y - previous_y;

//             // yaw角を計算 (進行方向)
//             double yaw = std::atan2(dy, dx);

//             // 近傍の障害物のみを考慮（距離閾値 1.0）
//             double distance = std::sqrt(dx * dx + dy * dy);
//             if (distance < 1.0) 
//             {
//                 sum_yaw += yaw;
//                 count++;
//             }

            
//         }

//         // 現在の位置を次のステップ用に保存
//         previous_positions[id] = {current_x, current_y};
//     }

//           if (count > 0) 
//           {
//               return sum_yaw / count; // 平均的なyaw角を返す
//           }

//           return 0.0; // デフォルト値（向き不明の場合）
// }


// double PersonalSpaceDWA::calculateTotalPersonalSpaceViolation(const geometry_msgs::Point& robot_pos) 
// {
//     double total_violation = 0.0;

//     for (const auto& pedestrian : current_pedestrians_) 
//     {
//         double violation = calculatePersonalSpaceInvasion(robot_pos, pedestrian);
//         total_violation += violation;
//     }

//     return total_violation;
// }

// };