#include <potbot_lib/artificial_potential_field_ros.h>

namespace potbot_lib{

    ArtificialPotentialFieldROS::ArtificialPotentialFieldROS(std::string name)
    {
        apf_ = new ArtificialPotentialField();
        initNode(name);
    }

    void ArtificialPotentialFieldROS::initNode(std::string name)
    {
        ros::NodeHandle private_nh("~/" + name);
        private_nh.getParam("frame_id_global",           frame_id_global_);
        pub_potential_field_ = private_nh.advertise<sensor_msgs::PointCloud2>("field/potential", 1);

        dsrv_ = new dynamic_reconfigure::Server<potbot_lib::PotentialFieldConfig>(private_nh);
        dynamic_reconfigure::Server<potbot_lib::PotentialFieldConfig>::CallbackType cb = boost::bind(&ArtificialPotentialFieldROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        
    }

    void ArtificialPotentialFieldROS::initPotentialField(costmap_2d::Costmap2D* costmap)
    {
        apf_->initPotentialField(
                                costmap->getSizeInCellsY(), costmap->getSizeInCellsX(), 
                                costmap->getResolution(), 
                                costmap->getOriginX() + costmap->getSizeInMetersX()/2, costmap->getOriginY() + costmap->getSizeInMetersY()/2);
    }

    void ArtificialPotentialFieldROS::initPotentialField(costmap_2d::Costmap2DROS* costmap_ros)
    {
        frame_id_global_ = costmap_ros->getGlobalFrameID();
        initPotentialField(costmap_ros->getCostmap());
        geometry_msgs::PoseStamped p;
        costmap_ros->getRobotPose(p);
        setRobot(p);
        clearObstacles();
        setObstacle(costmap_ros->getCostmap());
    }

    void ArtificialPotentialFieldROS::reconfigureCB(const potbot_lib::PotentialFieldConfig& param, uint32_t level)
    {
        Point o = apf_->getOrigin();
        // apf_->initPotentialField(param.potential_field_rows, param.potential_field_cols, param.potential_field_resolution, o.x, o.y);
        apf_->setParams(param.weight_attraction_field, param.weight_repulsion_field, param.distance_threshold_repulsion_field);
    }

    ArtificialPotentialField* ArtificialPotentialFieldROS::getApf()
    {
        return apf_;
    }

    std::string ArtificialPotentialFieldROS::getFrameIdGlobal()
    {
        return frame_id_global_;
    }

    void ArtificialPotentialFieldROS::setGoal(const geometry_msgs::PoseStamped& goal)
    {
        apf_->setGoal(goal.pose.position.x, goal.pose.position.y);
    }

    void ArtificialPotentialFieldROS::setRobot(const geometry_msgs::Pose& robot)
    {
        apf_->setRobot(robot.position.x, robot.position.y);
    }

    void ArtificialPotentialFieldROS::setRobot(const geometry_msgs::PoseStamped& robot)
    {
        setRobot(robot.pose);
    }

    void ArtificialPotentialFieldROS::setRobot(const nav_msgs::Odometry& robot)
    {
        setRobot(robot.pose.pose);
    }

    void ArtificialPotentialFieldROS::setObstacle(const visualization_msgs::Marker& obs)
    {
        // return;
        double origin_x = obs.pose.position.x;
        double origin_y = obs.pose.position.y;
        double origin_th = tf2::getYaw(obs.pose.orientation);
        double res = apf_->getHeader().resolution*5;

        Eigen::MatrixXd vertexes;
        if (obs.type == visualization_msgs::Marker::CUBE)
        {
            double width = obs.scale.x;
            double height = obs.scale.y;

            Eigen::Matrix2d rotation = utility::get_rotate_matrix(origin_th);
            Eigen::MatrixXd translation(4,2);
            translation <<  origin_x, origin_y,
                            origin_x, origin_y,
                            origin_x, origin_y,
                            origin_x, origin_y;
            Eigen::MatrixXd origin_vertexes(4,2);
            origin_vertexes <<  -width/2,   -height/2,
                                -width/2,   height/2,
                                width/2,    height/2,
                                width/2,    -height/2;

            vertexes = rotation*origin_vertexes.transpose() + translation.transpose();
            
        }
        else if (obs.type == visualization_msgs::Marker::SPHERE)
        {
            double width = obs.scale.x;
            double height = obs.scale.y;

            Eigen::Matrix2d rotation = utility::get_rotate_matrix(origin_th);
            Eigen::Vector2d translation;
            translation <<  origin_x, origin_y;
            Eigen::MatrixXd origin_vertexes(4,2);
            
            size_t vertex_num = 2*M_PI/res;
            vertexes.resize(2,vertex_num);
            for (size_t i = 0; i < vertex_num; i++)
            {
                double t = 2 * M_PI * i / vertex_num;
                double x = width/2 * cos(t);
                double y = height/2 * sin(t);
                Eigen::Vector2d p;
                p<< x,y;
                vertexes.col(i) = rotation*p + translation;
            }
        }

        for (size_t i = 0; i < vertexes.cols(); i++)
        {
            // std::cout<<vertexes.row(i)<<std::endl;
            apf_->setObstacle(vertexes.col(i));
        }
    }

    void ArtificialPotentialFieldROS::setObstacle(const std::vector<visualization_msgs::Marker>& obs)
    {
        for (const auto& o:obs)
        {
            setObstacle(o);
        }
    }
    void ArtificialPotentialFieldROS::setObstacle(const geometry_msgs::Point& obs)
    {
        apf_->setObstacle(obs.x, obs.y);
    }

    void ArtificialPotentialFieldROS::setObstacle(const std::vector<geometry_msgs::Point>& obs)
    {
        for (const auto& o:obs)
        {
            setObstacle(o);
        }
    }

    void ArtificialPotentialFieldROS::setObstacle(const geometry_msgs::PointStamped& obs)
    {
        setObstacle(obs.point);
    }

    void ArtificialPotentialFieldROS::setObstacle(const std::vector<geometry_msgs::PointStamped>& obs)
    {
        for (const auto& o:obs)
        {
            setObstacle(o);
        }
    }

    void ArtificialPotentialFieldROS::setObstacle(const geometry_msgs::Pose& obs)
    {
        setObstacle(obs.position);
    }

    void ArtificialPotentialFieldROS::setObstacle(const std::vector<geometry_msgs::Pose>& obs)
    {
        for (const auto& o:obs)
        {
            setObstacle(o);
        }
    }

    void ArtificialPotentialFieldROS::setObstacle(const geometry_msgs::PoseStamped& obs)
    {
        setObstacle(obs.pose);
    }

    void ArtificialPotentialFieldROS::setObstacle(const std::vector<geometry_msgs::PoseStamped>& obs)
    {
        for (const auto& o:obs)
        {
            setObstacle(o);
        }
    }

    void ArtificialPotentialFieldROS::setObstacle(costmap_2d::Costmap2D* costmap)   //costmapからの障害物設定箇所
    {
        unsigned char* costs = costmap->getCharMap();
        unsigned int map_size = costmap->getSizeInCellsX()*costmap->getSizeInCellsY();
        for (unsigned int i = 0; i < map_size; i++)
        {
            int cost = costs[i];
            if (cost == 254)
            {
                unsigned int xi,yi;
                costmap->indexToCells(i,xi,yi);
                double x,y;
                costmap->mapToWorld(xi,yi,x,y);
                apf_->setObstacle(x, y);
            }
        }
    }

    void ArtificialPotentialFieldROS::clearObstacles()
    {
        apf_->clearObstacles();
    }

    void ArtificialPotentialFieldROS::createPotentialField()
    {
        apf_->createPotentialField();
    }

    void ArtificialPotentialFieldROS::publishPotentialField()
    {
        sensor_msgs::PointCloud2 potential_field_msg;
        potbot_lib::utility::field_to_pcl2(*(apf_->getValues()), potential_field_msg);
        potential_field_msg.header.frame_id = frame_id_global_;
        potential_field_msg.header.stamp = ros::Time::now();
        pub_potential_field_.publish(potential_field_msg);
    }
}