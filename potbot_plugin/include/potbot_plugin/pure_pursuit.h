#ifndef H_POTBOT_NAV_PURE_PURSUIT_
#define H_POTBOT_NAV_PURE_PURSUIT_

#include <potbot_lib/pure_pursuit.h>
#include <potbot_base/base_controller.h>
#include <potbot_lib/utility.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <potbot_lib/ControllerConfig.h>
#include <visualization_msgs/Marker.h>

namespace potbot_nav
{
    namespace controller
    {
        class PurePursuit : public potbot_base::Controller
        {
            private:
                potbot_lib::controller::PurePursuit pure_pursuit_;
                ros::Publisher pub_lookahead_;
                std::string frame_id_global_ = "map";
                dynamic_reconfigure::Server<potbot_lib::ControllerConfig> *dsrv_;

                void reconfigureCB(const potbot_lib::ControllerConfig& param, uint32_t level); 

                void getLookahead(visualization_msgs::Marker& marker_msg);
                void publishLookahead();

                void purePursuitController();
                void normalizedPurePursuit();

            public:
                PurePursuit(){};
                ~PurePursuit(){};

                void initialize(std::string name);
                
                void calculateCommand(geometry_msgs::Twist& cmd_vel);
                void setTargetPath(const std::vector<geometry_msgs::PoseStamped>& path_msg);

                bool reachedTarget();

        };
    }
}

#endif	// H_POTBOT_NAV_PURE_PURSUIT_