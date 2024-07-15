#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include "nav2_autodock_behavior/autodock.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_autodock_behavior/action/auto_dock.hpp"


using Action = nav2_autodock_behavior::action::AutoDock;

enum class DockState : uint8_t
{
  INVALID = Action::Feedback::STATE_INVALID,
  IDLE = Action::Feedback::STATE_IDLE,
  PREDOCK = Action::Feedback::STATE_PREDOCK,
  PARALLEL_CORRECTION = Action::Feedback::STATE_PARALLEL_CORRECTION,
  STEER_DOCK = Action::Feedback::STATE_STEER_DOCK,
  LAST_MILE = Action::Feedback::STATE_LAST_MILE,
  ACTIVATE_CHARGER = Action::Feedback::STATE_ACTIVATE_CHARGER,
  RETRY = Action::Feedback::STATE_RETRY,
  PAUSE = Action::Feedback::STATE_PAUSE
};

// class DockState{
//     public:
//         DockState(){}
//         ~DockState() = default;

//         uint8_t INVALID = Action::Feedback::STATE_INVALID;
//         uint8_t IDLE = Action::Feedback::STATE_IDLE;
//         uint8_t PREDOCK = Action::Feedback::STATE_PREDOCK;
//         uint8_t PARALLEL_CORRECTION = Action::Feedback::STATE_PARALLEL_CORRECTION;
//         uint8_t STEER_DOCK = Action::Feedback::STATE_STEER_DOCK;
//         uint8_t LAST_MILE = Action::Feedback::STATE_LAST_MILE;
//         uint8_t ACTIVATE_CHARGER = Action::Feedback::STATE_ACTIVATE_CHARGER;
//         uint8_t RETRY = Action::Feedback::STATE_RETRY;
//         uint8_t PAUSE = Action::Feedback::STATE_PAUSE;

//         std::string to_string(uint8_t input){
//             std::unordered_map<uint8_t, std::string>_map;
//             _map = {
//                 {INVALID, "INVALID"},
//                 {IDLE, "IDLE"},
//                 {PREDOCK, "PREDOCK"},
//                 {PARALLEL_CORRECTION, "PARALLEL_CORRECTION"},
//                 {STEER_DOCK, "STEER_DOCK"},
//                 {LAST_MILE, "LAST_MILE"},
//                 {ACTIVATE_CHARGER, "ACTIVATE_CHARGER"},
//                 {RETRY, "RETRY"},
//                 {PAUSE, "PAUSE"}
//             };
//             return _map[input];
//         }

//         double to_percent(std::uint8_t input){
//             /**
//             Simple util to convert DockState to percent representation,
//             use in publishing feedback in dock server
//             */
//             std::unordered_map<std::uint8_t, double>_map;
//             _map = {
//                 {IDLE, 0.0},
//                 {PREDOCK, 0.15},
//                 {PARALLEL_CORRECTION, 0.35},
//                 {STEER_DOCK, 0.50},
//                 {LAST_MILE, 0.8},
//                 {ACTIVATE_CHARGER, 0.9},
//                 {RETRY, 0.1},
//                 {PAUSE, 0.1}
//             };
//             return _map[input];
//         }
// };

namespace autodock_util
{
    geometry_msgs::msg::PoseStamped get_center_tf(
        geometry_msgs::msg::PoseStamped tf_frame1,
        geometry_msgs::msg::PoseStamped tf_frame2,
        double offset=0.0){
        
        geometry_msgs::msg::PoseStamped output_pose;
        output_pose.header.stamp = tf_frame1.header.stamp;
        output_pose.header.frame_id = tf_frame1.header.frame_id;

        double _yaw = -std::atan2(tf_frame2.pose.position.x - tf_frame1.pose.position.x,tf_frame2.pose.position.y - tf_frame1.pose.position.y);
        tf2::Quaternion q;
        q.setRPY(0, 0, _yaw);
        output_pose.pose.position.x = 0.5*(tf_frame1.pose.position.x + tf_frame2.pose.position.x);
        output_pose.pose.position.y = 0.5*(tf_frame1.pose.position.y + tf_frame2.pose.position.y);
        output_pose.pose.position.x += std::cos(_yaw)*offset;
        output_pose.pose.position.y += std::sin(_yaw)*offset;
        output_pose.pose.orientation = tf2::toMsg(q);
        return output_pose;
    };

    double bin_filter(double input, double threshold=0.1){
        /*
        Simple binary filter, to remove small values
        */
        return (input > 0.0) ? std::abs(threshold) : -std::abs(threshold);
    }

    double sat_proportional_filter(double input, double abs_min=0.0, double abs_max=10.0, double factor=1.0){
        
        /**
     * @brief Simple saturated proportional filter
     * @return output filtered value, within boundary
     */

        double output = 0.0;
        input *= factor;
        if (std::abs(input) < abs_min){
            if (input < 0){
                output = -abs_min;
            }else{
                output = abs_min;
            }
        }else if (std::abs(input) > abs_max){
            if (input > 0){
                output = abs_max;
            }else{
                output = -abs_max;
            }
        }else{
            output = input;
        }
        return output;
    }

    void flip_yaw(geometry_msgs::msg::Quaternion& _input_q){
        /*
        Flip yaw angle by 180 degree, input yaw range should be within
        [-pi, pi] radian. Else use set_angle() fn to fix the convention.
        Output will also be within the same range of [-pi, pi] radian.
        */
        double _yaw = tf2::getYaw(_input_q);
        _yaw = (_yaw >= 0) ? _yaw - M_PI : _yaw + M_PI;
        tf2::Quaternion q;
        q.setRPY(0, 0, _yaw);
        _input_q = tf2::toMsg(q);
        // return tf2::toMsg(q);
    }

    void flip_base_frame(geometry_msgs::msg::PoseStamped& _pose){
        /*
        Flip the current reference frame by 180 degree. As such, the negativity 
        of translation is flipped, and the yaw angle is located at the opposite 
        quadrant. Currently is used to flip from 'back dock' to 'front dock'
        */
        
        _pose.pose.position.x = -_pose.pose.position.x;
        _pose.pose.position.y = -_pose.pose.position.y;
        autodock_util::flip_yaw(_pose.pose.orientation);
        // return _pose;
    }
} // namespace autodock_util