#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

double triangularWave(double t, double T) {
    // Calculate the normalized time within one period
    double normalizedTime = std::fmod(t, T) / T;
    // Calculate the triangular wave value
    return 4.0 * std::fabs(normalizedTime - 0.5);
}
class ObMovement {
public:
    ObMovement() : nh_(),
                   state_sub_(nh_.subscribe("/gazebo/model_states", 10, &ObMovement::stateCallback, this)),
                   set_state_client_(nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state")),
                   start_time_(ros::Time::now()) {}

    void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        try {
            // Calculate elapsed time since the start
            ros::Time current_time = ros::Time::now();
            double t = current_time.toSec();
            
            // Calculate the new position [2 + 3*sin(0.05*t), 4, 1.25]
            double x = 2 + triangularWave(t,20);
            double y = 4;
            double z = 1.1;

            // Set the new position
            gazebo_msgs::SetModelState set_model_state;
            set_model_state.request.model_state.model_name = "obstacle"; // Replace with your obstacle model name
            set_model_state.request.model_state.pose.position.x = x;
            set_model_state.request.model_state.pose.position.y = y;
            set_model_state.request.model_state.pose.position.z = z;
            set_model_state.request.model_state.pose.orientation.w = 1.0; // Identity quaternion for no rotation
            set_model_state.request.model_state.pose.orientation.x = 0.0;
            set_model_state.request.model_state.pose.orientation.y = 0.0;
            set_model_state.request.model_state.pose.orientation.z = 0.0;

            // Send the request to update the model's state
            if (set_state_client_.call(set_model_state)) {
                ROS_INFO("Current position: [%f, %f, %f]", x, y, z);
            } else {
                ROS_ERROR("Failed to update obstacle position.");
            }

        } catch (const std::exception& e) {
            ROS_ERROR("Exception in stateCallback: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::ServiceClient set_state_client_;
    ros::Time start_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obmovement");
    ObMovement obmovement;
    ros::spin();
    return 0;
}
