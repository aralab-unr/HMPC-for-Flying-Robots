#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <tf/transform_datatypes.h>

double triangularWave(double t, double T) {
    double normalizedTime = std::fmod(t, T) / T;
    return 8.0 * std::fabs(normalizedTime - 0.5);
}

class ObMovement {
public:
    ObMovement() : nh_(),
                   state_sub_(nh_.subscribe("/gazebo/model_states", 10, &ObMovement::stateCallback, this)),
                   set_state_client_(nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state")),
                   last_wave_value1(0.0),  last_wave_value2(0.0),  last_wave_value3(0.0), last_wave_value4(0.0) {}

    void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        try {
            ros::Time current_time = ros::Time::now();
            double t = current_time.toSec();
            // For obstacle 1
            double current_wave_value1 = triangularWave( t, 40) - 2;
            double x1 = current_wave_value1;
            double y1 = -1;
            double z1 = 0;

            double rotate1;
            double rotate2;
            double rotate3;
            double rotate4;
            if ( t <= 0.05){
                rotate1=0;
                rotate2=-2*M_PI;
                rotate3=-2*M_PI;
                rotate4=0;
            }

            if ((current_wave_value1 >= 1.97 && last_wave_value1 < 1.97)) {
                rotate1 = 0;  // Toggle rotation by 180 degrees
            }
            if ((current_wave_value1 <= -1.97 && last_wave_value1 > -1.97)) {
                rotate1 = -2*M_PI;  // Toggle rotation by 180 degrees
            }

            last_wave_value1 = current_wave_value1;

            gazebo_msgs::SetModelState set_model_state1;
            set_model_state1.request.model_state.model_name = "obstacle1";
            set_model_state1.request.model_state.pose.position.x = x1;
            set_model_state1.request.model_state.pose.position.y = y1;
            set_model_state1.request.model_state.pose.position.z = z1;
            set_model_state1.request.model_state.pose.orientation.x = 0;
            set_model_state1.request.model_state.pose.orientation.y = 0;
            set_model_state1.request.model_state.pose.orientation.z = rotate1;
            set_model_state1.request.model_state.pose.orientation.w = 0;

            set_state_client_.call(set_model_state1);  // Send the model state update request
            
            // For obstacle 2
            double current_wave_value2 = -triangularWave(t , 40) + 2;
            double x2 = current_wave_value2;
            double y2 = 3;
            double z2 = 0;

            if ((current_wave_value2 >= 1.97 && last_wave_value2 < 1.97)) {
                rotate2 = 0;  // Toggle rotation by 180 degrees
            }
            if ((current_wave_value2 <= -1.97 && last_wave_value2 > -1.97)) {
                rotate2 = -2*M_PI;  // Toggle rotation by 180 degrees
            }

            last_wave_value2 = current_wave_value2;

            gazebo_msgs::SetModelState set_model_state2;
            set_model_state2.request.model_state.model_name = "obstacle2";
            set_model_state2.request.model_state.pose.position.x = x2;
            set_model_state2.request.model_state.pose.position.y = y2;
            set_model_state2.request.model_state.pose.position.z = z2;
            set_model_state2.request.model_state.pose.orientation.x = 0;
            set_model_state2.request.model_state.pose.orientation.y = 0;
            set_model_state2.request.model_state.pose.orientation.z = rotate2;
            set_model_state2.request.model_state.pose.orientation.w = 0;

            set_state_client_.call(set_model_state2);  // Send the model state update request

            // For obstacle 3
            double current_wave_value3 = -1.5 * triangularWave( t + 1, 60) + 3;
            double x3 = current_wave_value3;
            double y3 = -5;
            double z3 = 0;

            if ((current_wave_value3 >= 2.97 && last_wave_value3 < 2.97)) {
                rotate3 = 0;  // Toggle rotation by 180 degrees
            }
            if ((current_wave_value3 <= -2.97 && last_wave_value3 > -2.97)) {
                rotate3 = -2*M_PI;  // Toggle rotation by 180 degrees
            }
            last_wave_value3 = current_wave_value3;

            gazebo_msgs::SetModelState set_model_state3;
            set_model_state3.request.model_state.model_name = "obstacle3";  // Corrected model name
            set_model_state3.request.model_state.pose.position.x = x3;
            set_model_state3.request.model_state.pose.position.y = y3;
            set_model_state3.request.model_state.pose.position.z = z3;
            set_model_state3.request.model_state.pose.orientation.x = 0;
            set_model_state3.request.model_state.pose.orientation.y = 0;
            set_model_state3.request.model_state.pose.orientation.z = rotate3;
            set_model_state3.request.model_state.pose.orientation.w = 0;

            set_state_client_.call(set_model_state3);  // Send the model state update request


            double current_wave_value4 = 1.5 * triangularWave( t + 8, 60) - 3;
            double x4 = current_wave_value4;
            double y4 = 6;
            double z4 = 0;

            if ((current_wave_value4 >= 2.97 && last_wave_value4 < 2.97)) {
                rotate4 = 0;  // Toggle rotation by 180 degrees
            }
            if ((current_wave_value4 <= -2.97 && last_wave_value4 > -2.97)) {
                rotate4 = -2*M_PI;  // Toggle rotation by 180 degrees
            }    
            last_wave_value4 = current_wave_value4;

            gazebo_msgs::SetModelState set_model_state4;
            set_model_state4.request.model_state.model_name = "obstacle4";  // Corrected model name
            set_model_state4.request.model_state.pose.position.x = x4;
            set_model_state4.request.model_state.pose.position.y = y4;
            set_model_state4.request.model_state.pose.position.z = z4;
            set_model_state4.request.model_state.pose.orientation.x = 0;
            set_model_state4.request.model_state.pose.orientation.y = 0;
            set_model_state4.request.model_state.pose.orientation.z = rotate4;
            set_model_state4.request.model_state.pose.orientation.w = 0;

            set_state_client_.call(set_model_state4);  // Send the model state update request
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in stateCallback: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::ServiceClient set_state_client_;
    double last_wave_value1;
    double last_wave_value2;
    double last_wave_value3;
    double last_wave_value4;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obmovement");
    ObMovement obmovement;
    ros::spin();
    return 0;
}
