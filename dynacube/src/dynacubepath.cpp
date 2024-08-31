#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>
#include <casadi/casadi.hpp>
#include <fstream>

using namespace casadi;

MX f(const MX& x, const MX& u) {
    // Extract control inputs
    MX U1 = u(0), U2 = u(1), U3 = u(2);

    MX xd = U1;
    MX yd = U2;
    MX zd = U3;
   
    std::vector<MX> state_derivatives = {xd, yd, zd};
    MX x_dot = vertcat(state_derivatives);
    return x_dot;
}

double triangularWave(double t, double T) {
    // Calculate the normalized time within one period
    double normalizedTime = std::fmod(t, T) / T;
    // Calculate the triangular wave value
    return 4.0 * std::fabs(normalizedTime - 0.5);
}

class PathPlanning {
public:
    PathPlanning() : nh_(), state_sub_(nh_.subscribe("/gazebo/model_states", 10, &PathPlanning::stateCallback, this)),
                      traj_pub_(nh_.advertise<std_msgs::Float64MultiArray>("optimal_state", 10)),
                      last_time_(ros::Time::now()) {}

    void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        try {
            if (msg->pose.size() <= 1) {
                ROS_WARN("Insufficient pose data received.");
                return;
            }
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time_).toSec();
            last_time_ = current_time;

            auto it = std::find(msg->name.begin(), msg->name.end(), "drone");
            if (it == msg->name.end()) {
                ROS_ERROR("Drone model not found in the model states.");
                return;
            }
            int drone_index = std::distance(msg->name.begin(), it);
            geometry_msgs::Pose state = msg->pose[drone_index];
            double rx = state.position.x;
            double ry = state.position.y;
            double rz = state.position.z;
            double rphi = state.orientation.x; 
            double rtheta = state.orientation.y; 
            double rpsi = state.orientation.z; 

            DM initialstates = DM::zeros(3, 1);
            initialstates(0, 0) = rx;
            initialstates(1, 0) = ry;
            initialstates(2, 0) = rz;

            double T = 0.05;
            int N = 20; // number of control intervals

            Opti opti; // Optimization problem
            Slice all;

            // ---- decision variables ---------
            MX X = opti.variable(3, N + 1); // state trajectory
            MX U = opti.variable(3, N); // control trajectory (throttle)

            // ---- objective function ---------
            MX OBJ_FUNC = 0;
            DM Q = DM::zeros(3, 3);
            Q(0, 0) = 2;
            Q(1, 1) = 2;
            Q(2, 2) = 2;

            DM R = DM::zeros(3, 3);
            R(0, 0) = 0.5;
            R(1, 1) = 0.5;
            R(2, 2) = 0.5;

            double xdes = 4, ydes = 9, zdes = 1.25;
            DM reference = DM::vertcat({xdes, ydes, zdes});

            for (int k = 0; k < N; ++k) {
                MX st = X(Slice(), k);  // State at time step k
                OBJ_FUNC += mtimes(mtimes((st - reference).T(), Q), (st - reference));
            }

            for (int k = 0; k < N - 1; ++k) {
                MX con = U(Slice(), k + 1) - U(Slice(), k);
                OBJ_FUNC += mtimes(mtimes(con.T(), R), con);
            }

            opti.minimize(OBJ_FUNC);

            // ---- dynamic constraints --------
            for (int k = 0; k < N; ++k) {
                MX k1 = f(X(all, k), U(all, k));
                MX x_next = X(all, k) + T * k1;
                opti.subject_to(X(all, k + 1) == x_next);
            }

            // ---- obstacle avoidance constraints ----
            for (int k = 0; k < N; ++k) {
                double time = current_time.toSec() + k * T;
                double crx = 2 + triangularWave(time, 20);
                MX ob1 = abs(X(0, k + 1) - (crx - 1)) + 
                         abs(X(0, k + 1) - (crx + 1)) + 
                         abs(X(1, k + 1) - 3.65) + 
                         abs(X(1, k + 1) - 4.35) + 
                         abs(X(2, k + 1) + 0.15) + 
                         abs(X(2, k + 1) - 2.35);

                // Ensure the sum of the absolute differences is greater than 5.5
                opti.subject_to(ob1 >= 5.5);
            }

            // ---- path constraints -----------
            // Initial constraints
            opti.subject_to(X(all, 0) == initialstates);
            // Input constraints
            opti.subject_to(-0.3 <= U(0, all) <= 0.3); 
            opti.subject_to(-0.3 <= U(1, all) <= 0.3);  
            opti.subject_to(-0.3 <= U(2, all) <= 0.3);

            Dict ipopt_options;
            ipopt_options["ipopt.print_level"] = 0;    // Suppresses solver output
            ipopt_options["print_time"] = 0;           // Suppresses the printing of time
            ipopt_options["ipopt.sb"] = "yes";         // Suppresses solver banners
            ipopt_options["ipopt.max_iter"] = 500;    // Set the maximum number of iterations
            ipopt_options["ipopt.tol"] = 1e-3;        // Set tolerance to 1e-3
            opti.solver("ipopt", ipopt_options);

            auto sol = opti.solve();
            DM X_sol = sol.value(X);

            // Prepare message to publish the optimal state vector
            std_msgs::Float64MultiArray state_msg;
            state_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            state_msg.layout.dim[0].label = "states";
            state_msg.layout.dim[0].size = 3; // Only 3 elements
            state_msg.layout.dim[0].stride = 1;
            state_msg.data.resize(3);

            // Extract and assign the last state (3x1 vector)
            state_msg.data[0] = static_cast<double>(X_sol(0, N).scalar()); // X position
            state_msg.data[1] = static_cast<double>(X_sol(1, N).scalar()); // Y position
            state_msg.data[2] = static_cast<double>(X_sol(2, N).scalar()); // Z position
            traj_pub_.publish(state_msg);

            // Save the state to a file along with the current time
            std::ofstream outfile;
            outfile.open("dev_ws/src/dyna/data/planningdynauav.txt", std::ios_base::app); // Append mode
            outfile << "Time: " << current_time.toSec() << ", x: " << state_msg.data[0] << ", y: " << state_msg.data[1] << ", z: " << state_msg.data[2] << "\n";
            outfile.close();

            ROS_INFO("Robot's position: x: %f, y: %f, z: %f, phi: %f, theta: %f, psi: %f", rx, ry, rz, rphi, rtheta, rpsi);
            ROS_INFO("Optimal state published: X = %f, Y = %f, Z = %f", state_msg.data[0], state_msg.data[1], state_msg.data[2]);
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in stateCallback: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Publisher traj_pub_;
    ros::Time last_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning");
    PathPlanning path_planning;
    ros::spin();
    return 0;
}
