#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <casadi/casadi.hpp>
#include <fstream>

using namespace casadi;

// Global parameters
const double m = 2.79, g = 9.81;
const double Ixx = 0.094, Iyy = 0.073, Izz = 0.108;
double kpx = 0.105, kdx = 0.125, kpy = 0.075, kdy = 0.125;
double prev_err_x = 0.0, prev_err_y = 0.0;

MX f(const MX& x, const MX& u) {
    // Extract state variables
    MX xd = x(0), yd = x(1), zd = x(2), phid = x(3), thetad = x(4), psid = x(5);
    MX x_pos = x(6), y_pos = x(7), z_pos = x(8), phi = x(9), theta = x(10), psi = x(11);
    
    // Extract control inputs
    MX Fy = u(0), U1 = u(1), U2 = u(2), U3 = u(3), U4 = u(4);

    // Trigonometric shorthand
    MX cphi = cos(phi), sphi = sin(phi);
    MX ctheta = cos(theta), stheta = sin(theta);
    MX cpsi = cos(psi), spsi = sin(psi);

    // Equations of motion
    MX xdd = (1/m) * ( (-spsi * cphi + cpsi * stheta * sphi) * Fy + (spsi * sphi + cpsi * stheta * cphi) * U1 );
    MX ydd = (1/m) * ( (cpsi * cphi + spsi * stheta * sphi) * Fy + (spsi * stheta * cphi - cpsi * sphi) * U1 );
    MX zdd = (1/m) * ( (ctheta * sphi) * Fy + ctheta * cphi * U1 - m * g);
    MX phidd = (1/Ixx) * (psid * thetad * (Iyy - Izz) + U2);
    MX thetadd = (1/Iyy) * (psid * phid * (Izz - Ixx) + U3);
    MX psidd = (1/Izz) * (phid * thetad * (Ixx - Iyy) + U4);

    // State derivatives
    std::vector<MX> state_derivatives = {xdd, ydd, zdd, phidd, thetadd, psidd, xd, yd, zd, phid, thetad, psid};
    MX x_dot = vertcat(state_derivatives);

    return x_dot;
}

class MPCController {
public:
    MPCController() : nh_(), 
                      optimal_state_sub_(nh_.subscribe("/optimal_state", 10, &MPCController::optimalStateCallback, this)),
                      propvel_pub_(nh_.advertise<std_msgs::Float64MultiArray>("/prop_vel", 10)),
                      state_sub_(nh_.subscribe("/gazebo/model_states", 10, &MPCController::stateCallback, this)),
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
            geometry_msgs::Twist twist = msg->twist[drone_index];

            double rx = state.position.x;
            double ry = state.position.y;
            double rz = state.position.z;
            double rphi = state.orientation.x; 
            double rtheta = state.orientation.y; 
            double rpsi = state.orientation.z;   
            double rvx = twist.linear.x;
            double rvy = twist.linear.y;
            double rvz = twist.linear.z;
            double rphidot = twist.angular.x;
            double rthetadot = twist.angular.y;
            double rpsidot = twist.angular.z;

            DM initialstates = DM::zeros(12, 1);
            initialstates(0, 0) = rvx;
            initialstates(1, 0) = rvy;
            initialstates(2, 0) = rvz;
            initialstates(3, 0) = rphidot;
            initialstates(4, 0) = rthetadot;
            initialstates(5, 0) = rpsidot;
            initialstates(6, 0) = rx;
            initialstates(7, 0) = ry;
            initialstates(8, 0) = rz;
            initialstates(9, 0) = rphi;
            initialstates(10, 0) = rtheta;
            initialstates(11, 0) = rpsi;

            double T = dt;
            int N = 10; // Number of control intervals

            Opti opti; // Optimization problem
            Slice all;

            // ---- decision variables ---------
            MX X = opti.variable(12, N + 1); // State trajectory
            MX U = opti.variable(5, N); // Control trajectory

            // ---- objective function ---------
            MX OBJ_FUNC = 0;
            DM Q = DM::zeros(4, 4);
            Q(0, 0) = 1.5;
            Q(1, 1) = 175;
            Q(2, 2) = 175;
            Q(3, 3) = 175;

            DM R = DM::zeros(5, 5);
            R(0, 0) = 1.75;
            R(1, 1) = 1.2;
            R(2, 2) = 15;
            R(3, 3) = 15;
            R(4, 4) = 15;

            // Use optimal state values for reference
            if (!optimal_state_received_) {
                ROS_WARN("Optimal state not received yet.");
                return;
            }

            double xdes = optimal_state_(0).scalar();
            double ydes = optimal_state_(1).scalar();
            double zdes = optimal_state_(2).scalar();
            double psides = 0;

            ROS_INFO("Robot's desired: x: %f, y: %f, z: %f", xdes, ydes, zdes);

            double errorx = xdes - rx;
            double errory = ydes - ry;

            double derivativex = -rvx;
            double derivativey = -rvy;

            prev_err_x = errorx;
            prev_err_y = errory;

            // Proportional-Derivative control for pitch and roll
            double thetades = kpx * errorx + kdx * derivativex;
            double phides = -kpy * errory - kdy * derivativey;

            // Clamping the desired pitch and roll to avoid extreme values
            thetades = std::clamp(thetades, -0.2, 0.2);
            phides = std::clamp(phides, -0.2, 0.2);

            std::vector<double> reference = {zdes, phides, thetades, psides};

            for (int k = 0; k < N; ++k) {
                // Extract the specific elements from the state matrix X
                MX st = vertcat(X(8, k), X(9, k), X(10, k), X(11, k));  
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

            // ---- path constraints -----------
            // Initial constraints
            opti.subject_to(X(all, 0) == initialstates);
            // Input constraints
            opti.subject_to(0 <= U(0, all) <= 0.5);
            opti.subject_to(25 <= U(1, all) <= 32); 
            opti.subject_to(-0.05 <= U(2, all) <= 0.05);  
            opti.subject_to(-0.05 <= U(3, all) <= 0.05);
            opti.subject_to(-0.05 <= U(4, all) <= 0.05);

            // Set solver options to suppress the output
            Dict ipopt_options;
            ipopt_options["ipopt.print_level"] = 0;    // Suppresses solver output
            ipopt_options["print_time"] = 0;           // Suppresses the printing of time
            ipopt_options["ipopt.sb"] = "yes";         // Suppresses solver banners
            ipopt_options["ipopt.max_iter"] = 1000;    // Set the maximum number of iterations
            ipopt_options["ipopt.tol"] = 1e-3;  // Example: Setting tolerance to 1e-3
            opti.solver("ipopt", ipopt_options);

            auto sol = opti.solve();
            DM U_sol = sol.value(U);

            double Fys = static_cast<double>(U_sol(0, 0).scalar());
            double U1s = static_cast<double>(U_sol(1, 0).scalar());
            double U2s = static_cast<double>(U_sol(2, 0).scalar());
            double U3s = static_cast<double>(U_sol(3, 0).scalar());
            double U4s = static_cast<double>(U_sol(4, 0).scalar());

            double kt = 0.00025, kd = 0.000075, ly = 0.1845, lx= 0.219;
            double w12, w22, w32, w42, w92;

            w12 = (U2s*kd*lx + U3s*kd*ly + U1s*kd*lx*ly - U4s*kt*lx*ly) / (8 * kd * kt * lx * ly);
            w22 = (-U2s*kd*lx + U3s*kd*ly + U1s*kd*lx*ly + U4s*kt*lx*ly) / (8 * kd * kt * lx * ly);
            w32 = (U2s*kd*lx - U3s*kd*ly + U1s*kd*lx*ly + U4s*kt*lx*ly) / (8 * kd * kt * lx * ly);
            w42 = (-U2s*kd*lx - U3s*kd*ly + U1s*kd*lx*ly - U4s*kt*lx*ly) / (8 * kd * kt * lx * ly);
            w92 = Fys / (2*kt);

            w12 = std::max(0.0, w12);
            w22 = std::max(0.0, w22);
            w32 = std::max(0.0, w32);
            w42 = std::max(0.0, w42);
            w92 = std::max(0.0, w92);

            double w1 = sqrt(w12);
            double w2 = -sqrt(w22);
            double w3 = -sqrt(w32);
            double w4 = sqrt(w42);
            double w9 = sqrt(w92);
            double w10 = sqrt(w92);

            std::ofstream outfilex;
            outfilex.open("dev_ws/src/mpccube/data/positioncube.txt", std::ios_base::app); // Append mode
            outfilex << "Time: " << current_time.toSec() << ", x: " << rx << ", y: " << ry << ", z: " << rz << ", phi: " << rphi << ", theta: " << rtheta << ", psi: " << rpsi << "\n";
            outfilex.close();

            std::ofstream outfileu;
            outfileu.open("dev_ws/src/mpccube/data/controlcube.txt", std::ios_base::app); // Append mode
            outfileu << "Time: " << current_time.toSec() << ", Fy: " << Fys << ", U1: " << U1s << ", U2: " << U2s << ", U3: " << U3s << ", U4: " << U4s << "\n";
            outfileu.close();


            std_msgs::Float64MultiArray float64_array_msg;
            float64_array_msg.data = {w1, w2, w3, w4, w1, w2, w3, w4, w9, w10};
            propvel_pub_.publish(float64_array_msg);
            ROS_INFO("Control input sent to Gazebo.");
            ROS_INFO("Robot's position: x: %f, y: %f, z: %f", rx, ry, rz);
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in stateCallback: %s", e.what());
        }
    }

    void optimalStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.size() != 3) {
            ROS_WARN("Received unexpected optimal state size.");
            return;
        }

        optimal_state_ = DM::zeros(3, 1);
        for (size_t i = 0; i < 3; ++i) {
            optimal_state_(i) = msg->data[i];
        }
        optimal_state_received_ = true;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher propvel_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber optimal_state_sub_;
    ros::Time last_time_;
    DM optimal_state_;
    bool optimal_state_received_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller");
    MPCController controller;
    ros::spin();
    return 0;
}
