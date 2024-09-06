#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float64MultiArray.h>
#include <thread>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class DronePlugin : public gazebo::ModelPlugin {
public:
  DronePlugin() : gazebo::ModelPlugin() {
    std::cout << "Starting drone_plugin" << std::endl;
  }

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;

    if (_sdf->HasElement("updateRate")) {
      this->rate = _sdf->Get<double>("updateRate");
    } else {
      this->rate = 40;
    }

    if (_sdf->HasElement("publishTf")) {
      this->publish_tf = _sdf->Get<bool>("publishTf");
    } else {
      this->publish_tf = true;
    }

    if (_sdf->HasElement("rotorThrustCoeff")) {
      this->rotor_thrust_coeff = _sdf->Get<double>("rotorThrustCoeff");
    } else {
      this->rotor_thrust_coeff = 0.00025;
    }

    if (_sdf->HasElement("rotorTorqueCoeff")) {
      this->rotor_torque_coeff = _sdf->Get<double>("rotorTorqueCoeff");
    } else {
      this->rotor_torque_coeff = 0.000075;
    }

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
      return;
    }

    ROS_INFO("ROS Model Plugin Loaded!");
    this->sub = this->nh.subscribe("/prop_vel", 10, &DronePlugin::Activate_Callback, this);
    this->ros_thread = std::thread(std::bind(&DronePlugin::rosThread, this));
    this->jointbackleft = _model->GetJoint("pbl");
    this->jointbackright = _model->GetJoint("pbr");
    this->jointfrontleft = _model->GetJoint("pfl");
    this->jointfrontright = _model->GetJoint("pfr");
  }

public:
  void OnUpdate() {
    if (this->propvel.size() != 4) {
      ROS_WARN("Propeller velocities not properly initialized.");
      return;
    }

    this->model->GetJoint("pfl")->SetVelocity(0, this->propvel[0]);
    this->model->GetJoint("pfr")->SetVelocity(0, this->propvel[1]);
    this->model->GetJoint("pbl")->SetVelocity(0, this->propvel[2]);
    this->model->GetJoint("pbr")->SetVelocity(0, this->propvel[3]);

    double thrustfl = calculateThrust(this->propvel[0]);
    double thrustfr = calculateThrust(this->propvel[1]);
    double thrustbl = calculateThrust(this->propvel[2]);
    double thrustbr = calculateThrust(this->propvel[3]);

    double torquefl = calculatetorque(this->propvel[0]);
    double torquefr = calculatetorque(this->propvel[1]);
    double torquebl = calculatetorque(this->propvel[2]);
    double torquebr = calculatetorque(this->propvel[3]);

    gazebo::physics::LinkPtr linkfl = this->model->GetLink("propfrontleft");
    gazebo::physics::LinkPtr linkfr = this->model->GetLink("propfrontright");
    gazebo::physics::LinkPtr linkbl = this->model->GetLink("propbackleft");
    gazebo::physics::LinkPtr linkbr = this->model->GetLink("propbackright");
    gazebo::physics::LinkPtr base = this->model->GetLink("base_link");

    linkbl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfl));
    linkbr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfr));
    linkfl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbl));
    linkfr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbr));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquefl));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquefr));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquebl));
    base->AddRelativeTorque(ignition::math::Vector3d(0, 0, torquebr));
 
  }

  void rosThread() {
    while (ros::ok()) {
      ros::spinOnce();
    }
  }

  void Activate_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 4) {
      ROS_WARN("Received incorrect number of propeller velocities.");
      return;
    }
    ROS_INFO("Received Message = [%f, %f, %f, %f].", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    this->propvel = msg->data;
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&DronePlugin::OnUpdate, this));
  }

  double calculateThrust(double w) {
    double thrust = this->rotor_thrust_coeff * w * w;
    return thrust;
  }

  double calculatetorque(double w) {
    double torque =  -sgn(w) * this->rotor_torque_coeff * w * w;
    return torque;
  }

private:
  double rate;
  bool publish_tf;
  double rotor_thrust_coeff;
  double rotor_torque_coeff;
  gazebo::physics::ModelPtr model;
  gazebo::physics::JointPtr jointbackleft;
  gazebo::physics::JointPtr jointbackright;
  gazebo::physics::JointPtr jointfrontleft;
  gazebo::physics::JointPtr jointfrontright;
  gazebo::event::ConnectionPtr updateConnection;
  ros::NodeHandle nh;           // ROS node handler
  ros::Subscriber sub;
  std::thread ros_thread;
  std::vector<double> propvel = std::vector<double>(4, 0.0);
};

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)
