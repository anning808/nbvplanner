/*
 * Copyright (C) 2014 Roman Bapst, CVG, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether
 * in parts or entirely, is NOT PERMITTED.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include <rotors_gazebo_plugins/gazebo_wing_model_plugin.h>
#include <ctime>
#include <rotors_gazebo_plugins/common.h>
#include <math.h>

namespace gazebo {
GazeboWingModelPlugin::GazeboWingModelPlugin()
    : ModelPlugin(),
      node_handle_(0) {
    _elevon_pos_left = 0.0f;
    _elevon_pos_right = 0.0f;
    _rho = 1.0f;
    _F = 0.15f;
}

GazeboWingModelPlugin::~GazeboWingModelPlugin() {
  this->updateConnection_.reset();
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboWingModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();
  _pose_topic = "sensors/pose1";

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wing_model] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_wing_model] Please specify a linkName.\n";
  // Get the pointer to the link
  this->link_ = this->model_->GetLink(link_name_);
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWingModelPlugin::OnUpdate, this, _1));

  _pose_sub = node_handle_->subscribe(_pose_topic, 1000, &GazeboWingModelPlugin::PoseCallback, this);

}

// Called by the world update start event
void GazeboWingModelPlugin::OnUpdate(const common::UpdateInfo& _info) {
  UpdateForcesAndMoments();
  Publish();
}

void GazeboWingModelPlugin::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose) {
  _q.W() = pose.pose.pose.orientation.w;
  _q.X() = pose.pose.pose.orientation.x;
  _q.Y() = pose.pose.pose.orientation.y;
  _q.Z() = pose.pose.pose.orientation.z;
 }

void GazeboWingModelPlugin::UpdateForcesAndMoments() {
  // compute aerodynamic forces and moments induced by the wing
  ignition::math::Vector3<double> global_vel = link_->WorldLinearVel(); // vel in global frame
  ignition::math::Vector3<double> body_vel = _q.RotateVector(global_vel);
  ignition::math::Vector3<double> aerodynamic_forces = get_aerodynamic_forces(body_vel);
  ignition::math::Vector3<double> aerodynamic_moments = get_aerodynamic_moments(body_vel);

  // add forces and moments to the link
  this->link_->AddForce(aerodynamic_forces);
  this->link_->AddRelativeTorque(aerodynamic_moments);

}

void GazeboWingModelPlugin::Publish() {
  // turning_velocity_msg_.data = this->joint_->GetVelocity(0);
  // motor_vel_pub_.publish(turning_velocity_msg_);
}

ignition::math::Vector3<double> GazeboWingModelPlugin::get_aerodynamic_forces(ignition::math::Vector3<double> &vel) {
  // compute angle of attack
  float alpha = -::atan2f(vel[2],vel[0]);

  float c_A = 0;
  float c_D = 0;
  // get lift coeff from table
  if (alpha <= 1.5f && alpha >= -1.5f) {
    float c_A = alpha/1.5f;
    float c_D = 0.2;
  }
  // compute lift force
  float lift = 0.5f * _rho * c_A * _F * (vel[2]*vel[2] + vel[0]*vel[0]);
  ignition::math::Vector3<double> lift_body(0,0,lift);
  lift_body = _q.RotateVector(lift_body);

  // add lift induced by elevons

  // compute drag force
  float drag = 0.5f * _rho * c_D * _F * (vel[2]*vel[2] + vel[0]*vel[0]);
  ignition::math::Vector3<double> drag_body(-drag,0,0);
  drag_body = _q.RotateVector(drag_body);
  // compute total force acting on wing
  ignition::math::Vector3<double> f_aero = drag_body + lift_body;
}

ignition::math::Vector3<double> GazeboWingModelPlugin::get_aerodynamic_moments(ignition::math::Vector3<double> &vel) {
  // compute lift
  ignition::math::Vector3<double> lift = get_aerodynamic_forces(vel);

  // compute moment due to lift

  // compute moment due to elevons

  // compute total moment
  return ignition::math::Vector3<double>(0.0f,0.0f,0.0f);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWingModelPlugin);
}
