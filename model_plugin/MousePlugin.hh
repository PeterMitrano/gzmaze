#pragma once

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>

using namespace gazebo;

class MousePlugin: public ModelPlugin {
  public:

    /// \brief Load the dc motor and configures it according to the sdf.
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    /// \brief Update the torque on the joint from the dc motor each timestep.
    void Update(const common::UpdateInfo &info);

  private:

    physics::JointPtr left_wheel_joint;
    physics::JointPtr right_wheel_joint;

    /// \brief Callback for receiving msgs and storing the signal.
    void ControlCallback(ConstGzStringPtr &msg);

    physics::ModelPtr model;
    event::ConnectionPtr updateConn;
    transport::NodePtr node;
    transport::SubscriberPtr control_sub;
    transport::PublisherPtr sense_pub;
    transport::PublisherPtr pose_pub;

};
