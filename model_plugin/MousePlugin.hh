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

    void PublishInfo();

    physics::ModelPtr model;
    physics::LinkPtr body;
    event::ConnectionPtr updateConn;
    transport::NodePtr node;
    transport::PublisherPtr pose_pub;
};
