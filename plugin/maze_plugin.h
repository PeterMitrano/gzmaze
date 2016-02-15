#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

using namespace gazebo;

class MazePlugin : public ModelPlugin {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo &_info);
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
};
