#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"


namespace gazebo {
  class MazePlugin: public WorldPlugin {

    public:
      MazePlugin();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);


    private:
      sdf::ElementPtr model;

			transport::PublisherPtr factoryPub;

  };
}
