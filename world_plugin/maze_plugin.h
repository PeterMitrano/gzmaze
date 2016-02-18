#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"


namespace gazebo {
  class MazePlugin: public WorldPlugin {

    public:
      MazePlugin();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

			void Regenerate(ConstGzStringPtr &msg);

    private:
			transport::NodePtr node;
			sdf::SDFPtr modelSDF;

			transport::SubscriberPtr sub;
      physics::WorldPtr _parent;

  };
}
