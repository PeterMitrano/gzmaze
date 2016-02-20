#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"


namespace gazebo {
	class MazeFactory: public WorldPlugin {

		public:
			MazeFactory();

			void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

			void Regenerate(ConstGzStringPtr &msg);

		private:

      // \brief load from maze_base/model.sdf
      sdf::ElementPtr LoadModel();

      sdf::ElementPtr CreateWalls();
      sdf::ElementPtr CreateJoint();

			msgs::Geometry *CreateWallGeometry();

			transport::NodePtr node;
			sdf::SDFPtr modelSDF;

			transport::SubscriberPtr sub;
			physics::WorldPtr parent;

      const static float WALL_LENGTH,
            WALL_HEIGHT,
            WALL_THICKNESS,
            BASE_HEIGHT;

	};
}

