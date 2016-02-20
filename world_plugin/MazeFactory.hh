#include <utility>
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

			enum class Direction {
				N,
				E,
				S,
				W,
				Last,
				First = N,
				INVALID = -1
			};

      // \brief load from maze_base/model.sdf
      sdf::ElementPtr LoadModel();

      sdf::ElementPtr InsertWalls();

      std::list<sdf::ElementPtr> CreateWallVisual(int row,
        int col,
        Direction dir);

      sdf::ElementPtr CreateWallCollision(int row,
        int col,
        Direction dir);

      void InsertWall(sdf::ElementPtr link, int row,
        int col,
        Direction dir);

      std::pair<float, float> ToLocation(int row, int col, Direction dir);

			msgs::Geometry *CreateBoxGeometry(float x, float y, float z);
			msgs::Pose *CreatePose(float px, float py, float pz,
          float ox, float oy, float oz, float ow);

			transport::NodePtr node;
			sdf::SDFPtr modelSDF;

			transport::SubscriberPtr sub;
			physics::WorldPtr parent;

      const static int MAZE_SIZE;
      const static float WALL_LENGTH,
            WALL_HEIGHT,
            WALL_THICKNESS,
            PAINT_THICKNESS,
            UNIT,
            BASE_HEIGHT;

	};
}

