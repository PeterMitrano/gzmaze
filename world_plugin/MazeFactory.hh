#include <tuple>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo {

  enum class Direction {
    N,
    E,
    S,
    W,
    Last,
    First = N,
    INVALID = -1
  };

  /**
   * \brief increments the direction in the order N, E, S, W, N, ...
   */
  Direction operator++(Direction& dir, int);

  class MazeFactory: public WorldPlugin {

    public:
      MazeFactory();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      void Regenerate(ConstGzStringPtr &msg);

    private:

      char to_char(Direction dir);

      /// \brief load from maze_base/model.sdf
      sdf::ElementPtr LoadModel();

      /// \brief insert allll the walls
      // \param  base_link this should be the base link of the maze
      void InsertWallsFromFile(sdf::ElementPtr base_link);
      void InsertRandomWalls(sdf::ElementPtr base_link);

      /// \brief insert a wall collision and visual into the given link
      // \param link the link you're inserting the models to
      void InsertWall(sdf::ElementPtr link, int row,
          int col,
          Direction dir);

      std::list<sdf::ElementPtr> CreateWallVisual(int row,
          int col,
          Direction dir);

      sdf::ElementPtr CreateWallCollision(int row,
          int col,
          Direction dir);

      msgs::Geometry *CreateBoxGeometry(float x, float y, float z);
      msgs::Pose *CreatePose(int row, int col, float z, Direction dir);

      std::list<sdf::ElementPtr> all_wall_elements;

      transport::NodePtr node;

      transport::SubscriberPtr sub;

      physics::WorldPtr parent;

      sdf::SDFPtr modelSDF;

      std::string maze_filename;

      const static int MAZE_SIZE;
      const static float WALL_LENGTH,
            WALL_HEIGHT,
            WALL_THICKNESS,
            PAINT_THICKNESS,
            UNIT,
            BASE_HEIGHT;

  };
}

