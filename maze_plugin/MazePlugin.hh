#include <tuple>
#include <random>
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

  class MazePlugin: public WorldPlugin {

    public:
      MazePlugin();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      void Regenerate(ConstGzStringPtr &msg);

      /// \brief insert a circular visual model that can be set to different colors!
      void InsertIndicator(ConstGzStringPtr &msg);

    private:

      char to_char(Direction dir);

      /// \brief load from maze_base/model.sdf
      sdf::ElementPtr LoadModel();

      /// \brief insert allll the walls
      // \param  base_link this should be the base link of the maze
      void InsertWallsFromFile(sdf::ElementPtr base_link);
      void InsertRandomWalls(sdf::ElementPtr base_link);
      void InsertRandomNeighbor(int row, int col);

      /// \brief insert a wall collision and visual into the given link
      // \param link the link you're inserting the models to
      void InsertWall(sdf::ElementPtr link, int row,
          int col, Direction dir);

      std::list<sdf::ElementPtr> CreateWallVisual(int row,
          int col, Direction dir);

      sdf::ElementPtr CreateWallCollision(int row,
          int col, Direction dir);

      msgs::Geometry *CreateBoxGeometry(float x, float y, float z);
      msgs::Geometry *CreateCylinderGeometry(float r, float h);
      msgs::Pose *CreatePose(int row, int col, float z, Direction dir);

      std::list<sdf::ElementPtr> all_wall_elements;

      transport::NodePtr node;
      transport::SubscriberPtr regen_sub;

      physics::WorldPtr parent;

      sdf::SDFPtr modelSDF;

      std::string maze_filename;

      std::default_random_engine generator;
      std::uniform_int_distribution<int> neighbor_dist;

      constexpr static int MAZE_SIZE = 16;
      const static float WALL_LENGTH,
            WALL_HEIGHT,
            WALL_THICKNESS,
            PAINT_THICKNESS,
            INDICATOR_RADIUS,
            UNIT,
            BASE_HEIGHT;

      bool visited[MAZE_SIZE][MAZE_SIZE];

      /// \brief matrix of bool arrays of size four.
      // [0][0][0] represents the connection of 0,0 to the north
      // [2][1][3] represents the connection of 2,1 to the west
      // ect...
      bool connected[MAZE_SIZE][MAZE_SIZE][4];//order N, E, S, W

  };
}

