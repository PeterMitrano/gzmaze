#include "MazeFactory.hh"

#include <ignition/math/Pose3.hh>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cmath>

namespace gazebo
{

const float MazeFactory::UNIT = 0.18; //distance between centers of squares
const float MazeFactory::WALL_HEIGHT = 0.05;
const float MazeFactory::WALL_LENGTH = 0.192;
const float MazeFactory::WALL_THICKNESS = 0.012;
const float MazeFactory::BASE_HEIGHT= 0.005;
const float MazeFactory::PAINT_THICKNESS = 0.01;


MazeFactory::MazeFactory(): distribution(0,4) {}

void MazeFactory::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  node = transport::NodePtr(new transport::Node());
  node->Init(parent->GetName());
  sub = node->Subscribe("~/maze/regenerate", &MazeFactory::Regenerate, this);

  //seed random generator
  generator.seed(time(0));
}

void MazeFactory::Regenerate(ConstGzStringPtr &msg)
{
  maze_filename = msg->data();

  sdf::ElementPtr model = LoadModel();
  sdf::ElementPtr base_link = model->GetElement("link");

  if (maze_filename == "random")
  {
    //create random maze here
    InsertRandomWalls(base_link);
  }
  else
  {
    //load maze from file
    gzmsg << "loading from file " << maze_filename << std::endl;
    InsertWallsFromFile(base_link);
  }

  model->GetAttribute("name")->Set("my_maze");
  model->GetElement("pose")->Set(
    math::Pose(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0)));
  parent->InsertModelSDF(*modelSDF);
}

void MazeFactory::InsertWallsFromFile(sdf::ElementPtr base_link)
{
  std::fstream fs;
  fs.open(maze_filename, std::fstream::in);

  if (fs.good()){
    //clear the old walls
    all_wall_elements.clear();

    std::string line;

    //look West and North to connect any nodes
    for (int i=0;i<MAZE_SIZE;i++){ //read in each line
      std::getline(fs, line);

      if (!fs) {
        gzmsg  << "getline failed" << std::endl;
        return;
      }

      int charPos = 0;
      for (int j=0;j<MAZE_SIZE;j++){
        if (line.at(charPos) == '|'){
          //add a wall on the west
          InsertWall(base_link, i,j,Direction::W);
        }
        charPos++;
        if (line.at(charPos) == '_'){
          //add a wall on the south
          InsertWall(base_link, i,j,Direction::S);
        }
        charPos++;
      }
    }

    //add east and north walls
    for (int i=0;i<MAZE_SIZE;i++){
      InsertWall(base_link, i, MAZE_SIZE - 1, Direction::E);
      InsertWall(base_link, 0, i, Direction::N);
    }
  }
  else {
    gzmsg << "failed to load file " << maze_filename << std::endl;
  }
}

void MazeFactory::InsertRandomWalls(sdf::ElementPtr link)
{
  //reset
  for (int i=0;i<MAZE_SIZE;i++){
    for (int j=0;j<MAZE_SIZE;j++){
      visited[i][j] = false;
      for (int k=0;k<4;k++){
        connected[i][j][k] = false;
      }
    }
  }

  //start with maze "end" in the center
  InsertRandomNeighbor(MAZE_SIZE/2,MAZE_SIZE/2);

  for (int i=0;i<MAZE_SIZE;i++){
    for (int j=0;j<MAZE_SIZE;j++){
      if (!connected[i][j][3]) { InsertWall(link, i, j, Direction::W);}
      if (!connected[i][j][2]) { InsertWall(link, i, j, Direction::S);}
    }

    //add outer walls
    InsertWall(link, i, 0, Direction::W);
    InsertWall(link, i, MAZE_SIZE-1, Direction::E);
    InsertWall(link, 0, i, Direction::N);
    InsertWall(link, MAZE_SIZE-1, i, Direction::S);
  }
}

void MazeFactory::InsertRandomNeighbor(int row, int col)
{
  //make sure it's in bounds
  if (row >= MAZE_SIZE || row < 0 || col >= MAZE_SIZE || col < 0) return;

  //mark current cell visited
  visited[row][col] = true;

  //select random neighbor
  int neighbor = distribution(generator);

  for (int i=0;i<4;i++){
    switch(neighbor){
      case 0:
        if (row >= 0 && !visited[row-1][col]) {
          connected[row][col][neighbor] = true;
          connected[row-1][col][2] = true;
          InsertRandomNeighbor(row-1, col);
        }
        break;
      case 1:
        if (col < MAZE_SIZE && !visited[row][col+1]) {
          connected[row][col][neighbor] = true;
          connected[row][col+1][3] = true;
          InsertRandomNeighbor(row, col+1);
        }
        break;
      case 2:
        if (row < MAZE_SIZE && !visited[row+1][col]) {
          connected[row][col][neighbor] = true;
          connected[row+1][col][0] = true;
          InsertRandomNeighbor(row+1, col);
        }
        break;
      case 3:
        if (col >= 0 && !visited[row][col-1]) {
          connected[row][col][neighbor] = true;
          connected[row][col-1][1] = true;
          InsertRandomNeighbor(row, col-1);
        }
        break;
    }
    neighbor = (neighbor+1)%4;
  }
}

void MazeFactory::InsertWall(sdf::ElementPtr link, int row, int col, Direction dir)
{
  std::list<sdf::ElementPtr> walls_visuals = CreateWallVisual(row,col,dir);
  sdf::ElementPtr walls_collision = CreateWallCollision(row,col,dir);

  //add all those to a list so we can remove them for next time.
  all_wall_elements.insert(all_wall_elements.end(),
      walls_visuals.begin(),
      walls_visuals.end());
  all_wall_elements.push_front(walls_collision);

  //insert all the visuals
  std::list<sdf::ElementPtr>::iterator list_iter = walls_visuals.begin();
  while (list_iter != walls_visuals.end())
  {
    link->InsertElement(*(list_iter++));
  }

  link->InsertElement(walls_collision);
}

std::list<sdf::ElementPtr> MazeFactory::CreateWallVisual(int row, int col, Direction dir)
{
  msgs::Pose *visual_pose = CreatePose(row, col, BASE_HEIGHT + (WALL_HEIGHT - PAINT_THICKNESS)/2, dir);
  msgs::Pose *paint_visual_pose = CreatePose(row, col, BASE_HEIGHT + WALL_HEIGHT - PAINT_THICKNESS/2, dir);

  msgs::Geometry *visual_geo = CreateBoxGeometry(WALL_LENGTH, WALL_THICKNESS, WALL_HEIGHT - PAINT_THICKNESS);
  msgs::Geometry *paint_visual_geo = CreateBoxGeometry(WALL_LENGTH, WALL_THICKNESS, PAINT_THICKNESS);

  msgs::Visual visual;
  std::string visual_name = "v_" + std::to_string(row)
                            + "_" + std::to_string(col) + "_" + to_char(dir);
  visual.set_name(visual_name);
  visual.set_allocated_geometry(visual_geo);
  visual.set_allocated_pose(visual_pose);

  msgs::Material_Script *paint_script = new msgs::Material_Script();
  std::string *uri = paint_script->add_uri();
  *uri = "file://media/materials/scripts/gazebo.material";
  paint_script->set_name("Gazebo/Red");

  msgs::Material *paint_material = new msgs::Material();
  paint_material->set_allocated_script(paint_script);

  msgs::Visual paint_visual;
  std::string paint_visual_name = "paint_v_" + std::to_string(row)
                                  + "_" + std::to_string(col) + "_" + to_char(dir);
  paint_visual.set_name(paint_visual_name);
  paint_visual.set_allocated_geometry(paint_visual_geo);
  paint_visual.set_allocated_pose(paint_visual_pose);
  paint_visual.set_allocated_material(paint_material);

  sdf::ElementPtr visualElem = msgs::VisualToSDF(visual);
  sdf::ElementPtr visualPaintElem = msgs::VisualToSDF(paint_visual);
  std::list<sdf::ElementPtr> visuals;
  visuals.push_front(visualElem);
  visuals.push_front(visualPaintElem);
  return visuals;
}

sdf::ElementPtr MazeFactory::CreateWallCollision(int row, int col, Direction dir)
{
  msgs::Pose *collision_pose = CreatePose(row, col, BASE_HEIGHT + WALL_HEIGHT/2, dir);

  msgs::Geometry *collision_geo = CreateBoxGeometry(WALL_LENGTH, WALL_THICKNESS, WALL_HEIGHT);

  msgs::Collision collision;
  std::string collision_name = "p_" + std::to_string(row) + "_" + std::to_string(col) + "_" + to_char(dir);
  collision.set_name(collision_name);
  collision.set_allocated_geometry(collision_geo);
  collision.set_allocated_pose(collision_pose);

  sdf::ElementPtr collisionElem = msgs::CollisionToSDF(collision);
  return collisionElem;
}

msgs::Pose *MazeFactory::CreatePose(int row, int col, float z, Direction dir) {
  float x_offset=0, y_offset=0;
  float z_rot = 0;

  switch(dir){
    case Direction::N:
      y_offset = UNIT/2;
      break;
    case Direction::E:
     x_offset = UNIT/2;
      z_rot = M_PI/2;
     break;
    case Direction::S:
     y_offset = -UNIT/2;
     break;
    case Direction::W:
     x_offset = -UNIT/2;
     z_rot = M_PI/2;
     break;
  }

  float zero_offset = (UNIT * (MAZE_SIZE - 1)/2);
  float x = -zero_offset + x_offset + col * UNIT;
  float y = zero_offset + y_offset - row * UNIT;

  msgs::Vector3d *position = new msgs::Vector3d();
  position->set_x(x);
  position->set_y(y);
  position->set_z(z);

  msgs::Quaternion *orientation = new msgs::Quaternion();
  orientation->set_z(sin(z_rot/2));
  orientation->set_w(cos(z_rot/2));

  msgs::Pose *pose = new msgs::Pose;
  pose->set_allocated_orientation(orientation);
  pose->set_allocated_position(position);
}

msgs::Geometry *MazeFactory::CreateBoxGeometry(float x, float y, float z)
{
  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(x);
  size->set_y(y);
  size->set_z(z);

  msgs::BoxGeom *box = new msgs::BoxGeom();
  box->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_BOX);
  geo->set_allocated_box(box);

  return geo;
}

sdf::ElementPtr MazeFactory::LoadModel()
{
  modelSDF.reset(new sdf::SDF);

  sdf::initFile("root.sdf", modelSDF);
  sdf::readFile("maze_base/model.sdf", modelSDF);

  return modelSDF->Root()->GetElement("model");
}

Direction operator++(Direction& dir, int) {
  switch(dir){
    case Direction::N:
      dir = Direction::E;
      break;
    case Direction::E:
      dir = Direction::S;
      break;
    case Direction::S:
      dir = Direction::W;
      break;
    case Direction::W:
      dir = Direction::Last;
      break;
    default:
      dir = Direction::INVALID;
  }
  return dir;
}

char MazeFactory::to_char(Direction dir)
{
  switch(dir)
  {
  case Direction::N:
    return 'N';
  case Direction::E:
    return 'E';
  case Direction::S:
    return 'S';
  case Direction::W:
    return 'W';
  default:
    return '\0';
  }
}

GZ_REGISTER_WORLD_PLUGIN(MazeFactory)
}
