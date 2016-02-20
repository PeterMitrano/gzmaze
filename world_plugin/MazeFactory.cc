#include "MazeFactory.hh"

#include <ignition/math/Pose3.hh>

namespace gazebo
{

const float MazeFactory::WALL_HEIGHT = 0.05;
const float MazeFactory::WALL_LENGTH = 0.16;
const float MazeFactory::WALL_THICKNESS = 0.012;
const float MazeFactory::BASE_HEIGHT= 0.1;

MazeFactory::MazeFactory(): modelSDF() {}

void MazeFactory::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  node = transport::NodePtr(new transport::Node());
  node->Init(parent->GetName());
  sub = node->Subscribe("~/maze/regenerate", &MazeFactory::Regenerate, this);
}

void MazeFactory::Regenerate(ConstGzStringPtr &msg)
{
  std::string maze_filename = msg->data();

  sdf::ElementPtr model = LoadModel();

  sdf::ElementPtr walls_link = CreateWalls();
  sdf::ElementPtr walls_joint = CreateJoint();

  model->InsertElement(walls_link);
  //model->InsertElement(walls_joint);

  if (maze_filename == "random")
  {
    //create random maze here
  }
  else
  {
    //load maze from file
  }

  model->GetAttribute("name")->Set("my_maze");
  model->GetElement("pose")->Set(
    math::Pose(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0)));
  parent->InsertModelSDF(*modelSDF);
}

sdf::ElementPtr MazeFactory::CreateJoint()
{
  msgs::Joint joint;

  sdf::ElementPtr newJointElem = msgs::JointToSDF(joint);
  return newJointElem;
}

sdf::ElementPtr MazeFactory::CreateWalls()
{
  msgs::Link link;
  link.set_name("walls");
  link.set_self_collide(true);

  msgs::Vector3d *position = new msgs::Vector3d();
  position->set_z(BASE_HEIGHT);

  msgs::Quaternion *orientation = new msgs::Quaternion();
  orientation->set_x(0);
  orientation->set_y(0);
  orientation->set_z(0);
  orientation->set_w(0);

  msgs::Pose *pose = new msgs::Pose;
  pose->set_allocated_orientation(orientation);
  pose->set_allocated_position(position);

  link.set_allocated_pose(pose);

  msgs::Geometry *visual_geo = CreateWallGeometry();
  msgs::Geometry *collision_geo = CreateWallGeometry();

  msgs::Collision *collision = link.add_collision();
  collision->set_name("c1");
  collision->set_allocated_geometry(collision_geo);

  msgs::Visual *visual = link.add_visual();
  visual->set_name("v1");
  visual->set_allocated_geometry(visual_geo);

  sdf::ElementPtr newLinkElem = msgs::LinkToSDF(link);
  return newLinkElem;
}

msgs::Geometry *MazeFactory::CreateWallGeometry()
{
  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(WALL_LENGTH);
  size->set_y(WALL_THICKNESS);
  size->set_z(WALL_HEIGHT);

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

GZ_REGISTER_WORLD_PLUGIN(MazeFactory)
}
