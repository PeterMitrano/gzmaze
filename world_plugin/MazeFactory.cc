#include "MazeFactory.hh"

#include <ignition/math/Pose3.hh>

namespace gazebo
{

const float MazeFactory::WALL_HEIGHT = 0.05;
const float MazeFactory::WALL_LENGTH = 0.16;
const float MazeFactory::WALL_THICKNESS = 0.012;
const float MazeFactory::BASE_HEIGHT= 0.2;
const float MazeFactory::PAINT_THICKNESS = 0.01;


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
  model->InsertElement(walls_joint);

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
  joint.set_name("walls_joint");
  joint.set_parent("base");
  joint.set_child("walls");
  joint.set_type(msgs::Joint_Type::Joint_Type_FIXED);

  sdf::ElementPtr newJointElem = msgs::JointToSDF(joint);
  return newJointElem;
}

sdf::ElementPtr MazeFactory::CreateWalls()
{
  msgs::Link link;
  link.set_name("walls");
  link.set_self_collide(true);

  msgs::Pose *link_pose = CreatePose(0, 0, 0.1, 0, 0, 0, 0);
  msgs::Pose *collision_pose = CreatePose(0, 0, WALL_HEIGHT/2, 0, 0, 0, 0);
  msgs::Pose *visual_pose = CreatePose(0, 0, WALL_HEIGHT/2 - PAINT_THICKNESS/2,
      0, 0, 0, 0);
  msgs::Pose *paint_visual_pose = CreatePose(0, 0, WALL_HEIGHT - PAINT_THICKNESS/2,
      0, 0, 0, 0);

  link.set_allocated_pose(link_pose);

  msgs::Geometry *visual_geo = CreateBoxGeometry(WALL_LENGTH,
                               WALL_THICKNESS,
                               WALL_HEIGHT - PAINT_THICKNESS);

  msgs::Geometry *paint_visual_geo = CreateBoxGeometry(WALL_LENGTH,
                                     WALL_THICKNESS,
                                     PAINT_THICKNESS);

  msgs::Geometry *collision_geo = CreateBoxGeometry(WALL_LENGTH,
                                  WALL_THICKNESS,
                                  WALL_HEIGHT);

  msgs::Collision *collision = link.add_collision();
  collision->set_name("c1");
  collision->set_allocated_geometry(collision_geo);
  collision->set_allocated_pose(collision_pose);

  msgs::Visual *visual = link.add_visual();
  visual->set_name("v1");
  visual->set_allocated_geometry(visual_geo);
  visual->set_allocated_pose(visual_pose);

  msgs::Material_Script *paint_script = new msgs::Material_Script();
  std::string *uri = paint_script->add_uri();
  *uri = "file://media/materials/scripts/gazebo.material";
  paint_script->set_name("Gazebo/Red");

  msgs::Material *paint_material = new msgs::Material();
  paint_material->set_allocated_script(paint_script);

  msgs::Visual *paint_visual = link.add_visual();
  paint_visual->set_name("v1_paint");
  paint_visual->set_allocated_geometry(paint_visual_geo);
  paint_visual->set_allocated_pose(paint_visual_pose);
  paint_visual->set_allocated_material(paint_material);

  sdf::ElementPtr newLinkElem = msgs::LinkToSDF(link);
  return newLinkElem;
}

msgs::Pose *MazeFactory::CreatePose(float px, float py, float pz,
                                    float ox, float oy, float oz, float ow)
{
  msgs::Vector3d *position = new msgs::Vector3d();
  position->set_x(px);
  position->set_y(py);
  position->set_z(pz);

  msgs::Quaternion *orientation = new msgs::Quaternion();
  orientation->set_x(ox);
  orientation->set_y(oy);
  orientation->set_z(oz);
  orientation->set_w(ow);

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

GZ_REGISTER_WORLD_PLUGIN(MazeFactory)
}