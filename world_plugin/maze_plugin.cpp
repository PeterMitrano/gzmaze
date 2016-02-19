#include "maze_plugin.h"

#include <ignition/math/Pose3.hh>

namespace gazebo {

  MazePlugin::MazePlugin(): modelSDF() {}

  void MazePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    this->parent = _parent;
    node = transport::NodePtr(new transport::Node());
    node->Init(parent->GetName());
    sub = node->Subscribe("~/maze/regenerate", &MazePlugin::Regenerate, this);
  }

  void MazePlugin::Regenerate(ConstGzStringPtr &msg){
    std::string maze_filename = msg->data();

    sdf::ElementPtr model = LoadModel();

    AddWalls(model);

    if (maze_filename == "random") {
      //create random maze here
    }
    else {
      //load maze from file
    }

    model->GetAttribute("name")->Set("my_maze");
    model->GetElement("pose")->Set(
        math::Pose(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0)));
    parent->InsertModelSDF(*modelSDF);
  }

  void MazePlugin::AddWalls(sdf::ElementPtr model){
    msgs::Link link;
    link.set_name("walls");

    msgs::Vector3d size;
    size.set_x(0.05);
    size.set_y(0.1);
    size.set_z(0.2);

    msgs::BoxGeom box;
    box.set_allocated_size(&size);

    msgs::Geometry geometry;
    geometry.set_type(msgs::Geometry_Type::Geometry_Type_BOX);
    geometry.set_allocated_box(&box);

    msgs::Visual *visual = link.add_visual();
    visual->set_name("v1");
    visual->set_parent_name("walls");
    visual->set_type(msgs::Visual_Type::Visual_Type_LINK);
    visual->set_allocated_geometry(&geometry);

    sdf::ElementPtr newLinkElem = msgs::LinkToSDF(link);
    model->InsertElement(newLinkElem);
  }

  sdf::ElementPtr MazePlugin::LoadModel() {
    modelSDF.reset(new sdf::SDF);

    sdf::initFile("root.sdf", modelSDF);
    sdf::readFile("maze_base/model.sdf", modelSDF);

    return modelSDF->Root()->GetElement("model");
  }

  GZ_REGISTER_WORLD_PLUGIN(MazePlugin)
}