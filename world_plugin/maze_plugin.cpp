#include "maze_plugin.h"

#include <ignition/math/Pose3.hh>

namespace gazebo {

  MazePlugin::MazePlugin(): modelSDF() {}

  void MazePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    this->_parent = _parent;
    node = transport::NodePtr(new transport::Node());
    node->Init(_parent->GetName());
    sub = node->Subscribe("~/maze/regenerate", &MazePlugin::Regenerate, this);
  }

  void MazePlugin::Regenerate(ConstGzStringPtr &msg){
    std::string maze_filename = msg->data();

    modelSDF.reset(new sdf::SDF);

    sdf::initFile("root.sdf", modelSDF);
    sdf::readFile("maze_base/model.sdf", modelSDF);
    sdf::ElementPtr modelElem = modelSDF->Root()->GetElement("model");

    modelElem->GetAttribute("name")->Set("my_maze");

    modelElem->GetElement("pose")->Set(
        math::Pose(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0)));

		gui::LinkData link = CreateLink();
    sdf::ElementPtr newLinkElem = this->GenerateLinkSDF(link);
    modelElem->InsertElement(newLinkElem);

    if (maze_filename == "random") {
      //create random maze here
    }
    else {
      //load maze from file
    }

    _parent->InsertModelSDF(*modelSDF);
  }

sdf::ElementPtr MazePlugin::GenerateLinkSDF(gui::LinkData *_link)
{
  std::stringstream visualNameStream;
  std::stringstream collisionNameStream;
  visualNameStream.str("");
  collisionNameStream.str("");

  sdf::ElementPtr newLinkElem = _link->linkSDF->Clone();
  newLinkElem->GetElement("pose")->Set(_link->Pose());

  // visuals
  for (auto const &it : _link->visuals)
  {
    rendering::VisualPtr visual = it.first;
    msgs::Visual visualMsg = it.second;
    sdf::ElementPtr visualElem = visual->GetSDF()->Clone();

    visualElem->GetElement("transparency")->Set<double>(
        visualMsg.transparency());
    newLinkElem->InsertElement(visualElem);
  }

  // collisions
  for (auto const &colIt : _link->collisions)
  {
    sdf::ElementPtr collisionElem = msgs::CollisionToSDF(colIt.second);
    newLinkElem->InsertElement(collisionElem);
  }
  return newLinkElem;
}

LinkData *ModelCreator::CreateLink(const rendering::VisualPtr &_visual)
{
  LinkData *link = new LinkData();

  msgs::Model model;
  double mass = 1.0;

  // set reasonable inertial values based on geometry
  std::string geomType = _visual->GetGeometryType();
  if (geomType == "cylinder")
    msgs::AddCylinderLink(model, mass, 0.5, 1.0);
  else if (geomType == "sphere")
    msgs::AddSphereLink(model, mass, 0.5);
  else
    msgs::AddBoxLink(model, mass, ignition::math::Vector3d::One);
  link->Load(msgs::LinkToSDF(model.link(0)));

  MainWindow *mainWindow = gui::get_main_window();
  if (mainWindow)
  {
    connect(gui::get_main_window(), SIGNAL(Close()), link->inspector,
        SLOT(close()));
  }

  link->linkVisual = _visual->GetParent();
  link->AddVisual(_visual);

  link->inspector->SetLinkId(link->linkVisual->GetName());

  // override transparency
  _visual->SetTransparency(_visual->GetTransparency() *
      (1-ModelData::GetEditTransparency()-0.1)
      + ModelData::GetEditTransparency());

  // create collision with identical geometry
  rendering::VisualPtr collisionVis =
      _visual->Clone(link->linkVisual->GetName() + "::collision",
      link->linkVisual);

  // orange
  collisionVis->SetMaterial("Gazebo/Orange");
  collisionVis->SetTransparency(
      math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));
  ModelData::UpdateRenderGroup(collisionVis);
  link->AddCollision(collisionVis);

  std::string linkName = link->linkVisual->GetName();

  std::string leafName = linkName;
  size_t idx = linkName.rfind("::");
  if (idx != std::string::npos)
    leafName = linkName.substr(idx+2);

  link->SetName(leafName);

  {
    std::lock_guard<std::recursive_mutex> lock(this->updateMutex);
    this->allLinks[linkName] = link;
    if (this->canonicalLink.empty())
      this->canonicalLink = linkName;
  }

  this->ModelChanged();

  return link;
}


GZ_REGISTER_WORLD_PLUGIN(MazePlugin)
}
