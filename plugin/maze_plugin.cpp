#include "maze_plugin.h"

GZ_REGISTER_MODEL_PLUGIN(MazePlugin)

void MazePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
  this->model = _parent;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MazePlugin::OnUpdate, this, _1));

  std::string maze_file;
	if (_sdf->HasElement("maze_file")) {
		maze_file = _sdf->Get<std::string>("maze_file");
	}

  gzmsg << "loading plugin" << "file=" << maze_file << std::endl;

	GenerateSDF();

}

void MazePlugin::GenerateSDF() {
  sdf::ElementPtr modelElem;

  this->modelSDF.reset(new sdf::SDF);
  this->modelSDF->SetFromString(ModelData::GetTemplateSDFString());

  modelElem = this->modelSDF->Root()->GetElement("model");

  modelElem->ClearElements();
  modelElem->GetAttribute("name")->Set(this->folderName);

  std::lock_guard<std::recursive_mutex> lock(this->updateMutex);

  if (this->serverModelName.empty())
  {
    // set center of all links and nested models to be origin
    /// \todo issue #1485 set a better origin other than the centroid
    ignition::math::Vector3d mid;
    int entityCount = 0;
    for (auto &linksIt : this->allLinks)
    {
      LinkData *link = linksIt.second;
      if (link->nested)
        continue;
      mid += link->Pose().Pos();
      entityCount++;
    }
    for (auto &nestedModelsIt : this->allNestedModels)
    {
      NestedModelData *modelData = nestedModelsIt.second;

      // get only top level nested models
      if (modelData->Depth() != 2)
        continue;

      mid += modelData->Pose().Pos();
      entityCount++;
    }

    if (!(this->allLinks.empty() && this->allNestedModels.empty()))
    {
      mid /= entityCount;
    }

    this->modelPose.Pos() = mid;
  }

  // Update poses in case they changed
  for (auto &linksIt : this->allLinks)
  {
    LinkData *link = linksIt.second;
    if (link->nested)
      continue;
    ignition::math::Pose3d linkPose =
        link->linkVisual->GetWorldPose().Ign() - this->modelPose;
    link->SetPose(linkPose);
    link->linkVisual->SetPose(linkPose);
  }
  for (auto &nestedModelsIt : this->allNestedModels)
  {
    NestedModelData *modelData = nestedModelsIt.second;

    if (!modelData->modelVisual)
      continue;

    // get only top level nested models
    if (modelData->Depth() != 2)
      continue;

    ignition::math::Pose3d nestedModelPose =
        modelData->modelVisual->GetWorldPose().Ign() - this->modelPose;
    modelData->SetPose(nestedModelPose);
    modelData->modelVisual->SetPose(nestedModelPose);
  }

  // generate canonical link sdf first.
  if (!this->canonicalLink.empty())
  {
    auto canonical = this->allLinks.find(this->canonicalLink);
    if (canonical != this->allLinks.end())
    {
      LinkData *link = canonical->second;
      if (!link->nested)
      {
        link->UpdateConfig();
        sdf::ElementPtr newLinkElem = this->GenerateLinkSDF(link);
        modelElem->InsertElement(newLinkElem);
      }
    }
  }

  // loop through rest of all links and generate sdf
  for (auto &linksIt : this->allLinks)
  {
    LinkData *link = linksIt.second;

    if (linksIt.first == this->canonicalLink || link->nested)
      continue;

    link->UpdateConfig();

    sdf::ElementPtr newLinkElem = this->GenerateLinkSDF(link);
    modelElem->InsertElement(newLinkElem);
  }

  // generate canonical model sdf first.
  if (!this->canonicalModel.empty())
  {
    auto canonical = this->allNestedModels.find(this->canonicalModel);
    if (canonical != this->allNestedModels.end())
    {
      NestedModelData *nestedModelData = canonical->second;
      modelElem->InsertElement(nestedModelData->modelSDF);
    }
  }

  // loop through rest of all nested models and add sdf
  for (auto &nestedModelsIt : this->allNestedModels)
  {
    NestedModelData *nestedModelData = nestedModelsIt.second;

    if (nestedModelsIt.first == this->canonicalModel ||
        nestedModelData->Depth() != 2)
      continue;

    modelElem->InsertElement(nestedModelData->modelSDF);
  }

  // Add joint sdf elements
  this->jointMaker->GenerateSDF();
  sdf::ElementPtr jointsElem = this->jointMaker->SDF();

  sdf::ElementPtr jointElem;
  if (jointsElem->HasElement("joint"))
    jointElem = jointsElem->GetElement("joint");
  while (jointElem)
  {
    modelElem->InsertElement(jointElem->Clone());
    jointElem = jointElem->GetNextElement("joint");
  }

  // Model settings
  modelElem->GetElement("static")->Set(this->isStatic);
  modelElem->GetElement("allow_auto_disable")->Set(this->autoDisable);

  // Add plugin elements
  for (auto modelPlugin : this->allModelPlugins)
    modelElem->InsertElement(modelPlugin.second->modelPluginSDF->Clone());

  // update root visual pose at the end after link, model, joint visuals
  this->previewVisual->SetWorldPose(this->modelPose);
}

void MazePlugin::OnUpdate(const common::UpdateInfo & _info) {
}
