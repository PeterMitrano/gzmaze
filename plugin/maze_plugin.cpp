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

  physics::JointPtr joint = _parent->GetWorld()->GetPhysicsEngine()->CreateJoint("fixed", _parent);
  gzmsg << "joint name=" << joint->GetName() << std::endl;

  sdf::SDFPtr modelSDF(new sdf::SDF);
  sdf::initFile("root.sdf", modelSDF);


	const std::string filename = "/home/peter/Projects/gzmaze/wall_model/model.sdf";
	if (!sdf::readFile(filename, modelSDF)) {
		gzerr << "Unable to load file[" << filename << "]\n";
		return;
	}
	if (modelSDF->Root()->HasElement("model")) {
    sdf::ElementPtr wall_element = modelSDF->Root()->GetElement("model");
		gzmsg << "adding entity" << wall_element->GetName() << std::endl;
    //joint->Attach(_parent->GetLink("base"), wall_element);
	}
	else {
		gzerr << "No model in SDF\n";
		return;
	}

}


void MazePlugin::OnUpdate(const common::UpdateInfo & _info) {
}
