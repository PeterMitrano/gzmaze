#include "maze_plugin.h"

#include <ignition/math/Pose3.hh>

namespace gazebo {

	MazePlugin::MazePlugin(): modelSDF() {}

	void MazePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    this->_parent = _parent;
		node = transport::NodePtr(new transport::Node());
		node->Init(_parent->GetName());
		sub = node->Subscribe("~/maze/generate", &MazePlugin::Regenerate, this);

	}

	void MazePlugin::Regenerate(ConstGzStringPtr &msg){

		modelSDF.reset(new sdf::SDF);

		sdf::initFile("root.sdf", modelSDF);
		sdf::readFile("maze_base/model.sdf", modelSDF);
		sdf::ElementPtr modelElem = modelSDF->Root()->GetElement("model");

		modelElem->GetAttribute("name")->Set("my_maze");

		modelElem->GetElement("pose")->Set(
				math::Pose(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0)));

		_parent->InsertModelSDF(*modelSDF);
	}

	GZ_REGISTER_WORLD_PLUGIN(MazePlugin)
}
