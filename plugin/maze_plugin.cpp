#include "maze_plugin.h"

#include <ignition/math/Pose3.hh>

namespace gazebo {

	MazePlugin::MazePlugin() {}

	void MazePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
		sdf::SDFPtr modelSDF;

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
