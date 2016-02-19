#include "gazebo/physics/physics.hh"
#include "LinkData.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"


namespace gazebo {
	class MazePlugin: public WorldPlugin {

		public:
			MazePlugin();

			void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

			void Regenerate(ConstGzStringPtr &msg);

			sdf::ElementPtr GenerateLinkSDF(gui::LinkData *_link);

		private:

      /// \brief A map of model link names to their data.
      std::map<std::string, LinkData *> allLinks;

			transport::NodePtr node;
			sdf::SDFPtr modelSDF;

			transport::SubscriberPtr sub;
			physics::WorldPtr _parent;

	};
}

