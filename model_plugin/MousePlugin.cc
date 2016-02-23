#include "MousePlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(MousePlugin)

void MousePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  left_wheel_joint = model->GetJoint(sdf->Get<std::string>("left_wheel_joint"));
  right_wheel_joint = model->GetJoint(sdf->Get<std::string>("right_wheel_joint"));

  node = transport::NodePtr(new transport::Node());
  node->Init();
  control_sub = node->Subscribe("~/mouse/control", &MousePlugin::ControlCallback, this);
  pose_pub = node->Advertise<msgs::Pose>("~/mouse/pose");
  sense_pub = node->Advertise<msgs::LaserScan>("~/mouse/sense");

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MousePlugin::Update, this, _1));
}

void MousePlugin::Update(const common::UpdateInfo &info) {
}

void MousePlugin::ControlCallback(ConstGzStringPtr &msg) {
}
