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
  sense_pub = node->Advertise<msgs::GzString>("~/mouse/sense");

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MousePlugin::Update, this, _1));
}

void MousePlugin::Update(const common::UpdateInfo &info) {
  msgs::Vector3d *pos = new msgs::Vector3d();
  pos->set_x(0);
  pos->set_y(1);
  pos->set_z(2);

  msgs::Quaternion *rot = new msgs::Quaternion();
  rot->set_x(0);
  rot->set_y(0);
  rot->set_z(0);
  rot->set_w(1);

  msgs::Pose pose;
  pose.set_allocated_position(pos);
  pose.set_allocated_orientation(rot);

  msgs::GzString scan;
  scan.set_data("1010");

  sense_pub->Publish(scan);
  pose_pub->Publish(pose);
}

void MousePlugin::ControlCallback(ConstGzStringPtr &msg) {
  gzmsg << msg->data() << std::endl;
}
