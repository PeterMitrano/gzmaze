#include "MousePlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(MousePlugin)

void MousePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  body = model->GetLink(sdf->Get<std::string>("body"));

  node = transport::NodePtr(new transport::Node());
  node->Init();
  pose_pub = node->Advertise<msgs::Pose>("~/mouse/pose");

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MousePlugin::Update, this, _1));
}

void MousePlugin::Update(const common::UpdateInfo &info) {
  PublishInfo();
}

void MousePlugin::PublishInfo(){
  math::Pose realtivePose = body->GetWorldPose();

  msgs::Vector3d *pos = new msgs::Vector3d();
  pos->set_x(realtivePose.pos[0]);
  pos->set_y(realtivePose.pos[1]);
  pos->set_z(realtivePose.pos[2]);

  msgs::Quaternion *rot = new msgs::Quaternion();
  rot->set_x(realtivePose.rot.x);
  rot->set_y(realtivePose.rot.y);
  rot->set_z(realtivePose.rot.z);
  rot->set_w(realtivePose.rot.w);

  msgs::Pose pose;
  pose.set_allocated_position(pos);
  pose.set_allocated_orientation(rot);

  pose_pub->Publish(pose);
}
