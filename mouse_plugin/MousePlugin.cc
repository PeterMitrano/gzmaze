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
  auto const relativePose = body->WorldPose();


  msgs::Pose pose;
  pose.mutable_position()->set_x(relativePose.Pos().X());
  pose.mutable_position()->set_y(relativePose.Pos().Y());
  pose.mutable_position()->set_z(relativePose.Pos().Z());
  pose.mutable_orientation()->set_x(relativePose.Rot().X());
  pose.mutable_orientation()->set_y(relativePose.Rot().Y());
  pose.mutable_orientation()->set_z(relativePose.Rot().Z());
  pose.mutable_orientation()->set_w(relativePose.Rot().W());

  pose_pub->Publish(pose);
}
