#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

#include "ui_regenerate_widget.h"

namespace gazebo
{
class GAZEBO_VISIBLE RegenerateWidget : public GUIPlugin
{
 Q_OBJECT
 public:
  RegenerateWidget();

 protected slots:

  void OnRandomButton();

  void OnBrowseFile();

  void OnButton();

 private:
  transport::NodePtr node;

  transport::PublisherPtr regenerate_pub_;

  std::string maze_filename_;

  Ui_RegenerateWidget ui_;
};
}