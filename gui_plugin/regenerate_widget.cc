#include <sstream>
#include <boost/algorithm/string/replace.hpp>
#include <gazebo/msgs/msgs.hh>
#include "regenerate_widget.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(RegenerateWidget)

RegenerateWidget::RegenerateWidget() : GUIPlugin()
{
  ui_.setupUi(this);

  connect(ui_.browse_button, &QAbstractButton::clicked, this, &RegenerateWidget::OnBrowseFile);
  connect(ui_.from_file_button, &QAbstractButton::clicked, this, &RegenerateWidget::OnButton);
  connect(ui_.random_button, &QAbstractButton::clicked, this, &RegenerateWidget::OnRandomButton);

  node = transport::NodePtr(new transport::Node());
  node->Init();
  regenerate_pub_ = node->Advertise<msgs::GzString>("~/maze/regenerate");
}

void RegenerateWidget::OnBrowseFile()
{
  QFileDialog dialog(this, tr("Open Maze"), QDir::homePath(), tr("Maze Files (*.mz)"));
  auto const flags = Qt::Window | Qt::WindowCloseButtonHint | Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint;
  dialog.setWindowFlags(flags);
  dialog.setAcceptMode(QFileDialog::AcceptOpen);

  if (dialog.exec() == QDialog::Accepted)
  {
    auto const selected = dialog.selectedFiles();
    if (selected.empty())
    {
      return;
    }
    maze_filename_ = selected[0].toStdString();
    ui_.filename_edit->setText(maze_filename_.c_str());
  }
}

void RegenerateWidget::OnRandomButton()
{
  msgs::GzString msg;
  msg.set_data("random");
  regenerate_pub_->Publish(msg);
}

void RegenerateWidget::OnButton()
{
  msgs::GzString msg;
  maze_filename_ = ui_.filename_edit->text().toStdString();
  std::string user = std::getenv("USER");
  boost::replace_all(maze_filename_, "~", "/home/" + user);
  msg.set_data(maze_filename_);
  regenerate_pub_->Publish(msg);
}
