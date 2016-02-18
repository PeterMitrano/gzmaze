#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "RegenerateWidget.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(RegenerateWidget)

/////////////////////////////////////////////////
RegenerateWidget::RegenerateWidget()
  : GUIPlugin()
{
  this->counter = 0;

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  QFrame *mainFrame = new QFrame();
  QVBoxLayout *frameLayout = new QVBoxLayout();

  QPushButton *button = new QPushButton(tr("Regenerate From File"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  QPushButton *randomButton = new QPushButton(tr("Regenerate Randomly"));
  connect(randomButton, SIGNAL(clicked()), this, SLOT(OnRandomButton()));

  textEdit = new QTextEdit(tr("~/Projects/gzmaze/maze.mz"));
  textEdit->setContentsMargins(1, 1, 1, 1);
  textEdit->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
  textEdit->setObjectName("maze_filename");
  textEdit->setFixedHeight(30);

  frameLayout->addWidget(button);
  frameLayout->addWidget(textEdit);
  frameLayout->addWidget(randomButton);
  mainFrame->setLayout(frameLayout);
  mainLayout->addWidget(mainFrame);

  frameLayout->setContentsMargins(2, 2, 2, 2);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  this->move(10, 10);
  this->resize(250, 90);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->regenPub = this->node->Advertise<msgs::GzString>("~/maze/regenerate");
}

/////////////////////////////////////////////////
RegenerateWidget::~RegenerateWidget()
{
}

/////////////////////////////////////////////////
void RegenerateWidget::OnRandomButton()
{
  msgs::GzString msg;
  msg.set_data("random");
  this->regenPub->Publish(msg);
}

/////////////////////////////////////////////////
void RegenerateWidget::OnButton()
{
  msgs::GzString msg;
  maze_filename = textEdit->toPlainText().toStdString();
  msg.set_data(maze_filename);
  gzmsg << "loading from file " << maze_filename << std::endl;
  this->regenPub->Publish(msg);
}
