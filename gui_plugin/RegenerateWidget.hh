#ifndef _GUI_EXAMPLE_SPAWN_WIDGET_HH_
#define _GUI_EXAMPLE_SPAWN_WIDGET_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
    class GAZEBO_VISIBLE RegenerateWidget : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: RegenerateWidget();

      /// \brief Destructor
      public: virtual ~RegenerateWidget();

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnRandomButton();

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnBrowseFile();

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnButton();

      /// \brief Counter used to create unique model names
      private: unsigned int counter;

      /// \brief Node used to establish communication with gzserver.
      private: transport::NodePtr node;

      /// \brief Publisher of factory messages.
      private: transport::PublisherPtr regenPub;

      private: QTextEdit *textEdit;
      private: std::string maze_filename;
    };
}
#endif
