#pragma once

#include "gazebo/rendering/Visual.hh"

namespace gazebo {
  namespace gui {
    /// \class LinkData LinkData.hh
    /// \brief Helper class to store link data
    class LinkData
    {

        /// \brief Constructor
      public: LinkData();

              /// \brief Destructor
      public: ~LinkData();

              /// \brief Get the name of the link.
              /// \return Name of link.
      public: std::string GetName() const;

              /// \brief Set the name of the link.
              /// \param[in] _name Name of link.
      public: void SetName(const std::string &_name);

              /// \brief Get the pose of the link.
              /// \return Pose of link.
      public: ignition::math::Pose3d Pose() const;

              /// \brief Set the pose of the link.
              /// \param[in] _pose Pose of link.
      public: void SetPose(const ignition::math::Pose3d &_pose3d);

              /// \brief Load the link with data from SDF.
              /// \param[in] _sdf Link SDF element.
      public: void Load(sdf::ElementPtr _sdf);

              /// \brief Get the scale of the link.
              /// \return Scale of link.
      public: ignition::math::Vector3d Scale() const;

              /// \brief Set the scale of the link.
              /// \param[in] _scale Scale of link.
      public: void SetScale(const ignition::math::Vector3d &_scale);

              /// \brief Add a visual to the link.
              /// \param[in] _visual Visual to be added.
      public: void AddVisual(rendering::VisualPtr _visual);

              /// \brief Add a collision to the link.
              /// \param[in] _collisionVis Visual representing the collision.
              /// \param[in] _msg Optional message containing collision params.
      public: void AddCollision(rendering::VisualPtr _collisionVis,
                  const msgs::Collision *_msg = NULL);

              /// \brief Clone the link data.
              /// \param[in] _newName Name to give to the cloned link.
              /// \return A clone of this link data.
      public: LinkData *Clone(const std::string &_newName);

              /// \brief Computes the volume of a link.
              /// \param[in] _collision A collision message.
              /// \return The computed volume.
      public: static double ComputeVolume(const msgs::Collision &_collision);

              /// \brief Computes mass moment of inertia for a link.
              /// \param[in] _collision A collision message.
              /// \param[in] _mass The mass of the link.
              /// \return Vector of principal moments of inertia.
      public: static ignition::math::Vector3d ComputeMomentOfInertia(
                  const msgs::Collision &_collision, const double _mass);

              /// \brief Computes the volume of the link.
              /// \return The volume.
      public: double ComputeVolume() const;

              /// \brief Set the visual for the link.
              /// \param[in] _visual Visual for the link.
      public: void SetLinkVisual(const rendering::VisualPtr _visual);

              /// \brief Get the visual for the link.
              /// \return Visual for the link.
      public: rendering::VisualPtr LinkVisual() const;

               /// \brief SDF representing the link data.
      public: sdf::ElementPtr linkSDF;

              /// \brief mass.
      private: double mass;

               /// \brief Inertia ixx.
      private: double inertiaIxx;

               /// \brief Inertia iyy.
      private: double inertiaIyy;

               /// \brief Inertia izz.
      private: double inertiaIzz;

               /// \brief Scale of link.
      public: ignition::math::Vector3d scale;

              /// \brief Visual representing this link.
      public: rendering::VisualPtr linkVisual;

              /// \brief Visuals of the link.
      public: std::map<rendering::VisualPtr, msgs::Visual> visuals;

              /// \brief Msgs for updating visuals.
      public: std::vector<msgs::Visual *> visualUpdateMsgs;

              /// \brief Msgs for updating collision visuals.
      public: std::vector<msgs::Collision *> collisionUpdateMsgs;

              /// \brief Collisions of the link.
      public: std::map<rendering::VisualPtr, msgs::Collision> collisions;

              /// \brief Flag set to true if this is a link of a nested model.
      public: bool nested;
    };
  }
}
