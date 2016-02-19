#include "LinkData.hh"

LinkData::LinkData()
{
  this->linkSDF.reset(new sdf::Element);
  sdf::initFile("link.sdf", this->linkSDF);

  this->scale = ignition::math::Vector3d::One;
  this->inertiaIxx = 0;
  this->inertiaIyy = 0;
  this->inertiaIzz = 0;
  this->mass = 0;
  this->nested = false;
}

/////////////////////////////////////////////////
LinkData::~LinkData()
{
}

/////////////////////////////////////////////////
std::string LinkData::GetName() const
{
  return this->linkSDF->Get<std::string>("name");
}

/////////////////////////////////////////////////
void LinkData::SetName(const std::string &_name)
{
  this->linkSDF->GetAttribute("name")->Set(_name);
  this->inspector->SetName(_name);
}

/////////////////////////////////////////////////
ignition::math::Pose3d LinkData::Pose() const
{
  return this->linkSDF->Get<ignition::math::Pose3d>("pose");
}

/////////////////////////////////////////////////
void LinkData::SetPose(const ignition::math::Pose3d &_pose)
{
  this->linkSDF->GetElement("pose")->Set(_pose);

  LinkConfig *linkConfig = this->inspector->GetLinkConfig();
  linkConfig->SetPose(_pose);
}

/////////////////////////////////////////////////
void LinkData::SetScale(const ignition::math::Vector3d &_scale)
{
  GZ_ASSERT(this->linkVisual, "LinkVisual is NULL");
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();

  ignition::math::Vector3d dScale = _scale / this->scale;
  for (auto const &it : this->visuals)
  {
    std::string name = it.first->GetName();
    std::string linkName = this->linkVisual->GetName();
    std::string leafName =
        name.substr(name.find(linkName)+linkName.size()+2);
    ignition::math::Vector3d visOldSize;
    std::string uri;
    visualConfig->Geometry(leafName,  visOldSize, uri);
    ignition::math::Vector3d visNewSize = it.first->GetGeometrySize();
    visualConfig->SetGeometry(leafName, visNewSize);
  }

  std::map<std::string, ignition::math::Vector3d> colOldSizes;
  std::map<std::string, ignition::math::Vector3d> colNewSizes;
  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();
  for (auto const &it : this->collisions)
  {
    std::string name = it.first->GetName();
    std::string linkName = this->linkVisual->GetName();
    std::string leafName =
        name.substr(name.find(linkName)+linkName.size()+2);

    ignition::math::Vector3d colOldSize;
    std::string uri;
    collisionConfig->Geometry(leafName,  colOldSize, uri);
    ignition::math::Vector3d colNewSize = it.first->GetGeometrySize();
    collisionConfig->SetGeometry(leafName, colNewSize);
    colOldSizes[name] = colOldSize;
    colNewSizes[name] = colNewSize;
  }

  if (this->collisions.empty())
    return;

  // update link inertial values - assume uniform density
  LinkConfig *linkConfig = this->inspector->GetLinkConfig();
  sdf::ElementPtr inertialElem = this->linkSDF->GetElement("inertial");

  // update mass
  // density = mass / volume
  // assume fixed density and scale mass based on volume changes.
  double volumeRatio = 1;
  double newVol = 0;
  double oldVol = 0;
  for (auto const &it : this->collisions)
  {
    ignition::math::Vector3d oldSize = colOldSizes[it.first->GetName()];
    ignition::math::Vector3d newSize = colNewSizes[it.first->GetName()];
    std::string geomStr = it.first->GetGeometryType();
    if (geomStr == "sphere")
    {
      // sphere volume: 4/3 * PI * r^3
      oldVol += IGN_SPHERE_VOLUME(oldSize.X() * 0.5);
      newVol += IGN_SPHERE_VOLUME(newSize.X() * 0.5);
    }
    else if (geomStr == "cylinder")
    {
      // cylinder volume: PI * r^2 * height
      oldVol += IGN_CYLINDER_VOLUME(oldSize.X() * 0.5, oldSize.Z());
      newVol += IGN_CYLINDER_VOLUME(newSize.X() * 0.5, newSize.Z());
    }
    else
    {
      // box, mesh, and other geometry types - use bounding box
      oldVol += IGN_BOX_VOLUME_V(oldSize);
      newVol += IGN_BOX_VOLUME_V(newSize);
    }
  }

  if (oldVol < 1e-10)
  {
    gzerr << "Volume is too small to compute accurate inertial values"
        << std::endl;
    return;
  }

  volumeRatio = newVol / oldVol;

  // set new mass
  double oldMass = this->mass;
  double newMass = this->mass * volumeRatio;
  this->mass = newMass;
  linkConfig->SetMass(newMass);

  // scale the inertia values
  // 1) compute inertia size based on current inertia matrix and geometry
  // 2) apply scale to inertia size
  // 3) compute new inertia values based on new size

  // get current inertia values
  double ixx = this->inertiaIxx;
  double iyy = this->inertiaIyy;
  double izz = this->inertiaIzz;

  double newIxx = ixx;
  double newIyy = iyy;
  double newIzz = izz;

  ignition::math::Vector3d dInertiaScale;

  // we can compute better estimates of inertia values if the link only has
  // one collision made up of a simple shape
  // otherwise assume box geom
  bool boxInertia = false;
  if (this->collisions.size() == 1u)
  {
    auto const &it = this->collisions.begin();
    std::string geomStr = it->first->GetGeometryType();
    dInertiaScale = colNewSizes[it->first->GetName()] /
        colOldSizes[it->first->GetName()];
    if (geomStr == "sphere")
    {
      // solve for r^2
      double r2 = ixx / (oldMass * 0.4);

      // compute new inertia values based on new mass and radius
      newIxx = newMass * 0.4 * (dInertiaScale.X() * dInertiaScale.X()) * r2;
      newIyy = newIxx;
      newIzz = newIxx;
    }
    else if (geomStr == "cylinder")
    {
      // solve for r^2 and l^2
      double r2 = izz / (oldMass * 0.5);
      double l2 = (ixx / oldMass - 0.25 * r2) * 12.0;

      // compute new inertia values based on new mass, radius and length
      newIxx = newMass * (0.25 * (dInertiaScale.X() * dInertiaScale.X() * r2) +
          (dInertiaScale.Z() * dInertiaScale.Z() * l2) / 12.0);
      newIyy = newIxx;
      newIzz = newMass * 0.5 * (dInertiaScale.X() * dInertiaScale.X() * r2);
    }
    else
    {
      boxInertia = true;
    }
  }
  else
  {
    boxInertia = true;
  }

  if (boxInertia)
  {
    // solve for box inertia size: dx^2, dy^2, dz^2,
    // assuming solid box with uniform density
    double mc = 12.0 / oldMass;
    double ixxMc = ixx * mc;
    double iyyMc = iyy * mc;
    double izzMc = izz * mc;
    double dz2 = (iyyMc - izzMc + ixxMc) * 0.5;
    double dx2 = izzMc - (ixxMc - dz2);
    double dy2 = ixxMc - dz2;

    // scale inertia size
    double newDx2 = dInertiaScale.X() * dInertiaScale.X() * dx2;
    double newDy2 = dInertiaScale.Y() * dInertiaScale.Y() * dy2;
    double newDz2 = dInertiaScale.Z() * dInertiaScale.Z() * dz2;

    // compute new inertia values based on new inertia size
    double newMassConstant = newMass / 12.0;
    newIxx = newMassConstant * (newDy2 + newDz2);
    newIyy = newMassConstant * (newDx2 + newDz2);
    newIzz = newMassConstant * (newDx2 + newDy2);
  }

  // update inspector inertia
  linkConfig->SetInertiaMatrix(newIxx, newIyy, newIzz, 0, 0, 0);

  // update local inertal variables
  this->inertiaIxx = newIxx;
  this->inertiaIyy = newIyy;
  this->inertiaIzz = newIzz;

  // update sdf
  sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
  sdf::ElementPtr ixxElem = inertiaElem->GetElement("ixx");
  sdf::ElementPtr iyyElem = inertiaElem->GetElement("iyy");
  sdf::ElementPtr izzElem = inertiaElem->GetElement("izz");
  ixxElem->Set(newIxx);
  iyyElem->Set(newIyy);
  izzElem->Set(newIzz);

  sdf::ElementPtr massElem = inertialElem->GetElement("mass");
  massElem->Set(newMass);

  sdf::ElementPtr inertialPoseElem = inertialElem->GetElement("pose");
  ignition::math::Pose3d newPose =
      inertialPoseElem->Get<ignition::math::Pose3d>();
  newPose.Pos() *= dScale;

  inertialPoseElem->Set(newPose);
  linkConfig->SetInertialPose(newPose);

  this->scale = _scale;
}

/////////////////////////////////////////////////
ignition::math::Vector3d LinkData::Scale() const
{
  return this->scale;
}

/////////////////////////////////////////////////
void LinkData::Load(sdf::ElementPtr _sdf)
{
  LinkConfig *linkConfig = this->inspector->GetLinkConfig();

  this->SetName(_sdf->Get<std::string>("name"));
  this->SetPose(_sdf->Get<ignition::math::Pose3d>("pose"));

  msgs::LinkPtr linkMsgPtr(new msgs::Link);
  if (_sdf->HasElement("inertial"))
  {
    sdf::ElementPtr inertialElem = _sdf->GetElement("inertial");
    this->linkSDF->GetElement("inertial")->Copy(inertialElem);

    msgs::Inertial *inertialMsg = linkMsgPtr->mutable_inertial();

    if (inertialElem->HasElement("mass"))
    {
      this->mass = inertialElem->Get<double>("mass");
      inertialMsg->set_mass(this->mass);
    }

    if (inertialElem->HasElement("pose"))
    {
      ignition::math::Pose3d inertialPose =
        inertialElem->Get<ignition::math::Pose3d>("pose");
      msgs::Set(inertialMsg->mutable_pose(), inertialPose);
    }

    if (inertialElem->HasElement("inertia"))
    {
      sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
      this->inertiaIxx = inertiaElem->Get<double>("ixx");
      this->inertiaIyy = inertiaElem->Get<double>("iyy");
      this->inertiaIzz = inertiaElem->Get<double>("izz");
      inertialMsg->set_ixx(this->inertiaIxx);
      inertialMsg->set_iyy(this->inertiaIyy);
      inertialMsg->set_izz(this->inertiaIzz);
      inertialMsg->set_ixy(inertiaElem->Get<double>("ixy"));
      inertialMsg->set_ixz(inertiaElem->Get<double>("ixz"));
      inertialMsg->set_iyz(inertiaElem->Get<double>("iyz"));
    }
  }
  if (_sdf->HasElement("self_collide"))
  {
    sdf::ElementPtr selfCollideSDF = _sdf->GetElement("self_collide");
    linkMsgPtr->set_self_collide(selfCollideSDF->Get<bool>(""));
    this->linkSDF->InsertElement(selfCollideSDF->Clone());
  }
  if (_sdf->HasElement("kinematic"))
  {
    sdf::ElementPtr kinematicSDF = _sdf->GetElement("kinematic");
    linkMsgPtr->set_kinematic(kinematicSDF->Get<bool>());
    this->linkSDF->InsertElement(kinematicSDF->Clone());
  }
  if (_sdf->HasElement("must_be_base_link"))
  {
    sdf::ElementPtr baseLinkSDF = _sdf->GetElement("must_be_base_link");
    // TODO link.proto is missing the must_be_base_link field.
    // linkMsgPtr->set_must_be_base_link(baseLinkSDF->Get<bool>());
    this->linkSDF->InsertElement(baseLinkSDF->Clone());
  }
  if (_sdf->HasElement("velocity_decay"))
  {
    sdf::ElementPtr velocityDecaySDF = _sdf->GetElement("velocity_decay");
    // TODO link.proto is missing the velocity_decay field.
    // linkMsgPtr->set_velocity_decay(velocityDecaySDF->Get<double>());
    this->linkSDF->InsertElement(velocityDecaySDF->Clone());
  }
  linkConfig->Update(linkMsgPtr);

  if (_sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = _sdf->GetElement("sensor");
    while (sensorElem)
    {
      this->linkSDF->InsertElement(sensorElem->Clone());
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }
}

/////////////////////////////////////////////////
void LinkData::AddVisual(rendering::VisualPtr _visual)
{
  VisualConfig *visualConfig = this->inspector->GetVisualConfig();
  msgs::Visual visualMsg = msgs::VisualFromSDF(_visual->GetSDF());

  this->visuals[_visual] = visualMsg;

  std::string visName = _visual->GetName();
  std::string leafName = visName;
  size_t idx = visName.rfind("::");
  if (idx != std::string::npos)
    leafName = visName.substr(idx+2);

  visualConfig->AddVisual(leafName, &visualMsg);
}

/////////////////////////////////////////////////
void LinkData::AddCollision(rendering::VisualPtr _collisionVis,
    const msgs::Collision *_msg)
{
  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();

  sdf::ElementPtr collisionSDF(new sdf::Element);
  sdf::initFile("collision.sdf", collisionSDF);

  std::string visName = _collisionVis->GetName();
  std::string leafName = visName;
  size_t idx = visName.rfind("::");
  if (idx != std::string::npos)
    leafName = visName.substr(idx+2);

  msgs::Collision collisionMsg;
  // Use input message
  if (_msg)
  {
    collisionMsg = *_msg;
  }
  // Get data from input visual
  else
  {
    msgs::Visual visualMsg = msgs::VisualFromSDF(_collisionVis->GetSDF());
    collisionMsg.set_name(leafName);
    msgs::Geometry *geomMsg = collisionMsg.mutable_geometry();
    geomMsg->CopyFrom(visualMsg.geometry());
    msgs::Pose *poseMsg = collisionMsg.mutable_pose();
    poseMsg->CopyFrom(visualMsg.pose());
  }

  this->collisions[_collisionVis] = collisionMsg;
  collisionConfig->AddCollision(leafName, &collisionMsg);
}

/////////////////////////////////////////////////
LinkData *LinkData::Clone(const std::string &_newName)
{
  GZ_ASSERT(this->linkVisual, "LinkVisual is NULL");
  LinkData *cloneLink = new LinkData();

  cloneLink->Load(this->linkSDF);
  cloneLink->SetName(_newName);

  std::string linkVisualName = this->linkVisual->GetName();
  std::string cloneVisName = _newName;
  size_t linkIdx = linkVisualName.find("::");
  if (linkIdx != std::string::npos)
    cloneVisName = linkVisualName.substr(0, linkIdx+2) + _newName;

  // clone linkVisual;
  rendering::VisualPtr linkVis(new rendering::Visual(cloneVisName,
      this->linkVisual->GetParent()));
  linkVis->Load();

  cloneLink->linkVisual = linkVis;

  for (auto &visIt : this->visuals)
  {
    std::string newVisName = visIt.first->GetName();
    size_t idx = newVisName.rfind("::");
    std::string leafName = newVisName.substr(idx+2);
    if (idx != std::string::npos)
      newVisName = cloneVisName + "::" + leafName;
    else
      newVisName = cloneVisName + "::" + newVisName;

    rendering::VisualPtr cloneVis =
        visIt.first->Clone(newVisName, cloneLink->linkVisual);

    // store the leaf name in sdf not the full scoped name
    cloneVis->GetSDF()->GetAttribute("name")->Set(leafName);

    // override transparency
    cloneVis->SetTransparency(visIt.second.transparency());
    cloneLink->AddVisual(cloneVis);
    cloneVis->SetTransparency(visIt.second.transparency() *
        (1-ModelData::GetEditTransparency()-0.1)
        + ModelData::GetEditTransparency());
  }

  for (auto &colIt : this->collisions)
  {
    std::string newColName = colIt.first->GetName();
    size_t idx = newColName.rfind("::");
    std::string leafName = newColName.substr(idx+2);
    if (idx != std::string::npos)
      newColName = cloneVisName + "::" + leafName;
    else
      newColName = cloneVisName + "::" + newColName;
    rendering::VisualPtr collisionVis = colIt.first->Clone(newColName,
        cloneLink->linkVisual);

    // store the leaf name in sdf not the full scoped name
    collisionVis->GetSDF()->GetAttribute("name")->Set(leafName);

    collisionVis->SetTransparency(
       ignition::math::clamp(ModelData::GetEditTransparency() * 2.0, 0.0, 0.8));
    ModelData::UpdateRenderGroup(collisionVis);
    cloneLink->AddCollision(collisionVis);
  }
  return cloneLink;
}

/////////////////////////////////////////////////
double LinkData::ComputeVolume(const msgs::Collision &_collision)
{
  double volume = -1;

  if (_collision.has_geometry())
  {
    const msgs::Geometry &geometry = _collision.geometry();
    if (geometry.has_type())
    {
      switch (geometry.type())
      {
        case msgs::Geometry_Type_BOX:
        case msgs::Geometry_Type_MESH:
        case msgs::Geometry_Type_POLYLINE:
          if (geometry.has_box())
          {
            const msgs::BoxGeom &box = geometry.box();
            if (box.has_size())
            {
              const msgs::Vector3d &size = box.size();
              volume = IGN_BOX_VOLUME(size.x(), size.y(), size.z());
            }
          }
          break;

        case msgs::Geometry_Type_CYLINDER:
          if (geometry.has_cylinder())
          {
            const msgs::CylinderGeom &cylinder = geometry.cylinder();
            if (cylinder.has_radius() && cylinder.has_length())
            {
              // Cylinder volume: PI * r^2 * height
              volume = IGN_CYLINDER_VOLUME(cylinder.radius(),
                                           cylinder.length());
            }
          }
          break;

        case msgs::Geometry_Type_SPHERE:
          if (geometry.has_sphere())
          {
            const msgs::SphereGeom &sphere = geometry.sphere();
            if (sphere.has_radius())
            {
              // Sphere volume: 4/3 * PI * r^3
              volume = IGN_SPHERE_VOLUME(sphere.radius());
            }
          }
          break;

        default:
          break;
      }
    }
  }
  return volume;
}

/////////////////////////////////////////////////
ignition::math::Vector3d LinkData::ComputeMomentOfInertia(
    const msgs::Collision &_collision, const double _mass)
{
  ignition::math::Vector3d result;
  result.Set(0, 0, 0);

  if (_collision.has_geometry())
  {
    const msgs::Geometry &geometry = _collision.geometry();
    if (geometry.has_type())
    {
      switch (geometry.type())
      {
        case msgs::Geometry_Type_BOX:
        case msgs::Geometry_Type_MESH:
        case msgs::Geometry_Type_POLYLINE:
          if (geometry.has_box())
          {
            const msgs::BoxGeom &box = geometry.box();
            if (box.has_size())
            {
              // Box:
              //    Ih = 1/12 * M * (w^2 + d^2)
              //    Iw = 1/12 * M * (h^2 + d^2)
              //    Id = 1/12 * M * (h^2 + w^2)
              double h = box.size().x();
              double h2 = h*h;
              double w = box.size().y();
              double w2 = w*w;
              double d = box.size().z();
              double d2 = d*d;

              double Ih = 1.0 / 12.0 * _mass * (w2 + d2);
              double Iw = 1.0 / 12.0 * _mass * (h2 + d2);
              double Id = 1.0 / 12.0 * _mass * (h2 + w2);

              result.Set(Ih, Iw, Id);
            }
          }
          break;

        case msgs::Geometry_Type_CYLINDER:
          if (geometry.has_cylinder())
          {
            const msgs::CylinderGeom &cylinder = geometry.cylinder();
            if (cylinder.has_radius() && cylinder.has_length())
            {
              // Cylinder:
              //    central axis: I = 1/2 * M * R^2
              //    other axes:   I = 1/4 * M * R^2 + 1/12 * M * L^2
              double r = cylinder.radius();
              double r2 = r*r;
              double l = cylinder.length();
              double l2 = l*l;
              double Icentral = 1.0 / 2.0 * _mass * r2;
              double Iother = (1.0 / 4.0 * _mass * r2) +
                  (1.0 / 12.0 * _mass * l2);
              result.Set(Iother, Iother, Icentral);
            }
          }
          break;

        case msgs::Geometry_Type_SPHERE:
          if (geometry.has_sphere())
          {
            const msgs::SphereGeom &sphere = geometry.sphere();
            if (sphere.has_radius())
            {
              // Sphere: I = 2/5 * M * R^2
              double r = sphere.radius();
              double r2 = r*r;
              double I = 2.0 / 5.0 * _mass * r2;
              result.Set(I, I, I);
            }
          }
          break;

        default:
          break;
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////
double LinkData::ComputeVolume() const
{
  CollisionConfig *collisionConfig = this->inspector->GetCollisionConfig();
  double volume = 0;

  GZ_ASSERT(this->linkVisual, "LinkVisual is NULL");
  GZ_ASSERT(collisionConfig, "CollisionConfig is NULL");

  for (auto const &it : this->collisions)
  {
    std::string name = it.first->GetName();
    std::string linkName = this->linkVisual->GetName();

    std::string leafName = name.substr(name.find(linkName)+linkName.size()+2);
    std::string shape = it.first->GetGeometryType();

    ignition::math::Vector3d size;
    std::string uri;
    collisionConfig->Geometry(leafName,  size, uri);

    if (shape == "sphere")
    {
      // Sphere volume: 4/3 * PI * r^3
      volume += IGN_SPHERE_VOLUME(size.X() * 0.5);
    }
    else if (shape == "cylinder")
    {
      // Cylinder volume: PI * r^2 * height
      volume += IGN_CYLINDER_VOLUME(size.X() * 0.5, size.Z());
    }
    else
    {
      // Box, mesh, and other geometry types - use bounding box
      volume += IGN_BOX_VOLUME_V(size);
    }
  }
  return volume;
}

/////////////////////////////////////////////////
void LinkData::SetLinkVisual(const rendering::VisualPtr _visual)
{
  this->linkVisual = _visual;
}

/////////////////////////////////////////////////
rendering::VisualPtr LinkData::LinkVisual() const
{
  return this->linkVisual;
}
