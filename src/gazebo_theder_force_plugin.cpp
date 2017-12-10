// created form Jonas

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo_theder_force_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ThederForcePlugin)

/////////////////////////////////////////////////
ThederForcePlugin::ThederForcePlugin()
{
  this->i=0;
  this->ropeLength = 100;
  this->forceConstantA = 6;
  this->forceConstantB = 5;
  this->forceConstant_a = 35;
  this->dragConst = 0.002347995;
}

ThederForcePlugin::~ThederForcePlugin()
{
}

void ThederForcePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  GZ_ASSERT(_model, "ThederForcePlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "ThederForcePlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;


  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
//    GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
//    std::cout << linkName << "\n";
//    GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The ThederForcePlugin will not generate forces\n";

    }
    else
    {
      link_ = this->link;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ThederForcePlugin::OnUpdate, this, _1));
    }
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
//  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//    boost::bind(&ModelPush::OnUpdate, this, _1));
}

// Called by the world update start event
void ThederForcePlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  position = this->model->GetWorldPose();
  distance = position.pos.GetLength();
  math::Vector3 dragForce = -(this->model->GetRelativeLinearVel())*dragConst*distance;
  thederForce = forceConstantA*exp(forceConstantB*distance/ropeLength);
//  if(distance < ropeLength)
//  thederForce = forceConstant_a/(-distance+ropeLength);
//  else thederForce = 100000000;

//  thederForce = 0;

  math::Vector3 velocity = this->model->GetRelativeLinearVel();
  math::Vector3 normalizedPosition = position.pos.Normalize();
// calculate the speed tangential to the distance to the groundstation:
  double speed = velocity.x*normalizedPosition.x+velocity.y+normalizedPosition.y+velocity.z+normalizedPosition.z;
// calculate Magnitude of drag force:
  dragForce = (1/4)*dragConst*speed*speed*distance;
//  dragForce = 0;
//  add forces to the link:
  link_->AddForce(position.pos.Normalize().operator*(-thederForce));
  link_->AddForce(this->model->GetRelativeLinearVel().Normalize().operator*(-dragForce));


  // output current informations of the model
  if(i>100)
  {
    std::cout << position.pos.Normalize().operator*(-thederForce) << "\t " << thederForce << "\t " << distance  << "\n";
    i=0;
  }
  i++;
}
