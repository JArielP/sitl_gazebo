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
  this->dragConst = 0.002347995;
  this->dampingConstant = 3;
  this->eModule = 121000;
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
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);

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
}

// Called by the world update start event
void ThederForcePlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  math::Pose position = this->model->GetWorldPose(); // get the current position of the aircraft
  double distance = position.pos.GetLength(); // Distance between the Plane and the Groundstation
  math::Vector3 velocity = this->model->GetRelativeLinearVel(); // get the velocity of the aircraft
  double speed = velocity.GetLength();
  math::Vector3 normalizedPosition = position.pos.Normalize(); // get direction of the tether

  /* calculate the drag force and damping force */
  // variables:
  double tetherForce = 0;
  double damping = 0;

//  if(distance >= ropeLength)
  {
    tetherForce = eModule*3.14*4*(distance-ropeLength)/ropeLength+20;
    damping = 0; // dampingConstant*speed;
  }
//  else if (distance > 0.8*ropeLength)
  {
    tetherForce = forceConstantA*exp(forceConstantB*(distance-0.8*ropeLength)/ropeLength);
    damping = 0; // dampingConstant*(distance-0.8*ropeLength)/(0.2*ropeLength)*speed;
  }

  /* calculate the dragforce */
  // variables:
  double dragForce = 0;

  // calculate the speed perpendicular to the tether:
  math::Vector3 perpendicularToTether = (normalizedPosition.Cross(velocity.Cross(normalizedPosition))).Normalize();
  double speedPerpendicularToTether = velocity.x*perpendicularToTether.x+velocity.y*perpendicularToTether.y+velocity.z*perpendicularToTether.z;
  // calculate Magnitude of drag force:
  dragForce = (1/4)*dragConst*speedPerpendicularToTether*speedPerpendicularToTether*distance;

  /* add forces to the link */
  if(tetherForce > 0)
  {
    // add tether force:
    link_->AddForce(normalizedPosition*(-tetherForce));
    // add drag force
    link_->AddForce(velocity.Normalize().operator*(-dragForce));
    // add damping force:
    if (damping > 50)
      damping = 50;
      link_->AddForce(this->model->GetWorldLinearVel().Normalize()*(-damping));
  }

  // output current informations of the model
  if(i>100)
  {
    std::cout << position.pos.Normalize().operator*(-tetherForce) << "\t " << tetherForce << "\t " << distance  << "\n";
    i=0;
  }
  i++;
}
