// created form Jonas

#ifndef _GAZEBO_THEDER_FORCE_PLUGIN_HH_
#define _GAZEBO_THEDER_FORCE_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <math.h>
#include <stdio.h>

#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE ThederForcePlugin : public ModelPlugin
  {
  protected: int i; // Countervariable
  protected: double ropeLength; //Length of the rope in meters
  protected: double forceConstantA; // F = A*exp(B*distance/ropeLength)
  protected: double forceConstantB;
  protected: double dampingConstant;
  protected: double dragConst; // = 1/2*rhoLuft*diameter*cDragRope = 0.5*1.2041*0.003*1.3 = 0.002347995
  protected: double eModule;
  protected: physics::LinkPtr link_;

  // brief Constructor
  public: ThederForcePlugin();

  // brief Destructor
  public: ~ThederForcePlugin();

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/);

  // Pointer to the model
  private: physics::ModelPtr model;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  /// \brief Pointer to link currently targeted by mud joint.
  protected: physics::LinkPtr link;

  /// \brief SDF for this plugin;
  protected: sdf::ElementPtr sdf;
  };
}
#endif
