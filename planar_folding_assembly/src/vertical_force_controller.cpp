#include "base/controller_base.h"

#include <ros/ros.h>

namespace gazebo
{


class VerticalForceController : public ControllerBase
{

  virtual void controllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf );

  virtual void controllerUpdate( const common::UpdateInfo info );

private:

  double target_force_;

  bool time_initialised_;
  common::Time previous_time_;
  double integral_force_error_;

};


void VerticalForceController::controllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf )
{

  if( !sdf->HasElement("target_force") )
  {
    ROS_WARN("Plugin missing <target_force>, using default (10 N)");
    target_force_ = 10;
  }
  else
  {
    target_force_ = sdf->GetElement("target_force")->Get<double>();
  }

  time_initialised_ = false;
  integral_force_error_ = 0.0;

}

void VerticalForceController::controllerUpdate( const common::UpdateInfo info )
{

  physics::JointWrench receptacle_wrench = receptacle_joint_->GetForceTorque(0);

  if( !time_initialised_ )
  {
    previous_time_ = info.simTime;
    time_initialised_ = true;
  }

  double error = -target_force_ - receptacle_wrench.body2Force.z;

  integral_force_error_ += (info.simTime - previous_time_).Double() * error;

  math::Pose slider_handle_pose = slider_handle_link_->GetWorldCoGPose();

  slider_handle_link_->SetLinearVel( math::Vector3( -0.1*slider_handle_pose.pos.x, 0, 0.1*error + 0.05*integral_force_error_ ) );

  previous_time_ = info.simTime;

}



GZ_REGISTER_MODEL_PLUGIN( VerticalForceController )


}
