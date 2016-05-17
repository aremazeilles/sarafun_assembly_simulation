#include "base/admittance_controller_base.h"

#include <ros/ros.h>

namespace gazebo
{
  
class VerticalForceAdmittanceController : public AdmittanceControllerBase
{

  virtual void admittanceControllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf )
  {
 
    if( !sdf->HasElement("target_force") )
    {
      ROS_WARN("Plugin missing <target_force>, using default (-40N)");
      target_force_ = -40;
    }
    else
    {
      target_force_ = sdf->GetElement("target_force")->Get<double>();
    }

    time_initialised_ = false;
    integral_force_error_ = 0.0;

  }

  virtual void admittanceControllerUpdate( const common::UpdateInfo info,
                                           const math::Vector3& receptacle_force,
                                           const math::Vector3& receptacle_torque,
                                           const math::Vector3& slider_force,
                                           const math::Vector3& slider_torque,
                                           math::Vector3* slider_lin_vel,
                                           math::Vector3* slider_rot_vel )
  {

    if( !time_initialised_ )
    {
      previous_time_ = info.simTime;
      time_initialised_ = true;
    }

    double error = target_force_ - receptacle_force.z;

    integral_force_error_ += (info.simTime - previous_time_).Double() * error;

    math::Pose slider_handle_pose = slider_handle_link_->GetWorldCoGPose();

    *slider_lin_vel = math::Vector3( -0.1*slider_handle_pose.pos.x, 0, 0.1*error + 0.05*integral_force_error_ );

    previous_time_ = info.simTime;

  }

private:

  double target_force_;

  bool time_initialised_;
  common::Time previous_time_;
  double integral_force_error_;

};

GZ_REGISTER_MODEL_PLUGIN( VerticalForceAdmittanceController )

}
