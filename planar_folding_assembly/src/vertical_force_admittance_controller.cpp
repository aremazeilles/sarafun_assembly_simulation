#include "base/admittance_controller_base.h"

#include <ros/ros.h>

namespace gazebo
{


class VerticalForceAdmittanceController : public AdmittanceControllerBase
{

  virtual void admittanceControllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf );

  virtual void admittanceControllerUpdate( const common::UpdateInfo info,
                                           const math::Vector3& receptacle_force,
                                           const math::Vector3& receptacle_torque,
                                           const math::Vector3& slider_force,
                                           const math::Vector3& slider_torque,
                                           math::Vector3* slider_lin_vel,
                                           math::Vector3* slider_rot_vel );

private:

  double target_force_;
  double p_gain_;
  double i_gain_;
  double vert_correct_gain_;

  bool time_initialised_;
  common::Time previous_time_;
  double integral_force_error_;

};


void VerticalForceAdmittanceController::admittanceControllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf )
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

  if( !sdf->HasElement("p_gain") )
  {
    ROS_WARN("Plugin missing <p_gain>, using default (0.1)");
    p_gain_ = 0.1;
  }
  else
  {
    p_gain_ = sdf->GetElement("p_gain")->Get<double>();
  }

  if( !sdf->HasElement("i_gain") )
  {
    ROS_WARN("Plugin missing <i_gain>, using default (0.05)");
    i_gain_ = 0.05;
  }
  else
  {
    i_gain_ = sdf->GetElement("i_gain")->Get<double>();
  }

  if( !sdf->HasElement("vert_correct_gain") )
  {
    ROS_WARN("Plugin missing <vert_correct_gain>, using default (0.1)");
    vert_correct_gain_ = 0.1;
  }
  else
  {
    vert_correct_gain_ = sdf->GetElement("vert_correct_gain")->Get<double>();
  }

  time_initialised_ = false;
  integral_force_error_ = 0.0;

}


void VerticalForceAdmittanceController::admittanceControllerUpdate( const common::UpdateInfo info,
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

  double error = -target_force_ - receptacle_force.z;

  integral_force_error_ += (info.simTime - previous_time_).Double() * error;

  math::Pose slider_handle_pose = slider_handle_link_->GetWorldCoGPose();

  *slider_lin_vel = math::Vector3( -vert_correct_gain_*slider_handle_pose.pos.x, 0, p_gain_*error + i_gain_*integral_force_error_ );

  previous_time_ = info.simTime;

}


GZ_REGISTER_MODEL_PLUGIN( VerticalForceAdmittanceController )


}
