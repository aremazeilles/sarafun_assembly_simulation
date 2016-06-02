#include "base/admittance_controller_base.h"

#include <ros/ros.h>

namespace gazebo
{


namespace PlanarFoldingPhases
{
enum PlanarFoldingPhase
{
  SEARCHING_CONTACT,
  SLIDING,
  FOLDING
};
}
typedef PlanarFoldingPhases::PlanarFoldingPhase PlanarFoldingPhase;


class PlanarFoldingAdmittanceController : public AdmittanceControllerBase
{

  virtual void admittanceControllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf );

  virtual void admittanceControllerUpdate( const common::UpdateInfo info,
                                           const math::Vector3& receptacle_force,
                                           const math::Vector3& receptacle_torque,
                                           const math::Vector3& slider_force,
                                           const math::Vector3& slider_torque,
                                           math::Vector3* slider_lin_vel,
                                           math::Vector3* slider_rot_vel );

  void doSearchingContactUpdate( const common::UpdateInfo info,
                                 const math::Vector3& receptacle_force,
                                 const math::Vector3& receptacle_torque,
                                 const math::Vector3& slider_force,
                                 const math::Vector3& slider_torque,
                                 math::Vector3* slider_lin_vel,
                                 math::Vector3* slider_rot_vel );

  void doSlidingUpdate( const common::UpdateInfo info,
                        const math::Vector3& receptacle_force,
                        const math::Vector3& receptacle_torque,
                        const math::Vector3& slider_force,
                        const math::Vector3& slider_torque,
                        math::Vector3* slider_lin_vel,
                        math::Vector3* slider_rot_vel );

  void doFoldingUpdate( const common::UpdateInfo info,
                        const math::Vector3& receptacle_force,
                        const math::Vector3& receptacle_torque,
                        const math::Vector3& slider_force,
                        const math::Vector3& slider_torque,
                        math::Vector3* slider_lin_vel,
                        math::Vector3* slider_rot_vel );

private:

  ros::NodeHandle* nh_;

  ros::Publisher contact_point_pose_pub_;

  double target_sliding_angle_;
  double target_sliding_vel_;
  double target_folding_angle_;
  double target_normal_force_;
  double target_tangential_force_;
  double normal_force_p_gain_;
  double normal_force_i_gain_;
  double tangential_force_p_gain_;
  double tangential_force_i_gain_;
  double rot_gain_;
  double switch_to_fold_threshold_;
  double switch_to_slide_threshold_;

  bool time_initialised_;
  common::Time previous_time_;
  double integral_force_error_;

  math::Pose slider_cog_T_virtual_contact_point_;

  PlanarFoldingPhase phase_;

};


void PlanarFoldingAdmittanceController::admittanceControllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf )
{

  if( !sdf->HasElement("target_sliding_angle") )
  {
    ROS_WARN("Plugin missing <target_sliding_angle>, using default (1 rad)");
    target_sliding_angle_ = 1.0;
  }
  else
  {
    target_sliding_angle_ = sdf->GetElement("target_sliding_angle")->Get<double>();
  }

  if( !sdf->HasElement("target_sliding_vel") )
  {
    ROS_WARN("Plugin missing <target_sliding_vel>, using default (0.005 m/s)");
    target_sliding_vel_ = 0.005;
  }
  else
  {
    target_sliding_vel_ = sdf->GetElement("target_sliding_vel")->Get<double>();
  }

  if( !sdf->HasElement("target_folding_angle") )
  {
    ROS_WARN("Plugin missing <target_folding_angle>, using default (1 rad)");
    target_folding_angle_ = 1.0;
  }
  else
  {
    target_folding_angle_ = sdf->GetElement("target_folding_angle")->Get<double>();
  }

  if( !sdf->HasElement("target_normal_force") )
  {
    ROS_WARN("Plugin missing <target_normal_force>, using default (10 N)");
    target_normal_force_ = 10.0;
  }
  else
  {
    target_normal_force_ = sdf->GetElement("target_normal_force")->Get<double>();
  }

  if( !sdf->HasElement("target_tangential_force") )
  {
    ROS_WARN("Plugin missing <target_tangential_force>, using default (10 N)");
    target_tangential_force_ = 10.0;
  }
  else
  {
    target_tangential_force_ = sdf->GetElement("target_tangential_force")->Get<double>();
  }

  if( !sdf->HasElement("normal_force_p_gain") )
  {
    ROS_WARN("Plugin missing <normal_force_p_gain>, using default (0.1)");
    normal_force_p_gain_ = 0.1;
  }
  else
  {
    normal_force_p_gain_ = sdf->GetElement("normal_force_p_gain")->Get<double>();
  }

  if( !sdf->HasElement("normal_force_i_gain") )
  {
    ROS_WARN("Plugin missing <normal_force_i_gain>, using default (0.05)");
    normal_force_i_gain_ = 0.05;
  }
  else
  {
    normal_force_i_gain_ = sdf->GetElement("normal_force_i_gain")->Get<double>();
  }

  if( !sdf->HasElement("tangential_force_p_gain") )
  {
    ROS_WARN("Plugin missing <tangential_force_p_gain>, using default (0.1)");
    tangential_force_p_gain_ = 0.1;
  }
  else
  {
    tangential_force_p_gain_ = sdf->GetElement("tangential_force_p_gain")->Get<double>();
  }

  if( !sdf->HasElement("tangential_force_i_gain") )
  {
    ROS_WARN("Plugin missing <tangential_force_i_gain>, using default (0.05)");
    tangential_force_i_gain_ = 0.05;
  }
  else
  {
    tangential_force_i_gain_ = sdf->GetElement("tangential_force_i_gain")->Get<double>();
  }

  if( !sdf->HasElement("rot_gain") )
  {
    ROS_WARN("Plugin missing <rot_gain>, using default (0.05)");
    rot_gain_ = 0.05;
  }
  else
  {
    rot_gain_ = sdf->GetElement("rot_gain")->Get<double>();
  }

  if( !sdf->HasElement("switch_to_fold_threshold") )
  {
    ROS_WARN("Plugin missing <switch_to_fold_threshold>, using default (0.5 N)");
    switch_to_fold_threshold_ = 0.5;
  }
  else
  {
    switch_to_fold_threshold_ = sdf->GetElement("switch_to_fold_threshold")->Get<double>();
  }

  if( !sdf->HasElement("switch_to_slide_threshold") )
  {
    ROS_WARN("Plugin missing <switch_to_slide_threshold>, using default (0.5 N)");
    switch_to_slide_threshold_ = 0.2;
  }
  else
  {
    switch_to_slide_threshold_ = sdf->GetElement("switch_to_slide_threshold")->Get<double>();
  }

  time_initialised_ = false;
  integral_force_error_ = 0.0;

  nh_ = new ros::NodeHandle;

  ros::AdvertiseOptions contact_point_pose_ao = ros::AdvertiseOptions::create<geometry_msgs::PoseStamped>( "virtual_contact_point_pose",
                                                                                                           1,
                                                                                                           boost::bind( &ControllerBase::connectCB, this ),
                                                                                                           boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                           ros::VoidPtr(),
                                                                                                           &queue_ );
  contact_point_pose_pub_ = nh_->advertise( contact_point_pose_ao );

  slider_cog_T_virtual_contact_point_ = math::Pose( 0, 0, -0.04, 0, 0, 0 );

  phase_ = PlanarFoldingPhases::SEARCHING_CONTACT;

}


void PlanarFoldingAdmittanceController::admittanceControllerUpdate( const common::UpdateInfo info,
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

  doSlidingUpdate( info, receptacle_force, receptacle_torque, slider_force, slider_torque, slider_lin_vel, slider_rot_vel );

  previous_time_ = info.simTime;

  math::Pose virtual_contact_point_pose = slider_cog_T_virtual_contact_point_ + slider_link_->GetWorldCoGPose();

  if( connect_count_ <= 0 )
    return;

  geometry_msgs::PoseStamped virtual_contact_point_pose_msg;
  virtual_contact_point_pose_msg.header.frame_id = "world";
  virtual_contact_point_pose_msg.header.stamp.sec = world_->GetSimTime().sec;
  virtual_contact_point_pose_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  virtual_contact_point_pose_msg.pose.position.x = virtual_contact_point_pose.pos.x;
  virtual_contact_point_pose_msg.pose.position.y = virtual_contact_point_pose.pos.y;
  virtual_contact_point_pose_msg.pose.position.z = virtual_contact_point_pose.pos.z;
  virtual_contact_point_pose_msg.pose.orientation.x = virtual_contact_point_pose.rot.x;
  virtual_contact_point_pose_msg.pose.orientation.y = virtual_contact_point_pose.rot.y;
  virtual_contact_point_pose_msg.pose.orientation.z = virtual_contact_point_pose.rot.z;
  virtual_contact_point_pose_msg.pose.orientation.w = virtual_contact_point_pose.rot.w;

  contact_point_pose_pub_.publish( virtual_contact_point_pose_msg );

}


void PlanarFoldingAdmittanceController::doSearchingContactUpdate( const common::UpdateInfo info,
                                                                  const math::Vector3& receptacle_force,
                                                                  const math::Vector3& receptacle_torque,
                                                                  const math::Vector3& slider_force,
                                                                  const math::Vector3& slider_torque,
                                                                  math::Vector3* slider_lin_vel,
                                                                  math::Vector3* slider_rot_vel )
{

}

void PlanarFoldingAdmittanceController::doSlidingUpdate( const common::UpdateInfo info,
                                                         const math::Vector3& receptacle_force,
                                                         const math::Vector3& receptacle_torque,
                                                         const math::Vector3& slider_force,
                                                         const math::Vector3& slider_torque,
                                                         math::Vector3* slider_lin_vel,
                                                         math::Vector3* slider_rot_vel )
{

  math::Vector3 tangent( 1, 0, 0 );
  math::Vector3 slider_z = slider_link_->GetWorldCoGPose().rot.GetZAxis();

  double contact_angle = acos( slider_z.Normalize().Dot(tangent.Normalize()) );
  double contact_angle_error = contact_angle - target_sliding_angle_;

  math::Vector3 contact_point_des_rot_vel( 0, rot_gain_*contact_angle_error, 0 );

  double force_error = -target_normal_force_ - receptacle_force.z;
  integral_force_error_ += (info.simTime - previous_time_).Double() * force_error;

  math::Vector3 normal_force_corrective_vel( 0, 0, normal_force_p_gain_*force_error + normal_force_i_gain_*integral_force_error_ );

  math::Vector3 contact_point_des_lin_vel = math::Vector3( -target_sliding_vel_, 0, 0 ) + normal_force_corrective_vel;

  math::Vector3 rot_corr_handle_lin_vel = -contact_point_des_rot_vel.Cross( math::Vector3( -cos(contact_angle)*0.04, 0, -sin(contact_angle)*0.04 ) );
  math::Vector3 handle_des_lin_vel = rot_corr_handle_lin_vel + contact_point_des_lin_vel;

  *slider_lin_vel = handle_des_lin_vel;
  *slider_rot_vel = contact_point_des_rot_vel;

}

void PlanarFoldingAdmittanceController::doFoldingUpdate( const common::UpdateInfo info,
                                                         const math::Vector3& receptacle_force,
                                                         const math::Vector3& receptacle_torque,
                                                         const math::Vector3& slider_force,
                                                         const math::Vector3& slider_torque,
                                                         math::Vector3* slider_lin_vel,
                                                         math::Vector3* slider_rot_vel )
{

}


GZ_REGISTER_MODEL_PLUGIN( PlanarFoldingAdmittanceController )


}
