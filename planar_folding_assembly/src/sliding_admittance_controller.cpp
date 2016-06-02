#include "base/admittance_controller_base.h"

#include <ros/ros.h>

namespace gazebo
{


class SlidingAdmittanceController : public AdmittanceControllerBase
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

  ros::NodeHandle* nh_;

  ros::Publisher contact_point_pose_pub_;

  double target_angle_;
  double target_vel_;
  double target_force_;
  double p_gain_;
  double i_gain_;
  double rot_gain_;
  double vert_correct_gain_;

  bool time_initialised_;
  common::Time previous_time_;
  double integral_force_error_;

  math::Pose slider_cog_T_virtual_contact_point_;

};


void SlidingAdmittanceController::admittanceControllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf )
{

  if( !sdf->HasElement("target_angle") )
  {
    ROS_WARN("Plugin missing <target_angle>, using default (1 rad)");
    target_angle_ = 1.0;
  }
  else
  {
    target_angle_ = sdf->GetElement("target_angle")->Get<double>();
  }

  if( !sdf->HasElement("target_vel") )
  {
    ROS_WARN("Plugin missing <target_vel>, using default (0.005 m/s)");
    target_vel_ = 0.005;
  }
  else
  {
    target_vel_ = sdf->GetElement("target_vel")->Get<double>();
  }

  if( !sdf->HasElement("target_force") )
  {
    ROS_WARN("Plugin missing <target_force>, using default (10 N)");
    target_force_ = 10.0;
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

  if( !sdf->HasElement("rot_gain") )
  {
    ROS_WARN("Plugin missing <rot_gain>, using default (0.05)");
    rot_gain_ = 0.05;
  }
  else
  {
    rot_gain_ = sdf->GetElement("rot_gain")->Get<double>();
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

  nh_ = new ros::NodeHandle;

  ros::AdvertiseOptions contact_point_pose_ao = ros::AdvertiseOptions::create<geometry_msgs::PoseStamped>( "virtual_contact_point_pose",
                                                                                                           1,
                                                                                                           boost::bind( &ControllerBase::connectCB, this ),
                                                                                                           boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                           ros::VoidPtr(),
                                                                                                           &queue_ );
  contact_point_pose_pub_ = nh_->advertise( contact_point_pose_ao );

  slider_cog_T_virtual_contact_point_ = math::Pose( 0, 0, -0.04, 0, 0, 0 );

}


void SlidingAdmittanceController::admittanceControllerUpdate( const common::UpdateInfo info,
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

  math::Vector3 tangent( 1, 0, 0 );
  math::Vector3 slider_z = slider_link_->GetWorldCoGPose().rot.GetZAxis();

  double contact_angle = acos( slider_z.Normalize().Dot(tangent.Normalize()) );
  double contact_angle_error = contact_angle - target_angle_;

  math::Vector3 contact_point_des_rot_vel( 0, rot_gain_*contact_angle_error, 0 );

  double force_error = -target_force_ - receptacle_force.z;
  integral_force_error_ += (info.simTime - previous_time_).Double() * force_error;

  math::Vector3 normal_force_corrective_vel( 0, 0, p_gain_*force_error + i_gain_*integral_force_error_ );

  math::Vector3 contact_point_des_lin_vel = math::Vector3( -target_vel_, 0, 0 ) + normal_force_corrective_vel;

  math::Vector3 rot_corr_handle_lin_vel = -contact_point_des_rot_vel.Cross( math::Vector3( -cos(contact_angle)*0.04, 0, -sin(contact_angle)*0.04 ) );
  math::Vector3 handle_des_lin_vel = rot_corr_handle_lin_vel + contact_point_des_lin_vel;

  *slider_lin_vel = handle_des_lin_vel;
  *slider_rot_vel = contact_point_des_rot_vel;

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


GZ_REGISTER_MODEL_PLUGIN( SlidingAdmittanceController )


}
