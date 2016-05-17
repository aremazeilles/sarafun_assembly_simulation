#include "controller_base.h"

namespace gazebo
{
  
class AdmittanceControllerBase : public ControllerBase
{
  
  void controllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf )
  {

    nh_ = new ros::NodeHandle;

    ros::AdvertiseOptions filtered_receptacle_wrench_ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>( "filtered_receptacle_wrench",
                                                                                                                       1,
                                                                                                                       boost::bind( &ControllerBase::connectCB, this ),
                                                                                                                       boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                                       ros::VoidPtr(),
                                                                                                                       &queue_ );
    filtered_receptacle_wrench_pub_ = nh_->advertise( filtered_receptacle_wrench_ao );

    ros::AdvertiseOptions filtered_slider_wrench_ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>( "filtered_slider_wrench",
                                                                                                                   1,
                                                                                                                   boost::bind( &ControllerBase::connectCB, this ),
                                                                                                                   boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                                   ros::VoidPtr(),
                                                                                                                   &queue_ );
    filtered_slider_wrench_pub_ = nh_->advertise( filtered_slider_wrench_ao );

    filter_decay_ = 0.9;

    admittanceControllerLoad( model, sdf );

  }

  void controllerUpdate( const common::UpdateInfo info )
  {

    physics::JointWrench receptacle_wrench = receptacle_joint_->GetForceTorque(0);
    physics::JointWrench slider_wrench = slider_joint_->GetForceTorque(0);

    filtered_receptacle_force_ = filtered_receptacle_force_*filter_decay_ + receptacle_wrench.body2Force*(1-filter_decay_);
    filtered_receptacle_torque_ = filtered_receptacle_torque_*filter_decay_ + receptacle_wrench.body2Torque*(1-filter_decay_);
    filtered_slider_force_ = filtered_slider_force_*filter_decay_ + slider_wrench.body2Force*(1-filter_decay_);
    filtered_slider_torque_ = filtered_slider_torque_*filter_decay_ + slider_wrench.body2Torque*(1-filter_decay_);

    math::Vector3 slider_lin_vel, slider_rot_vel;
    admittanceControllerUpdate( info, filtered_receptacle_force_, filtered_receptacle_torque_, filtered_slider_force_, filtered_slider_torque_, &slider_lin_vel, &slider_rot_vel );

    slider_handle_link_->SetLinearVel( slider_lin_vel );
    slider_handle_link_->SetAngularVel( slider_rot_vel );

    geometry_msgs::WrenchStamped filtered_receptacle_wrench_msg;
    filtered_receptacle_wrench_msg.header.frame_id = "receptacle";
    filtered_receptacle_wrench_msg.header.stamp.sec = world_->GetSimTime().sec;
    filtered_receptacle_wrench_msg.header.stamp.nsec = world_->GetSimTime().nsec;
    filtered_receptacle_wrench_msg.wrench.force.x = filtered_receptacle_force_.x;
    filtered_receptacle_wrench_msg.wrench.force.y = filtered_receptacle_force_.y;
    filtered_receptacle_wrench_msg.wrench.force.z = filtered_receptacle_force_.z;
    filtered_receptacle_wrench_msg.wrench.torque.x = filtered_receptacle_torque_.x;
    filtered_receptacle_wrench_msg.wrench.torque.y = filtered_receptacle_torque_.y;
    filtered_receptacle_wrench_msg.wrench.torque.z = filtered_receptacle_torque_.z;

    geometry_msgs::WrenchStamped filtered_slider_wrench_msg;
    filtered_slider_wrench_msg.header.frame_id = "slider";
    filtered_slider_wrench_msg.header.stamp.sec = world_->GetSimTime().sec;
    filtered_slider_wrench_msg.header.stamp.nsec = world_->GetSimTime().nsec;
    filtered_slider_wrench_msg.wrench.force.x = filtered_slider_force_.x;
    filtered_slider_wrench_msg.wrench.force.y = filtered_slider_force_.y;
    filtered_slider_wrench_msg.wrench.force.z = filtered_slider_force_.z;
    filtered_slider_wrench_msg.wrench.torque.x = filtered_slider_torque_.x;
    filtered_slider_wrench_msg.wrench.torque.y = filtered_slider_torque_.y;
    filtered_slider_wrench_msg.wrench.torque.z = filtered_slider_torque_.z;

    filtered_receptacle_wrench_pub_.publish( filtered_receptacle_wrench_msg );
    filtered_slider_wrench_pub_.publish( filtered_slider_wrench_msg );

  }

  virtual void admittanceControllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf ) {}

  virtual void admittanceControllerUpdate( const common::UpdateInfo info,
                                           const math::Vector3& receptacle_force,
                                           const math::Vector3& receptacle_torque,
                                           const math::Vector3& slider_force,
                                           const math::Vector3& slider_torque,
                                           math::Vector3* slider_lin_vel,
                                           math::Vector3* slider_rot_vel ) = 0;

private:

  ros::NodeHandle* nh_;
  ros::Publisher filtered_receptacle_wrench_pub_;
  ros::Publisher filtered_slider_wrench_pub_;

  math::Vector3 filtered_receptacle_force_;
  math::Vector3 filtered_receptacle_torque_;
  math::Vector3 filtered_slider_force_;
  math::Vector3 filtered_slider_torque_;

  double filter_decay_;

};

}
