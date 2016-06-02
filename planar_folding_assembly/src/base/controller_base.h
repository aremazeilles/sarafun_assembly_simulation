#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/bind.hpp>

#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>


namespace gazebo
{


class ControllerBase : public ModelPlugin
{

public:

  ControllerBase();

  virtual ~ControllerBase();

  void Load( physics::ModelPtr model, sdf::ElementPtr sdf );

  void onUpdate( const common::UpdateInfo info );

  virtual void controllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf ) {}

  virtual void controllerUpdate( const common::UpdateInfo info ) {}

  void connectCB();

  void disconnectCB();

private:

  void QueueThread();

  ros::NodeHandle* nh_;

  boost::thread callback_queue_thread_;

  ros::Publisher receptacle_pose_pub_;
  ros::Publisher receptacle_wrench_pub_;
  ros::Publisher slider_pose_pub_;
  ros::Publisher slider_wrench_pub_;

  event::ConnectionPtr updateConnection_;

protected:

  int connect_count_;
  ros::CallbackQueue queue_;
  
  physics::WorldPtr world_;

  physics::ModelPtr slider_model_;
  physics::LinkPtr slider_handle_link_;
  physics::LinkPtr slider_link_;
  physics::JointPtr slider_joint_;

  physics::ModelPtr receptacle_model_;
  physics::LinkPtr receptacle_handle_link_;
  physics::LinkPtr receptacle_link_;
  physics::JointPtr receptacle_joint_;

};


ControllerBase::ControllerBase()
  : connect_count_(0)
{

}


ControllerBase::~ControllerBase()
{
  event::Events::DisconnectWorldUpdateBegin( updateConnection_ );
  queue_.clear();
  queue_.disable();
  nh_->shutdown();
  callback_queue_thread_.join();
  delete nh_;
}


void ControllerBase::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
{

  world_ = model->GetWorld();

  slider_model_ = model;
  slider_handle_link_ = slider_model_->GetLink( "slider_handle" );
  slider_link_ = slider_model_->GetLink( "slider" );
  slider_joint_ = slider_model_->GetJoint( "slider_joint" );
  slider_joint_->SetProvideFeedback( true );

  receptacle_model_ = world_->GetModel( "receptacle" );
  receptacle_handle_link_ = receptacle_model_->GetLink( "receptacle_handle" );
  receptacle_link_ = receptacle_model_->GetLink( "receptacle" );
  receptacle_joint_ = receptacle_model_->GetJoint( "receptacle_joint" );
  receptacle_joint_->SetProvideFeedback( true );

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  nh_ = new ros::NodeHandle;

  ros::AdvertiseOptions receptacle_pose_ao = ros::AdvertiseOptions::create<geometry_msgs::PoseStamped>( "receptacle_pose",
                                                                                                        1,
                                                                                                        boost::bind( &ControllerBase::connectCB, this ),
                                                                                                        boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                        ros::VoidPtr(),
                                                                                                        &queue_ );
  receptacle_pose_pub_ = nh_->advertise( receptacle_pose_ao );

  ros::AdvertiseOptions slider_pose_ao = ros::AdvertiseOptions::create<geometry_msgs::PoseStamped>( "slider_pose",
                                                                                                    1,
                                                                                                    boost::bind( &ControllerBase::connectCB, this ),
                                                                                                    boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                    ros::VoidPtr(),
                                                                                                    &queue_ );
  slider_pose_pub_ = nh_->advertise( slider_pose_ao );

  ros::AdvertiseOptions receptacle_wrench_ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>( "receptacle_wrench",
                                                                                                            1,
                                                                                                            boost::bind( &ControllerBase::connectCB, this ),
                                                                                                            boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                            ros::VoidPtr(),
                                                                                                            &queue_ );
  receptacle_wrench_pub_ = nh_->advertise( receptacle_wrench_ao );

  ros::AdvertiseOptions slider_wrench_ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>( "slider_wrench",
                                                                                                        1,
                                                                                                        boost::bind( &ControllerBase::connectCB, this ),
                                                                                                        boost::bind( &ControllerBase::disconnectCB, this ),
                                                                                                        ros::VoidPtr(),
                                                                                                        &queue_ );
  slider_wrench_pub_ = nh_->advertise( slider_wrench_ao );

  callback_queue_thread_ = boost::thread( boost::bind( &ControllerBase::QueueThread, this ) );

  controllerLoad( model, sdf );

  updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind( &ControllerBase::onUpdate, this, _1 ) );

}


void ControllerBase::onUpdate( const common::UpdateInfo info )
{

  controllerUpdate( info );

  if( connect_count_ <= 0 )
    return;

  math::Pose receptacle_pose = receptacle_link_->GetWorldCoGPose();
  math::Pose slider_pose = slider_link_->GetWorldCoGPose();

  geometry_msgs::PoseStamped receptacle_pose_msg;
  receptacle_pose_msg.header.frame_id = "world";
  receptacle_pose_msg.header.stamp.sec = world_->GetSimTime().sec;
  receptacle_pose_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  receptacle_pose_msg.pose.position.x = receptacle_pose.pos.x;
  receptacle_pose_msg.pose.position.y = receptacle_pose.pos.y;
  receptacle_pose_msg.pose.position.z = receptacle_pose.pos.z;
  receptacle_pose_msg.pose.orientation.x = receptacle_pose.rot.x;
  receptacle_pose_msg.pose.orientation.y = receptacle_pose.rot.y;
  receptacle_pose_msg.pose.orientation.z = receptacle_pose.rot.z;
  receptacle_pose_msg.pose.orientation.w = receptacle_pose.rot.w;

  geometry_msgs::PoseStamped slider_pose_msg;
  slider_pose_msg.header.frame_id = "world";
  slider_pose_msg.header.stamp.sec = world_->GetSimTime().sec;
  slider_pose_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  slider_pose_msg.pose.position.x = slider_pose.pos.x;
  slider_pose_msg.pose.position.y = slider_pose.pos.y;
  slider_pose_msg.pose.position.z = slider_pose.pos.z;
  slider_pose_msg.pose.orientation.x = slider_pose.rot.x;
  slider_pose_msg.pose.orientation.y = slider_pose.rot.y;
  slider_pose_msg.pose.orientation.z = slider_pose.rot.z;
  slider_pose_msg.pose.orientation.w = slider_pose.rot.w;

  receptacle_pose_pub_.publish( receptacle_pose_msg );
  slider_pose_pub_.publish( slider_pose_msg );

  physics::JointWrench receptacle_wrench = receptacle_joint_->GetForceTorque(0);
  physics::JointWrench slider_wrench = slider_joint_->GetForceTorque(0);

  geometry_msgs::WrenchStamped receptacle_wrench_msg;
  receptacle_wrench_msg.header.frame_id = "receptacle";
  receptacle_wrench_msg.header.stamp.sec = world_->GetSimTime().sec;
  receptacle_wrench_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  receptacle_wrench_msg.wrench.force.x = receptacle_wrench.body2Force.x;
  receptacle_wrench_msg.wrench.force.y = receptacle_wrench.body2Force.y;
  receptacle_wrench_msg.wrench.force.z = receptacle_wrench.body2Force.z;
  receptacle_wrench_msg.wrench.torque.x = receptacle_wrench.body2Torque.x;
  receptacle_wrench_msg.wrench.torque.y = receptacle_wrench.body2Torque.y;
  receptacle_wrench_msg.wrench.torque.z = receptacle_wrench.body2Torque.z;

  geometry_msgs::WrenchStamped slider_wrench_msg;
  slider_wrench_msg.header.frame_id = "slider";
  slider_wrench_msg.header.stamp.sec = world_->GetSimTime().sec;
  slider_wrench_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  slider_wrench_msg.wrench.force.x = slider_wrench.body2Force.x;
  slider_wrench_msg.wrench.force.y = slider_wrench.body2Force.y;
  slider_wrench_msg.wrench.force.z = slider_wrench.body2Force.z;
  slider_wrench_msg.wrench.torque.x = slider_wrench.body2Torque.x;
  slider_wrench_msg.wrench.torque.y = slider_wrench.body2Torque.y;
  slider_wrench_msg.wrench.torque.z = slider_wrench.body2Torque.z;

  receptacle_wrench_pub_.publish( receptacle_wrench_msg );
  slider_wrench_pub_.publish( slider_wrench_msg );

}


void ControllerBase::connectCB()
{
  ++connect_count_;
}


void ControllerBase::disconnectCB()
{
  --connect_count_;
}


void ControllerBase::QueueThread()
{

  static const double timeout = 0.01;

  while( nh_->ok() )
  {
    queue_.callAvailable( ros::WallDuration(timeout) );
  }

}


}