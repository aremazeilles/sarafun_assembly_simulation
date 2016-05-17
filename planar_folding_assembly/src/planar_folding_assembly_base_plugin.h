#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/WrenchStamped.h>

#include <boost/bind.hpp>

#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>


namespace gazebo
{

class PlanarFoldingAssemblyBasePlugin : public ModelPlugin
{

public:

  PlanarFoldingAssemblyBasePlugin();

  virtual ~PlanarFoldingAssemblyBasePlugin();

  void Load( physics::ModelPtr model, sdf::ElementPtr sdf );

  void onUpdate( const common::UpdateInfo info );

  virtual void controllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf ) {}

  virtual void controllerUpdate( const common::UpdateInfo info ) {}

  void connectCB();

  void disconnectCB();

private:

  void QueueThread();
  
  ros::NodeHandle* nh_;

  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;

  int connect_count_;
  ros::Publisher receptacle_pose_pub_;
  ros::Publisher receptacle_wrench_pub_;
  ros::Publisher slider_pose_pub_;
  ros::Publisher slider_wrench_pub_;

  physics::WorldPtr world_;

  physics::ModelPtr slider_model_;
  physics::LinkPtr slider_handle_link_;
  physics::LinkPtr slider_link_;
  physics::JointPtr slider_joint_;

  physics::ModelPtr receptacle_model_;
  physics::LinkPtr receptacle_handle_link_;
  physics::LinkPtr receptacle_link_;
  physics::JointPtr receptacle_joint_;

  event::ConnectionPtr updateConnection_;

};


PlanarFoldingAssemblyBasePlugin::PlanarFoldingAssemblyBasePlugin()
  : connect_count_(0)
{

}


PlanarFoldingAssemblyBasePlugin::~PlanarFoldingAssemblyBasePlugin()
{
  event::Events::DisconnectWorldUpdateBegin( updateConnection_ );
  queue_.clear();
  queue_.disable();
  nh_->shutdown();
  callback_queue_thread_.join();
  delete nh_;
}


void PlanarFoldingAssemblyBasePlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf )
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

  ros::AdvertiseOptions receptacle_wrench_ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>( "receptacle_wrench",
                                                                                                            1,
                                                                                                            boost::bind( &PlanarFoldingAssemblyBasePlugin::connectCB, this ),
                                                                                                            boost::bind( &PlanarFoldingAssemblyBasePlugin::disconnectCB, this ),
                                                                                                            ros::VoidPtr(),
                                                                                                            &queue_ );
  receptacle_wrench_pub_ = nh_->advertise( receptacle_wrench_ao );

  ros::AdvertiseOptions slider_wrench_ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>( "slider_wrench",
                                                                                                        1,
                                                                                                        boost::bind( &PlanarFoldingAssemblyBasePlugin::connectCB, this ),
                                                                                                        boost::bind( &PlanarFoldingAssemblyBasePlugin::disconnectCB, this ),
                                                                                                        ros::VoidPtr(),
                                                                                                        &queue_ );
  slider_wrench_pub_ = nh_->advertise( slider_wrench_ao );

  callback_queue_thread_ = boost::thread( boost::bind( &PlanarFoldingAssemblyBasePlugin::QueueThread, this ) );

  controllerLoad( model, sdf );

  updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind( &PlanarFoldingAssemblyBasePlugin::onUpdate, this, _1 ) );

}


void PlanarFoldingAssemblyBasePlugin::onUpdate( const common::UpdateInfo info )
{

  controllerUpdate( info );

  if( connect_count_ <= 0 )
    return;

  physics::JointWrench receptacle_wrench = receptacle_joint_->GetForceTorque(0);
  physics::JointWrench slider_wrench = slider_joint_->GetForceTorque(0);

  geometry_msgs::WrenchStamped receptacle_wrench_msg;
  receptacle_wrench_msg.header.frame_id = "receptacle";
  receptacle_wrench_msg.header.stamp.sec = world_->GetSimTime().sec;
  receptacle_wrench_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  receptacle_wrench_msg.wrench.force.x = receptacle_wrench.body1Force.x;
  receptacle_wrench_msg.wrench.force.y = receptacle_wrench.body1Force.y;
  receptacle_wrench_msg.wrench.force.z = receptacle_wrench.body1Force.z;
  receptacle_wrench_msg.wrench.torque.x = receptacle_wrench.body1Torque.x;
  receptacle_wrench_msg.wrench.torque.y = receptacle_wrench.body1Torque.y;
  receptacle_wrench_msg.wrench.torque.z = receptacle_wrench.body1Torque.z;

  geometry_msgs::WrenchStamped slider_wrench_msg;
  slider_wrench_msg.header.frame_id = "slider";
  slider_wrench_msg.header.stamp.sec = world_->GetSimTime().sec;
  slider_wrench_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  slider_wrench_msg.wrench.force.x = slider_wrench.body1Force.x;
  slider_wrench_msg.wrench.force.y = slider_wrench.body1Force.y;
  slider_wrench_msg.wrench.force.z = slider_wrench.body1Force.z;
  slider_wrench_msg.wrench.torque.x = slider_wrench.body1Torque.x;
  slider_wrench_msg.wrench.torque.y = slider_wrench.body1Torque.y;
  slider_wrench_msg.wrench.torque.z = slider_wrench.body1Torque.z;

  receptacle_wrench_pub_.publish( receptacle_wrench_msg );
  slider_wrench_pub_.publish( slider_wrench_msg );

}


void PlanarFoldingAssemblyBasePlugin::connectCB()
{
  ++connect_count_;
}


void PlanarFoldingAssemblyBasePlugin::disconnectCB()
{
  --connect_count_;
}


void PlanarFoldingAssemblyBasePlugin::QueueThread()
{

  static const double timeout = 0.01;

  while( nh_->ok() )
  {
    queue_.callAvailable( ros::WallDuration(timeout) );
  }

}


}