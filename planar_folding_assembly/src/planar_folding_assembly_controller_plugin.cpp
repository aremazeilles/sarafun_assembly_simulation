#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <boost/bind.hpp>

#include <iostream>


namespace gazebo
{

class PlanarFoldingAssemblyControllerPlugin : public ModelPlugin
{

public:

  void Load( physics::ModelPtr parent, sdf::ElementPtr sdf )
  {

    model_ = parent;

    updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind( &PlanarFoldingAssemblyControllerPlugin::onUpdate, this, _1 ) );

    handle_link_ = model_->GetLink( "slider_handle" );
    box_link_ = model_->GetLink( "slider" );

    handle_joint_ = model_->GetJoint( "slider_joint" );

    handle_joint_->SetProvideFeedback( true );

  }

  void onUpdate( const common::UpdateInfo info )
  {

    handle_link_->SetLinearVel( math::Vector3( 0, 0, -0.1 ) );

    math::Vector3 fb1 = handle_joint_->GetForceTorque(0).body1Force;
    math::Vector3 fb2 = handle_joint_->GetForceTorque(0).body2Force;

    std::cout << "fb1: " << fb1.x << ", " << fb1.y << ", " << fb1.z << std::endl;
    std::cout << "fb2: " << fb2.x << ", " << fb2.y << ", " << fb2.z << std::endl;

  }

private:

  physics::ModelPtr model_;

  physics::LinkPtr handle_link_;
  physics::LinkPtr box_link_;

  physics::JointPtr handle_joint_;

  event::ConnectionPtr updateConnection_;

};

GZ_REGISTER_MODEL_PLUGIN( PlanarFoldingAssemblyControllerPlugin )

}