#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <boost/bind.hpp>

#include <iostream>


namespace gazebo
{

class PlanarFoldingAssemblyControllerPlugin : public ModelPlugin
{

public:

  void Load( physics::ModelPtr model, sdf::ElementPtr sdf )
  {

    model_ = model;

    slider_handle_link_ = model_->GetLink( "slider_handle" );
    slider_link_ = model_->GetLink( "slider" );

    slider_joint_ = model_->GetJoint( "slider_joint" );

    receptacle_joint_ = model_->GetWorld()->GetModel( "receptacle" )->GetJoint( "receptacle_joint" );

    slider_joint_->SetProvideFeedback( true );
    receptacle_joint_->SetProvideFeedback( true );

    updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind( &PlanarFoldingAssemblyControllerPlugin::onUpdate, this, _1 ) );

  }

  void onUpdate( const common::UpdateInfo info )
  {

    slider_handle_link_->SetLinearVel( math::Vector3( 0, 0, -0.1 ) );

    math::Vector3 fb1 = receptacle_joint_->GetForceTorque(0).body1Force;
    math::Vector3 fb2 = receptacle_joint_->GetForceTorque(0).body2Force;

    std::cout << "fb1: " << fb1.x << ", " << fb1.y << ", " << fb1.z << std::endl;
    std::cout << "fb2: " << fb2.x << ", " << fb2.y << ", " << fb2.z << std::endl;

  }

private:

  physics::ModelPtr model_;

  physics::LinkPtr slider_handle_link_;
  physics::LinkPtr slider_link_;

  physics::JointPtr slider_joint_;

  physics::JointPtr receptacle_joint_;

  event::ConnectionPtr updateConnection_;

};

GZ_REGISTER_MODEL_PLUGIN( PlanarFoldingAssemblyControllerPlugin )

}