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

    physics::JointPtr handle_joint = model_->GetJoint( "handle_joint" );
    handle_joint->SetProvideFeedback( true );

  }

  void onUpdate( const common::UpdateInfo info )
  {

    // model_->SetLinearVel( math::Vector3( 0, 0, -0.1 ) );

    physics::LinkPtr handle_link = model_->GetLink( "handle" );
    physics::LinkPtr box_link = model_->GetLink( "box" );

    physics::JointPtr handle_joint = model_->GetJoint( "handle_joint" );

    handle_link->SetLinearVel( math::Vector3( 0, 0, -0.1 ) );

    // box_link->AddForce( math::Vector3( 0.0, 0.0, 9.0 ) );

    // math::Vector3 f = box_link->GetWorldForce();
    math::Vector3 fb1 = handle_joint->GetForceTorque(0).body1Force;
    math::Vector3 fb2 = handle_joint->GetForceTorque(0).body2Force;

    std::cout << "fb1: " << fb1.x << ", " << fb1.y << ", " << fb1.z << std::endl;
    std::cout << "fb2: " << fb2.x << ", " << fb2.y << ", " << fb2.z << std::endl;

  }

private:

  physics::ModelPtr model_;

  event::ConnectionPtr updateConnection_;

};

GZ_REGISTER_MODEL_PLUGIN( PlanarFoldingAssemblyControllerPlugin )

}