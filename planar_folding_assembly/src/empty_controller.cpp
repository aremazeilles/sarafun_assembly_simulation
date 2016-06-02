#include "base/controller_base.h"

namespace gazebo
{


class EmptyController : public ControllerBase
{
  
  virtual void controllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf ) {}

  virtual void controllerUpdate( const common::UpdateInfo info ) {}

};


GZ_REGISTER_MODEL_PLUGIN( EmptyController )


}
