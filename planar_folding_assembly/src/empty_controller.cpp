#include "planar_folding_assembly_base_plugin.h"

namespace gazebo
{
  
class EmptyController : public PlanarFoldingAssemblyBasePlugin
{
  
  virtual void controllerLoad( physics::ModelPtr model, sdf::ElementPtr sdf ) {}

  virtual void controllerUpdate( const common::UpdateInfo info ) {}

};

GZ_REGISTER_MODEL_PLUGIN( EmptyController )

}
