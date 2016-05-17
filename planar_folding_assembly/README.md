# Planar folding assembly

This package contains everything required to simulate a planar folding assembly operation in Gazebo.
The naming convention used is that the __slider__ object is the one assembled on top of the __receptacle__ object.

![Simulation screenshot](img/sim_screenshot.bmp)


## Description of the objects

The objects are defined in two `xacro` files:

- `urdf/slider.urdf.xacro`
- `urdf/receptacle.urdf.xacro`

Both objects are constructed out of geometric primitives, since these supposedly provide better performance of the physics simulation with respect to using custom meshes.

### Slider

![Slider object](img/slider.png)

### Receptacle

![Receptacle object](img/receptacle.png)


## Basic usage

To start the simulation using one of the provided controllers, the simulator with the receptacle object need to be launched first, using

```
roslaunch planar_folding_assembly planar_folding_assembly.launch
```

Once the simluator is up and running, the slider object can be spawned with an argument specifying the controller plugin to be loaded with

```
roslaunch planar_folding_assembly spawn_slider.launch model_plugin:=PLUGIN_NAME.so
```

The reason for forcing the spawning order is that the controller plugin will try to retrieve a pointer to the receptacle object when being loaded, and spawning both at the same time can cause errors due to race conditions.

## Controller infrastructure

A base class for the controller plugin is provided in [`PlanarFoldingAssemblyBasePlugin`](src/planar_folding_assembly_base_plugin.h).
This base class takes care of the following resposabilities:

- Get pointers to both objects and the joints attaching them to their handles
- Set up ROS publishers for the wrench applied by the objects' joints
- Set up ROS publishers for the pose of the objects (__TODO__)

New controller plugins can be deployed following the next steps:

1. Create a new class deriving from `PlanarFoldingAssemblyBasePlugin` (see the [`EmptyController`](src/empty_controller.cpp) for a barebones example)
2. Make sure to advertise it as a plugin using the `GZ_REGISTER_MODEL_PLUGIN` macro
3. Add the new source file to the `CMakeLists.txt` as follows

        add_library( gazebo_ros_my_awesome_controller src/my_awesome_controller.cpp )
        target_link_libraries( gazebo_ros_my_awesome_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )

Then, when spawning the slider after the simulator has been started, you can load the new controller by specifying it as an argument with

```
roslaunch planar_folding_assembly spawn_slider.launch model_plugin:=libgazebo_ros_my_awesome_plugin.so
```

__Note that `lib` and `.so` are automatically pre/apended to the plugin name and need to be specified to the roslaunch argument.__
