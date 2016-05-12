# Planar folding assembly

This package contains everything required to simulate a planar folding assembly operation in Gazebo. The naming convention used is that the __slider__ object is the one assembled on top of the __receptacle__ object.


## Description of the objects

The objects are defined in two `xacro` files:

- `urdf/planar_folding_assembly_slider.urdf.xacro`
- `urdf/planar_folding_assembly_receptacle.urdf.xacro`

Both objects are constructed out of geometric primitives, since these supposedly provide better performance of the physics simulation with respect to using custom meshes.

### Slider

![Slider object](img/slider.png)

### Receptacle

![Receptacle object](img/receptacle.png)

