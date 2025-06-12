
# Installation:

```
git clone
mkdir build
cd build
source /opt/ros/jazzy/setup.bash
cmake ..
make
```
# Test on TALOS

You need to have the repository https://gitlab.laas.fr/ostasse/gz_gepetto_humanoids_models.git

For efficiency concerns you can put:
```
export GZ_IP=127.0.0.1
```

To start the demonstration with the ballon:
```
gz sim ./worlds/ball_talos/ball_talos.sdf
```

For some reason (probably some race conditions due to the numerous contact collision similated),
we need to listen to the cmd_forces Gazebo topic in order to make sure that Gazebo is listening to it:
```
gz topic -e -t /model/Pyrene/joints/cmd_forces
```

Then you can send the control in the build directory of gz_gep_tools:
```
./control_loop t
```
The 't' argument is to use the configuration informations for TALOS.
They are located in ./src/robots_data.cc
