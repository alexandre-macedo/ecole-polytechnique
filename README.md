# Bateauautomatix X2015

## Cloning

To clone the complete repository and its submodules do
```
git clone --recursive https://arj.macedo@gitlab.com/arj.macedo/bateautomotix2015.git
```

To clone the submodules in an already cloned repository do
```
git submodule update --init --recursive
```

## Installing

For this part it is assumed that ROS is installed in ` /opt/ros/(distribution)`. To install in this folder, you will need superuser privileges. Normally, ROS is not in the superuser path, therefore you should do
```
su
source /opt/ros/(distribution)/setup.bash
```
Then you can proceed to install the 3rd party packages as a superuser.
```
cd (cloned location)/catkin_ws_3rdparty
catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/(distribution)
```

To install the bateautomatix packages.

```
cd (cloned location)/catkin_ws
catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/(distribution)
```
