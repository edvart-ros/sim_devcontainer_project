project where i attempt to get a ros workspace and gazebo simulator (with gzweb) working.
With this we'll help new people get started with ros and gazebo without needing a dedicated ubuntu machine locally.



- clone repository in container

```
source /opt/ros/noetic/setup.bash
```


install package dependencies

```
cd /workspaces/sim_devcontainer_project/vrx_ws
```
```
rosdep install --from-paths src --ignore-src -r -y
```

build the workspace
```
catkin_make
```
source the workspace
```
source devel/setup.bash
```


TODO: fix models not loading properly in browser. manually copy model files? model files are stored in gzweb/http/client/assets
```
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/workspaces/sim_devcontainer_project/vrx_ws/src/vrx/wamv_description/models:/workspaces/sim_devcontainer_project/vrx_ws/src/navier_wamv:/workspaces/sim_devcontainer_project/vrx_ws/src/vrx/wave_gazebo/world_models:/workspaces/sim_devcontainer_project/vrx_ws/src/vrx/wamv_gazebo/models:/workspaces/sim_devcontainer_project/vrx_ws/src/vrx/vrx_gazebo/models
```


build gzweb and models
```
cd /workspaces/sim_devcontainer_project/gzweb
```


build gzweb. This imports the gazebo models that are on GAZEBO_MODEL_PATH such that they are visible in the gzweb browser later
```
npm run deploy --- -m
```

start the simulation (headless mode)
```
roslaunch navier_bringup bringup.launch
```

with the gazebo simulation running, start the gzweb server, from the gzweb directory
```
npm start
```



TODO: use this for gui/rviz stuff? 
(enable nvidia runtime)
https://gist.github.com/sgarciav/8636b50e37def33ae379c3ea3d5d30f7