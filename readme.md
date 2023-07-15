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


TODO: add model paths to GAZEBO_MODEL_PATH
```
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=<path_to_model>:<another_path_to_model>...
```




build gzweb and models
```
cd /workspaces/sim_devcontainer_project/gzweb
```
TODO: find out why this is required (We are root)
```
chmod 777 get_local_models.py webify_models_v2.py webify_models.py deploy.sh gzbridge/server.js
```


build gzweb. This imports gazebo models such that they are visible in the gzweb browser later
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







