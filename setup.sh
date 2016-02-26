export GAZEBO_PLUGIN_PATH=/home/peter/Projects/gzmaze/world_plugin/build:\
/home/peter/Projects/gzmaze/gui_plugin/build:\
/home/peter/Projects/gzmaze/model_plugin/build:\
/usr/lib/x86_64-linux-gnu/gazebo-7/plugins:\
$GAZEBO_PLUGIN_PATH

export LD_LIBRARY_PATH=/home/peter/Projects/gzmaze/world_plugin/build:\
/home/peter/Projects/gzmaze/gui_plugin/build:\
/home/peter/Projects/gzmaze/model_plugin/build:\
$LD_LIBRARY_PATH

export GAZEBO_MODEL_PATH=/home/peter/Projects/gzmaze

gazebo --verbose gzmaze.world
