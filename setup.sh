export GAZEBO_PLUGIN_PATH=${HOME}/Projects/smartmouse/gzmaze/world_plugin/build:\
${HOME}/Projects/smartmouse/gzmaze/gui_plugin/build:\
${HOME}/Projects/smartmousegzmaze/model_plugin/build:\
/usr/lib/x86_64-linux-gnu/gazebo-7/plugins:\
$GAZEBO_PLUGIN_PATH

export LD_LIBRARY_PATH=${HOME}/Projects/smartmouse/gzmaze/world_plugin/build:\
${HOME}/Projects/smartmouse/gzmaze/gui_plugin/build:\
${HOME}/Projects/smartmouse/gzmaze/model_plugin/build:\
/usr/local/lib/x86_64-linux-gnu/gazebo-7/plugins:\
$LD_LIBRARY_PATH

export GAZEBO_MODEL_PATH=${HOME}/Projects/smartmouse/gzmaze

gazebo -u --verbose gzmaze.world
