BUILD_DIR=cmake-build-debug

source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=${PWD}/${BUILD_DIR}:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=${PWD}/${BUILD_DIR}:$LD_LIBRARY_PATH
export GAZEBO_MODEL_PATH=${PWD}
