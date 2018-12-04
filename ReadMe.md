# What is gzmaze?
gzmaze is an attempt at flexing the power of [Gazebo](http://gazebosim.org)
The main goal here is to generate a maze in gazebo from a text file.

# How it works
There are two components, a GUI overlay plugin and a world plugin.

### GUI overlay
Gazebo has [a tutorial](http://gazebosim.org/tutorials?tut=gui_overlay&cat=user_input) on simple GUI overlays. I followed that tutorial, and extended it slightly.
There are two buttons and a textedit. When the buttons are clicked, a message is published to the topic ~/maze/regenerate

### World plugin
This is where the meat of the code is. We subscribe to ~/maze/regenerate and build mazes using gazebo messages. This plugin took example from the gazebo [Model editor](https://bitbucket.org/osrf/gazebo/src/default/gazebo/gui/model/). Essentially, it uses gazebo messages to construct Collision and Links. There links are then converted to and sdf::ElementPtr via the convenient funtions VisualToSDF and CollisionToSDF.

# Requirements
cmake 2.8
Gazebo 9

To get the dependencies, you can probably just run

    sudo apt install gazebo9 libgazebo9-dev

# Building

    mkdir build
    cd build
    cmake .. && make

# Running
Be sure to add the following to your `.gazebo/gui.ini` file

```
    [overlay_plugins]
    filenames=libregenerate_widget.so
```


    ./setup.sh #this will setup the environment variables you need and run gazebo

# The input files
look at sample_maze.mz for an example

# TODO:
 - let the mouse model actually drive around
