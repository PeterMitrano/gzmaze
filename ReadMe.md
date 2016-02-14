# What is gzmaze?
gzmaze is an attempt at flexing the power of [Gazebo](gazebosim.org)
The main goal here is to generate a maze in gazebo from a text file

#How it's gonna work
I think I'm going to need a model plugin and a base SDF model.
The base SDF model will simply have the base platform and maybe walls on all 4 sides
The model plugin will look for a maze file name in the SDF and on "Load" of the model
use it to add walls

#Future goals
I'd like to be able to generate random mazes with a GUI plugin
something along the lines of "regenerate" publishing to a topic
and the model plugin seeing that and regenerating a new maze

It would also be interesting if somehow I could add a file browse dialog
that could get a file name and them publish that
then the maze would reload to look like that maze
