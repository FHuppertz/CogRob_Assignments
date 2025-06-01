# CogRob_Assignments
# Notes
This repository contains our ROS2 nodes that executes the imitation learning for a trajectory and simulates a demo, where a robot arm follows a sampled and transformed trajectory.

Due to constraints in member availability, we hard coded the trajectory learning and execution in simulation. With little change, our code can learn from given demonstrations, as well as use different start and end positions for the demonstration.

# Learning the trajectory
In the repository there is a JupyterNotebook (trajectory_learning_from_demonstration.ipynb), which explains the Gaussian-Mixture-Model and its application in our assignment.

# Executing the demonstration
To run our code, first one needs to clone the repository into their ROS2 workspace and then use colcon build. 

Afterwards one needs to run the TracScale node: ros2 run TracScale TracScale

And then, when the TracScale is initialized, one must run the imitation_learning node: ros2 run imitation_learning trajectory_generator

Now a Gaussian-Mixture-Model is learned and sampled to generate a sample trajectory. This is then transformed to be inside the robots workspace with given start and goal positions. 

After the transformation a figure will apear showing the sampled trajectory as well as the transformation to be executed. Simply close this window to start the simulation.

### Video showing the full pipeline: 
In the files of this submission there is a video showing a simulated demonstration of our trajectory learning and execution.
