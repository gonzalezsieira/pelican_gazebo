#   pelican_gazebo: Asctec Pelican integration for the Gazebo simulator

This package provides integration for the Asctec Pelican
model in the simulator Gazebo. This integration consists in:
* Spawnning generic controllers for UAVs
* Generic dynamics for UAVs applied to the Pelican model
* Publishing pose, velocities and tf for simulated model

## Dependencies
To make this package work make sure you can access the 
following packages from your ROS workspace:
* xacro
* hector_quadrotor_controller
* hector_quadrotor_model
* pelican_description


## Information about the model
Gazebo publishes information about the model in the following topic:
**/gazebo/model_states**, under model named **pelican**.

Also, runnning script **pelican_gazebo_topics** will create two
additional topics in which this information is published in a more
accessible way:
* **/tf** 
* **/pelican/twist** (TwistStamped)
* **/pelican/pose** (PoseStamped)

Two last topics names can be changed in *parameters/pelican_gazebo_topics.yaml*.