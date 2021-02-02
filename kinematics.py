test

#Final PID values for 0.01kg End effector mass, Task H
#i_clamp_min and max added to allow adjustment of i values, otherwise default min/max is 0.
#final values p 110, i 0, d 90
#.yaml file
DESE3R:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 100  
  
  # Position Controllers ---------------------------------------
  joint_0_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_0
    pid: {p: 110, i: 0, d: 90, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_1_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_1
    pid: {p: 110, i: 0, d: 90, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_2
    pid: {p: 110, i: 0, d: 90, i_clamp_max: 1000, i_clamp_min: 1000}
      
##################################################################################
#TASK I
#.xacro file
<?xml version="1.0"?>
<robot name="DESE3R" xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:property name="link_length_0_init" value="1"/>
<xacro:property name="link_length_1_init" value="1"/>
<xacro:property name="link_length_2_init" value="1"/>

<xacro:property name="link_radius" value="0.05"/>
<xacro:property name="link_mass" value="0.5"/>
<xacro:property name="base_height" value="0.1"/>
<xacro:property name="base_side" value="0.6"/>
<xacro:property name="case_radius" value="${link_radius}"/>
<xacro:property name="case_length" value="${case_radius*2}"/>
<xacro:property name="ee_length" value="0.04"/>
<xacro:property name="ee_side" value="0.02"/>

<xacro:property name="ee_mass" value="30"/> # END EFFECTOR MASS CHANGED HERE, ONLY LINE CHANGED FOR THIS TASK.

<xacro:property name="link_length_0" value="${link_length_0_init-case_length}"/>
<xacro:property name="link_length_1" value="${link_length_1_init-case_radius}"/>
<xacro:property name="link_length_2" value="${link_length_2_init-case_radius}"/>
##################################################################################
#Final PID values for 30kg End effector mass, Task J
#final values p 3250, i 0, d 1600
#.yaml file
DESE3R:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 100  
  
  # Position Controllers ---------------------------------------
  joint_0_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_0
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_1_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_1
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}
  joint_2_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint_2
    pid: {p: 3250, i: 0, d: 1600, i_clamp_max: 1000, i_clamp_min: 1000}
      
##################################################################################
