#!/usr/bin/env python3

"""

Copyright (c) 2023 Kieran Aponte
This software is licensed under the MIT License.

"""


#This ROS2 executable defines joints, links, properties, etc for a ROS2 robot and outputs both a xacro & urdf

import os
import pandas as pd
import ast


def create_header(outfile, robot_name, properties, world_name, robot_base_name): 
    text = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{}">
<!-- Built using Xacro Generator: https://github.com/Miimiikiu/kora_tools.git -->'''.format(robot_name)

    for prop in properties:
        text += '''\n<xacro:property name="{}" value="{}"/>'''.format(prop, properties[prop])
    text += '''\n\n<link name="{}"/>
<joint name="{}_joint" type="fixed">
  <parent link="{}"/>
  <child link="{}"/>
  <origin rpy="0 0 0" xyz="0 0 0"/>
</joint>


    '''.format(world_name, world_name, world_name, robot_base_name)
    outfile.write(text)
    return(text)

def create_link(outfile, link_name, color, ox, oy, oz, roll, pitch, yaw, desc_path):
    
    df = pd.read_csv(desc_path + '/geometry/geometry.csv', sep=';') # get data for robot inertia values
    inertials_string = df.loc[df['Mesh Name'] == '{}.obj'.format(link_name), 'Inertia Tensor'].values[0]
    inertials = ast.literal_eval(inertials_string)
    mass_string = df.loc[df['Mesh Name'] == '{}.obj'.format(link_name), 'Mass'].values[0]
    mass = float(mass_string)
    inertia_scale = 100
    collision_scale = .01
    visual_scale = 1
    text = '''<link name="{}">
      <visual>
        <origin xyz="{} {} {}" rpy="{} {} {}"/>
        <geometry>
          <mesh filename="{}" scale="{} {} {}"/>
        </geometry>
    '''.format(link_name, ox, oy, oz, roll, pitch, yaw, '${' + link_name + '_visual_mesh' + '}', visual_scale, visual_scale, visual_scale)

    if color != 'None':
      text += '''
        <material name="{}"/>
        <color rgba="{}"/>
    '''.format(color, '${' + color + '}')
      
    text += '''      </visual>
      <collision>
        <origin xyz="{} {} {}" rpy="{} {} {}"/>
        <geometry>
          <mesh filename="{}" scale="{} {} {}"/>
        </geometry>
      </collision>


'''.format(ox, oy, oz, roll, pitch, yaw, '${' + link_name + '_collision_mesh' + '}', collision_scale, collision_scale, collision_scale)
    
    text += '''
      <inertial>
        <origin xyz="{} {} {}" rpy="{} {} {}"/>
        <mass value="{}" />
        <inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}" />
      </inertial>
      <gazebo reference="{}">
      </gazebo>
</link>'''.format(ox, oy, oz, roll, pitch, yaw, mass, inertials[0][0]*inertia_scale, inertials[0][1]*inertia_scale, inertials[0][2]*inertia_scale, inertials[1][1]*inertia_scale, inertials[1][2]*inertia_scale, inertials[2][2]*inertia_scale, link_name)
    
    outfile.write(text)
    return(text)

def create_joint(outfile, joint_name, parent, child, ox, oy, oz, roll, pitch, yaw, ax, ay, az, limit_lower, limit_upper, joint_type, effort, velocity):
    text = '''<joint name="{}" type="{}">
      <parent link="{}"/>
      <child link = "{}"/>
      <origin xyz="{} {} {}" rpy="{} {} {}"/>
      '''.format(joint_name, joint_type, parent, child, ox, oy, oz, roll, pitch, yaw)
    if joint_type != 'fixed':
      text += '''<axis xyz="{} {} {}"/>
      <limit lower="{}" upper="{}" effort="{}" velocity="{}"/>
      '''.format(ax, ay, az, limit_lower, limit_upper, effort, velocity)
    text += '</joint>\n'
    outfile.write(text)
    return(text)
 
def run():
  mesh_type = 'OBJ' #OBJ or STL
  desc_path = '/home/sunset/sunset_ws/src/kora/src/kora_desc'
  robot_name = 'kora'
  v = 1.75 #joint angular velocity
  effort = 3.92 #joint effort
  #{joint_name:[joint_name, parent, child, origin x, origin y, origin z, roll, pitch, yaw, axis x, axis y, axis z, limit_lower, limit_upper, joint_type, effort, velocity]}
  joints = {'spine_x':['spine_x', 'torso_lower', 'torso_mid', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],     
          'spine_z':['spine_z', 'torso_mid', 'torso_upper', 0, 0, .1, 0, 0, 0, 0, 0, 1, -.22, .22, 'revolute', effort, v],
          'backpack_joint':['backpack_joint', 'torso_upper', 'backpack', 0, .12, .1, 0, 0, 0, 0, 0, 1, -.22, .22, 'fixed', effort, v], #axis, limits, etc disregarded for fixed joints
          'neck_connection_joint':['neck_connection_joint', 'torso_upper', 'neck_connection', 0, 0, .22609, 0, 0, 0, 1, 0, 0, 0, .22, 'fixed', effort, v],
          'neck_x':['neck_x', 'torso_upper', 'neck', 0, 0, .14829, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'neck_z':['neck_z', 'neck', 'head', 0, 0, .12975, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'realsense_joint':['realsense_joint', 'head', 'realsense', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'fixed', effort, v], #-.2y, .2z

          'shoulder_roll_dexter':['shoulder_roll_dexter', 'torso_upper', 'shoulder_bracket_dexter', -.10976, 0, .14829, 0, .165143, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'shoulder_roll_sinister':['shoulder_roll_sinister', 'torso_upper', 'shoulder_bracket_sinister', .10976, 0, .14829, 0, -.165143, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'shoulder_pitch_dexter':['shoulder_pitch_dexter', 'shoulder_bracket_dexter', 'shoulder_link_dexter', -.076, 0, 0, 0, -.165143, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'shoulder_pitch_sinister':['shoulder_pitch_sinister', 'shoulder_bracket_sinister', 'shoulder_link_sinister', .076, 0, 0, 0, .165143, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'arm_upper_rot_dexter':['arm_upper_rot_dexter', 'shoulder_link_dexter', 'arm_upper_dexter', -.102, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'arm_upper_rot_sinister':['arm_upper_rot_sinister', 'shoulder_link_sinister', 'arm_upper_sinister', .102, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'elbow_dexter':['elbow_dexter', 'arm_upper_dexter', 'forearm_dexter', -.1550, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'elbow_sinister':['elbow_sinister', 'arm_upper_sinister', 'forearm_sinister', .1550, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'wrist_rot_dexter':['wrist_rot_dexter', 'forearm_dexter', 'wrist_rot_link_dexter', -.1, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'wrist_pitch_dexter':['wrist_pitch_dexter', 'wrist_rot_link_dexter', 'wrist_ball_dexter', -.055, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'wrist_yaw_dexter':['wrist_yaw_dexter', 'wrist_ball_dexter', 'hand_dexter', -.05, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          
          'wrist_rot_sinister':['wrist_rot_sinister', 'forearm_sinister', 'wrist_rot_link_sinister', .1, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'wrist_pitch_sinister':['wrist_pitch_sinister', 'wrist_rot_link_sinister', 'wrist_ball_sinister', .055, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'wrist_yaw_sinister':['wrist_yaw_sinister', 'wrist_ball_sinister', 'hand_sinister', .05, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          
          'hip_yaw_dexter':['hip_yaw_dexter', 'torso_lower', 'hip_bracket_dexter', -.074255, 0, -.05657, 0, -.785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_roll_dexter':['hip_roll_dexter', 'hip_bracket_dexter', 'hip_motor_link_dexter', -.09714, 0, -.08195, 0, .785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_pitch_dexter':['hip_pitch_dexter', 'hip_motor_link_dexter', 'thigh_dexter', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'knee_dexter':['knee_dexter', 'thigh_dexter', 'calf_dexter', -.005, -.0382, -.21666, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_pitch_dexter':['ankle_pitch_dexter', 'calf_dexter', 'ankle_link_dexter', 0, .0547, -.31021, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_roll_dexter':['ankle_roll_dexter', 'ankle_link_dexter', 'foot_dexter', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],

          'hip_yaw_sinister':['hip_yaw_sinister', 'torso_lower', 'hip_bracket_sinister', .074255, 0, -.05657, 0, .785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_roll_sinister':['hip_roll_sinister', 'hip_bracket_sinister', 'hip_motor_link_sinister', .09714, 0, -.08195, 0, -.785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_pitch_sinister':['hip_pitch_sinister', 'hip_motor_link_sinister', 'thigh_sinister', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'knee_sinister':['knee_sinister', 'thigh_sinister', 'calf_sinister', .005, -.0382, -.21666, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_pitch_sinister':['ankle_pitch_sinister', 'calf_sinister', 'ankle_link_sinister', 0, .0547, -.31021, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_roll_sinister':['ankle_roll_sinister', 'ankle_link_sinister', 'foot_sinister', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v]
 
  }
  #Define link names (and link stl colors)
  if mesh_type == 'STL':                                                         
    links = {'torso_lower':'Black',
            'torso_mid':'Black',
            'torso_upper':'Black'
            }
  elif mesh_type == 'OBJ':
  #{link_name:[color, origin x, origin y, origin z, roll, pitch, yaw]}
   links = {'torso_lower':['None', 0, 0, 0, 0, 0, 0],
            'torso_mid':['None', 0, 0, 0, 0, 0, 0],
            'torso_upper':['None', 0, 0, 0, 0, 0, 0],
            'backpack':['None', 0, 0, 0, 0, 0, 0],
            'neck':['None', 0, 0, .10357, 0, 0, 0],
            'neck_connection':['None', 0, 0, 0, 0, 0, 0],
            'head':['None', 0, 0, 0, 0, 0, 0],
            'realsense':['None', 0, 0, 0, 0, 0, 0],
            'shoulder_bracket_dexter':['None', 0, 0, 0, 0, 0, 0],
            'shoulder_link_dexter':['None', 0, 0, 0, 0, 0, 0],
            'arm_upper_dexter':['None', 0, 0, 0, 0, 0, 0],
            'forearm_dexter':['None', 0, 0, 0, 0, 0, 0],
            'wrist_rot_link_dexter':['None', 0, 0, 0, 0, 0, 0],
            'wrist_ball_dexter':['None', 0, 0, 0, 0, 0, 0],
            'hand_dexter':['None', 0, 0, 0, 0, 0, 0],

            'shoulder_bracket_sinister':['None', 0, 0, 0, 0, 0, 0],
            'shoulder_link_sinister':['None', 0, 0, 0, 0, 0, 0],
            'arm_upper_sinister':['None', 0, 0, 0, 0, 0, 0],
            'forearm_sinister':['None', 0, 0, 0, 0, 0, 0],
            'wrist_rot_link_sinister':['None', 0, 0, 0, 0, 0, 0],
            'wrist_ball_sinister':['None', 0, 0, 0, 0, 0, 0],
            'hand_sinister':['None', 0, 0, 0, 0, 0, 0],

            'hip_bracket_dexter':['None', 0, 0, 0, 0, 0, 0],
            'hip_motor_link_dexter':['None', 0, 0, 0, 0, 0, 0],
            'thigh_dexter':['None', 0, 0, 0, 0, 0, 0],
            'calf_dexter':['None', 0, 0, 0, 0, 0, 0],
            'ankle_link_dexter':['None', 0, 0, 0, 0, 0, 0],
            'foot_dexter':['None', 0, 0, 0, 0, 0, 0],

            'hip_bracket_sinister':['None', 0, 0, 0, 0, 0, 0],
            'hip_motor_link_sinister':['None', 0, 0, 0, 0, 0, 0],
            'thigh_sinister':['None', 0, 0, 0, 0, 0, 0],
            'calf_sinister':['None', 0, 0, 0, 0, 0, 0],
            'ankle_link_sinister':['None', 0, 0, 0, 0, 0, 0],
            'foot_sinister':['None', 0, 0, 0, 0, 0, 0]
            
            }    

  #define STL colors and mesh locations, and any additional required properties
  properties = {'Black':'0.0 0.0 0.0 1.0'}
  for link in links:
     properties[link + '_visual_mesh'] = 'file://{}/visuals/{}/{}.{}'.format(desc_path, mesh_type.upper(), link, mesh_type.lower())
     properties[link + '_collision_mesh'] = 'file://{}/collisions/{}/{}_collision.{}'.format(desc_path, mesh_type.upper(), link, mesh_type.lower())
  
  
  with open('{}/kora.urdf.xacro'.format(desc_path), 'w') as outfile:
    header_text = create_header(outfile, robot_name, properties, 'world', 'torso_lower') #Assumes fixed joint between robot & world
    outfile.write('\n\n')
    for link in links:
        link_text = create_link(outfile, link, *links[link], desc_path)
        outfile.write('\n')
    outfile.write('\n')
    for joint in joints:    
        joint_text = create_joint(outfile, *joints[joint]) # unpack value
        outfile.write('\n')
    outfile.write('\n</robot>')


  os.system('ros2 run xacro xacro {}/kora.urdf.xacro > {}/kora.urdf'.format(desc_path, desc_path)) #generate urdf from xacro

run()
