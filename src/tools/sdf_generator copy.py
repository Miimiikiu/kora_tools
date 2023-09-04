#!/usr/bin/env python3

"""

Copyright (c) 2023 Kieran Aponte
This software is licensed under the MIT License.

"""


#This ROS2 executable defines joints, links, properties, etc for a ROS2 robot and outputs an SDF file

import os

def create_header(outfile, robot_name, world_name): 
    text = '''<?xml version="1.0"?>
<sdf version="1.9">
  

<!-- Built using SDF Generator: https://github.com/Bentell-Robotics/kora_tools.git -->'''

    #for prop in properties:
    #    text += '''\n<xacro:property name="{}" value="{}"/>'''.format(prop, properties[prop])
    text += '''\n\n
  <world name="kora_world">
    <physics name="1ms" type="ignored">
        <!--<ode>
            <solver>
                <type>"quick"</type>
                <sor>1.3</sor>
                <iters>50</iters>
                <use_dynamic_moi_rescaling>false</use_dynamic_moi_rescaling>
            </solver>
        </ode>-->
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin
        filename="ignition-gazebo-physics-system"
        name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
        filename="ignition-gazebo-user-commands-system"
        name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin
        filename="ignition-gazebo-scene-broadcaster-system"
        name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>


    <light type="directional" name="sun">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 10 0 0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
    </light>

    <scene>
        <sky/>
    </scene>

    <model name="ground_plane">
        <static>true</static>
        <link name="link">
            <collision name="collision">
            <geometry>
                <plane>
                <normal>0 0 1</normal>
                </plane>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <plane>
                <normal>0 0 1</normal>
                <size>100 100</size>
                </plane>
            </geometry>
            <material>
                <ambient>0.8 0.8 0.8 1</ambient>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.8 0.8 0.8 1</specular>
            </material>
            </visual>
        </link>
    </model>

    <model name="kora" canonical_link='torso_lower'>
      <pose relative_to='world'>0 0 1 0 0 0</pose> 


    '''
    outfile.write(text)
    return(text)

def create_link(outfile, link_name, color, ox, oy, oz, roll, pitch, yaw, parent_link=None):
    
    text = '''      <link name="{}">
        <pose relative_to='{}'>{} {} {} {} {} {}</pose>
        <visual name='visual_{}'>
          <geometry>
            <mesh>
              <uri>./{}</uri>
            </mesh>
          </geometry>
         
        </visual>
    '''.format(link_name, parent_link[0], float(parent_link[1] + ox), float(parent_link[2] + oy), float(parent_link[3] + oz), float(parent_link[4]), float(parent_link[5]), float(parent_link[6]), link_name, 'OBJ/' + link_name + '.obj')

    if color != 'None':
      text += '''
        <material name="{}"/>
        <color rgba="{}"/>
    '''.format(color, '${' + color + '}')
      
    text += '''
        <collision name='collision_{}'>
        
          <geometry>
            <mesh>
              <uri>./{}</uri>
            </mesh>
          </geometry>
    
        </collision>
        <inertial> <!--inertial properties of the link mass, inertia matix-->
          <mass>1.14395</mass>
          <inertia>
            <ixx>0.095329</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.381317</iyy>
            <iyz>0</iyz>
            <izz>0.476646</izz>
          </inertia>
        </inertial>
      </link>
'''.format(link_name, 'OBJ/' + link_name + '.obj')
    
    outfile.write(text)
    return(text)

def create_joint(outfile, joint_name, parent, child, ox, oy, oz, roll, pitch, yaw, ax, ay, az, limit_lower, limit_upper, joint_type, effort, velocity):
    text = '''      <joint name="{}" type="{}">
        <parent>{}</parent>
        <child>{}</child>
        <pose relative_to='{}'>{} {} {} {} {} {}</pose>


      
        '''.format(joint_name, joint_type, parent, child, parent, float(ox), float(oy), float(oz), float(roll), float(pitch), float(yaw))
    if joint_type != 'fixed':
        
      text += '''<axis>
        <!--<xyz expressed_in='__model__'>1 1 1</xyz>-->
          <xyz>1 0 0</xyz> 
        

            <!--{} {} {}-->
          <limit>
            <lower>{}</lower> 
            <upper>{}</upper> 
            <effort>{}</effort>
            <velocity>{}</velocity>
          </limit>
        </axis>
      '''.format(ax, ay, az, float(limit_lower), float(limit_upper), float(effort), float(velocity))
    text += '</joint>\n'
    outfile.write(text)
    return(text)
 
def run():
  mesh_type = 'OBJ' #OBJ or STL
  output_location = '/home/sunset/sunset_ws/src/kora/src/kora_desc'
  robot_name = 'kora'
  v = 1.75 #joint angular velocity
  effort = 3.92 #joint effort
  #{joint_name:[joint_name, parent, child, origin x, origin y, origin z, roll, pitch, yaw, axis x, axis y, axis z, limit_lower, limit_upper, joint_type, effort, velocity]}
  joints = {'spine_x':['spine_x', 'torso_lower', 'torso_mid', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],     
          'spine_z':['spine_z', 'torso_mid', 'torso_upper', 0, 0, .1, 0, 0, 0, 0, 0, 1, -.22, .22, 'revolute', effort, v],
          'backpack_joint':['backpack_joint', 'torso_upper', 'backpack', 0, .12, .1, 0, 0, 0, 0, 0, 1, -.22, .22, 'fixed', effort, v], #axis, limits, etc disregarded for fixed joints
          'neck_connection_joint':['neck_connection_joint', 'torso_upper', 'neck_connection', 0, 0, .22609, 0, 0, 0, 1, 0, 0, 0, .22, 'fixed', effort, v],
          'neck_x':['neck_x', 'torso_upper', 'neck', 0, 0, .14829, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'neck_z':['neck_z', 'neck', 'head', 0, 0, .12975, 0, 0, 0, 0, 0, 1, 0, .22, 'revolute', effort, v],
          'realsense_joint':['realsense_joint', 'head', 'realsense', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'fixed', effort, v],

          'shoulder_roll_dexter':['shoulder_roll_dexter', 'torso_upper', 'shoulder_bracket_dexter', -.10976, 0, .14829, 0, .165143, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'shoulder_roll_sinister':['shoulder_roll_sinister', 'torso_upper', 'shoulder_bracket_sinister', .10976, 0, .14829, 0, -.165143, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'shoulder_pitch_dexter':['shoulder_pitch_dexter', 'shoulder_bracket_dexter', 'shoulder_link_dexter', -.076, 0, 0, 0, -.165143, 0, 0, 1, 0, 0, .22, 'revolute', effort, v],
          'shoulder_pitch_sinister':['shoulder_pitch_sinister', 'shoulder_bracket_sinister', 'shoulder_link_sinister', .076, 0, 0, 0, .165143, 0, 0, 1, 0, 0, .22, 'revolute', effort, v],
          'arm_upper_rot_dexter':['arm_upper_rot_dexter', 'shoulder_link_dexter', 'arm_upper_dexter', -.102, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'arm_upper_rot_sinister':['arm_upper_rot_sinister', 'shoulder_link_sinister', 'arm_upper_sinister', .102, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'elbow_dexter':['elbow_dexter', 'arm_upper_dexter', 'forearm_dexter', -.1550, 0, 0, 0, 0, 0, 0, 0, 1, 0, .22, 'revolute', effort, v],
          'elbow_sinister':['elbow_sinister', 'arm_upper_sinister', 'forearm_sinister', .1550, 0, 0, 0, 0, 0, 0, 0, 1, 0, .22, 'revolute', effort, v],
          'wrist_rot_dexter':['wrist_rot_dexter', 'forearm_dexter', 'wrist_rot_link_dexter', -.1, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'wrist_rot_sinister':['wrist_rot_sinister', 'forearm_sinister', 'wrist_rot_link_sinister', .1, 0, 0, 0, 0, 0, 0, 0, 1, 0, .22, 'revolute', effort, v],
          'wrist_pitch_dexter':['wrist_pitch_dexter', 'wrist_rot_link_dexter', 'wrist_ball_dexter', -.055, 0, 0, 0, 0, 0, 0, 1, 0, 0, .22, 'revolute', effort, v],
          'wrist_pitch_sinister':['wrist_pitch_sinister', 'wrist_rot_link_sinister', 'wrist_ball_sinister', .055, 0, 0, 0, 0, 0, 0, 0, 1, 0, .22, 'revolute', effort, v],
          'wrist_yaw_dexter':['wrist_yaw_dexter', 'wrist_ball_dexter', 'hand_dexter', -.05, 0, 0, 0, 0, 0, 0, 1, 0, 0, .22, 'revolute', effort, v],
          'wrist_yaw_sinister':['wrist_yaw_sinister', 'wrist_ball_sinister', 'hand_sinister', .05, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          
          'hip_yaw_dexter':['hip_yaw_dexter', 'torso_lower', 'hip_bracket_dexter', -.074255, 0, -.05657, 0, -.785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_yaw_sinister':['hip_yaw_sinister', 'torso_lower', 'hip_bracket_sinister', .074255, 0, -.05657, 0, .785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_roll_dexter':['hip_roll_dexter', 'hip_bracket_dexter', 'hip_motor_link_dexter', -.09714, 0, -.08195, 0, .785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_roll_sinister':['hip_roll_sinister', 'hip_bracket_sinister', 'hip_motor_link_sinister', .09714, 0, -.08195, 0, -.785398, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_pitch_dexter':['hip_pitch_dexter', 'hip_motor_link_dexter', 'thigh_dexter', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'hip_pitch_sinister':['hip_pitch_sinister', 'hip_motor_link_sinister', 'thigh_sinister', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'knee_dexter':['knee_dexter', 'thigh_dexter', 'calf_dexter', -.005, -.0382, -.21666, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'knee_sinister':['knee_sinister', 'thigh_sinister', 'calf_sinister', .005, -.0382, -.21666, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_pitch_dexter':['ankle_pitch_dexter', 'calf_dexter', 'ankle_link_dexter', 0, .0547, -.31021, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_pitch_sinister':['ankle_pitch_sinister', 'calf_sinister', 'ankle_link_sinister', 0, .0547, -.31021, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_roll_dexter':['ankle_roll_dexter', 'ankle_link_dexter', 'foot_dexter', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
          'ankle_roll_sinister':['ankle_roll_sinister', 'ankle_link_sinister', 'foot_sinister', 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, .22, 'revolute', effort, v],
 
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
            'head':['None', 0, 0, -.10357, 0, 0, 0],
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
  
  link_children = {}
  for joint in joints:
     print('joint:{}'.format(joint))
     link_children[joints[joint][2]] = [joints[joint][1], joints[joint][3], joints[joint][4], joints[joint][5], joints[joint][6], joints[joint][7], joints[joint][8]]
  link_children['torso_lower'] = ['__model__', 0, 0, 0, 0, 0, 0]
  
  for child in link_children:
     print('parent:{} child:{}'.format(link_children[child], child))
  #link_children['torso_lower'] = 'world'
  #define STL colors and mesh locations, and any additional required properties
  #properties = {'Black':'0.0 0.0 0.0 1.0'}
  #for link in links:
  #   properties[link + '_mesh'] = 'file://{}/{}/{}.{}'.format(output_location, mesh_type, link, mesh_type.lower())
  #
  #TODO: Add plugins
  
  with open('{}/kora.sdf'.format(output_location), 'w') as outfile:
    header_text = create_header(outfile, robot_name, 'world') #Assumes fixed joint between robot & world
    outfile.write('\n\n')
    for link in links:
        link_text = create_link(outfile, link, *links[link], link_children[link])
        outfile.write('\n')
    outfile.write('\n')
    for joint in joints:    
        joint_text = create_joint(outfile, *joints[joint]) # unpack value
        outfile.write('\n')
    outfile.write('\n</model>\n</world>\n</sdf>')


  #os.system('ros2 run xacro xacro {}/kora.urdf.xacro > {}/kora.urdf'.format(output_location, output_location)) #generate urdf from xacro

run()
