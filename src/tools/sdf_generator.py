#!/usr/bin/env python3

"""

Copyright (c) 2023 Kieran Aponte
This software is licensed under the MIT License.

"""


#This ROS2 executable defines joints, links, properties, etc for a ROS2 robot and outputs an SDF file

import os
import pandas as pd
import ast

allow_sensing = True # Debugging purposes

def create_header(outfile, robot_name, world_name): 
    
    text = '''<?xml version="1.0"?>
<sdf version="1.9">
  

<!-- Built using SDF Generator: https://github.com/Miimiikiu/kora_tools.git -->
<!-- Editing this SDF file by hand is not recommended. -->'''

    #for prop in properties:
    #    text += '''\n<xacro:property name="{}" value="{}"/>'''.format(prop, properties[prop])
    text += '''\n\n
  <world name="{}">
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
        <real_time_factor>100.0</real_time_factor>
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
'''.format(world_name)
    if allow_sensing == True:
        text += '''
    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">
      <!-- ogre2 not working with just the MESA_GL_VERSION_OVERRIDE=3.3 trick -->
      <render_engine>ogre</render_engine>
    </plugin>
    '''
    text += '''



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

    <gui>
      <fullscreen>true</fullscreen>
      <camera name='Camera'>
        <pose>10 -2 -2 0 0 0</pose>
      </camera>
    </gui>
    
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
    <!--
    <include>
      <static>true</static>
      <name>Electrical Box1</name>
      <pose>0 1.5 2 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/Electrical Box</uri>
    </include>

    <include>
      <static>true</static>
      <name>Electrical Box2</name>
      <pose>0 -1 1 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/Electrical Box</uri>
    </include>

    <include>
      <static>true</static>
      <name>Electrical Box3</name>
      <pose>2 1.5 2 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/Electrical Box</uri>
    </include>

    <include>
      <static>true</static>
      <name>Electrical Box4</name>
      <pose>-2 1.5 2 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/Electrical Box</uri>
    </include>-->

    <model name="{}" canonical_link='torso_lower'>
      <static>true</static>
      <plugin filename="libignition-gazebo-joint-state-publisher-system.so" name="ignition::gazebo::systems::JointStatePublisher"/>

      
      <pose relative_to='world'>0 0 1 0 0 0</pose> 


    '''.format(robot_name)
    outfile.write(text)
    return(text)

def create_link(outfile, link_name, link_data, desc_path, inertia_scale=1, visual_scale=1, collision_scale=1, collision_mesh_type='STL'):
    if collision_mesh_type == 'STL':
       stl_type = 'Binary_'
    else:
       stl_type = ''
    
    text = '''      <link name="{}">
        <pose relative_to='{}'>{} {} {} {} {} {}</pose>
        <!--<pose>[] [] [] [] [] []</pose>-->
        <visual name='visual_{}'>
          <geometry>
            <mesh>
              <uri>{}</uri>
              <scale>{} {} {}</scale>
            </mesh>
          </geometry>
        </visual>'''.format(link_name, link_data['parent'], float(link_data['ox']), float(link_data['oy']), float(link_data['oz']), float(link_data['roll']), float(link_data['pitch']), float(link_data['yaw']), 
                            link_name, desc_path + '/visuals/OBJ/' + link_name + '.obj', visual_scale, visual_scale, visual_scale)

    if link_data['color'] != 'None':
      text += '''
        <material name="{}"/>
        <color rgba="{}"/>
'''.format(link_data['color'], '${' + str(link_data['color']) + '}')
    df = pd.read_csv(desc_path + '/geometry/geometry.csv', sep=';') # get data for robot inertia values
    inertials_string = df.loc[df['Mesh Name'] == '{}.obj'.format(link_name), 'Inertia Tensor'].values[0]
    inertials = ast.literal_eval(inertials_string)
    mass_string = df.loc[df['Mesh Name'] == '{}.obj'.format(link_name), 'Mass'].values[0]
    mass = float(mass_string)
    #print(mass)
    text += '''        <collision name='collision_{}'>
          <geometry>
            <mesh>
              <uri>{}</uri>
              <scale>{} {} {}</scale>
            </mesh>
          </geometry>
        </collision>
        <inertial> <!--inertial properties of the link mass, inertia matix-->
          <mass>{}</mass>
          <inertia>
            <ixx>{}</ixx>
            <ixy>{}</ixy>
            <ixz>{}</ixz>
            <iyy>{}</iyy>
            <iyz>{}</iyz>
            <izz>{}</izz>
          </inertia>
        </inertial>
'''.format(link_name, desc_path + '/collisions/{}{}/'.format(stl_type, collision_mesh_type.upper()) + link_name + '_collision.{}'.format(collision_mesh_type.lower()), 
           collision_scale, collision_scale, collision_scale, mass, inertials[0][0]*inertia_scale, inertials[0][1]*inertia_scale, inertials[0][2]*inertia_scale, inertials[1][1]*inertia_scale, inertials[1][2]*inertia_scale, inertials[2][2]*inertia_scale)
      
    if link_data['sensor'] != {} and allow_sensing == True: #https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Camera#gazebo_ros_depth_camera
      text += '''<!-- {} -->
      
              <sensor type="camera" name="camera_sensor_color">
          

          <!-- Set always_on only sensor, not on plugin -->
          <always_on>0</always_on>

          <!-- Set update_rate only sensor, not on plugin -->
          <update_rate>1</update_rate>

          <camera name="camera_color">
            
            <distortion>
              <k1>0.1</k1>
              <k2>0.2</k2>
              <k3>0.3</k3>
              <p1>0.4</p1>
              <p2>0.5</p2>
              <center>0.5 0.5</center>
            </distortion>
          </camera>

          <!-- Use camera, not camera_triggered -->
          <plugin name="plugin_name" filename="libgazebo_ros_camera.so">
            <!-- Change namespace, camera name and topics so -
                 * Images are published to: /custom_ns/custom_camera/custom_image
                 * Camera info is published to: /custom_ns/custom_camera/custom_info
            -->
            <ros>
              <namespace>camera_color</namespace>
              <remapping>image_raw:=img</remapping>
              <remapping>camera_info:=info</remapping>
            </ros>

            <!-- Set camera name. If empty, defaults to sensor name (i.e. "sensor_name") -->
            <camera_name>camera_color</camera_name>

            <!-- Set TF frame name. If empty, defaults to link name (i.e. "link_name") -->
            <frame_name>realsense</frame_name>

            <hack_baseline>0.07</hack_baseline>

            <!-- No need to repeat distortion parameters or to set autoDistortion -->
          </plugin>
        </sensor>

        
        <sensor type="depth" name="depth_sensor">
          

          <!-- Set always_on only sensor, not on plugin -->
          <always_on>0</always_on>

          <!-- Set update_rate only sensor, not on plugin -->
          <update_rate>1</update_rate>

          <camera name="depth_camera">
            
            <distortion>
              <k1>0.1</k1>
              <k2>0.2</k2>
              <k3>0.3</k3>
              <p1>0.4</p1>
              <p2>0.5</p2>
              <center>0.5 0.5</center>
            </distortion>
          </camera>

          <plugin name="plugin_name_depth" filename="libgazebo_ros_camera.so">
            <!-- Change namespace, camera name and topics so -
                 * Raw images are published to: /custom_ns/custom_camera/custom_image
                 * Depth images are published to: /custom_ns/custom_camera/custom_image_depth
                 * Raw image camera info is published to: /custom_ns/custom_camera/custom_info_raw
                 * Depth image camera info is published to: /custom_ns/custom_camera/custom_info_depth
                 * Point cloud is published to: /custom_ns/custom_camera/custom_points
            -->
            <ros>
              <namespace>depth_camera</namespace>
              <remapping>depth_camera/image_raw:=depth_camera/custom_image</remapping>
              <remapping>depth_camera/image_depth:=depth_camera/custom_image_depth</remapping>
              <remapping>depth_camera/camera_info:=depth_camera/custom_info_raw</remapping>
              <remapping>depth_camera/camera_info_depth:=depth_camera/custom_info_depth</remapping>
              <remapping>depth_camera/points:=depth_camera/custom_points</remapping>
            </ros>

            <!-- Set camera name. If empty, defaults to sensor name (i.e. "sensor_name") -->
            <camera_name>depth_camera</camera_name>

            <!-- Set TF frame name. If empty, defaults to link name (i.e. "link_name") -->
            <frame_name>realsense</frame_name>

            <hack_baseline>0.07</hack_baseline>

            <!-- No need to repeat distortion parameters or to set autoDistortion -->

             <min_depth>0.001</min_depth>
          </plugin>
        </sensor>'''.format(link_data['sensor']['name']) #https://automaticaddison.com/how-to-add-a-depth-camera-to-an-sdf-file-for-gazebo/
    text += '''      </link>
    '''

    
    outfile.write(text)
    return(text)

def create_joint(outfile, joint_name, parent, child, ox, oy, oz, roll, pitch, yaw, ax, ay, az, limit_lower, limit_upper, joint_type, effort, velocity):
    text = '''      <joint name="{}" type="{}">
        <parent>{}</parent>
        <child>{}</child>
        <!--<pose relative_to='{}'>[] [] [] [] [] []</pose>-->
        <pose>{} {} {} {} {} {}</pose>


      
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
  visual_mesh_type = 'OBJ' #OBJ or STL
  collision_mesh_type = 'STL'
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
          'neck_z':['neck_z', 'neck', 'head', 0, 0, .12975, 0, 0, 0, 0, 0, 1, 0, .22, 'revolute', effort, v],
          'realsense_joint':['realsense_joint', 'head', 'realsense', 0, -.2, .2, 0, 0, 0, 1, 0, 0, 0, .22, 'fixed', effort, v],

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
  if visual_mesh_type == 'STL':                                                         
    links = {'torso_lower':'Black',
            'torso_mid':'Black',
            'torso_upper':'Black'
            }
  elif visual_mesh_type == 'OBJ':
  #{link_name:[color, origin x, origin y, origin z, roll, pitch, yaw]}
    #'neck':['None', 0, 0, .10357, 0, 0, 0],
    #        'neck_connection':['None', 0, 0, 0, 0, 0, 0],
    #        'head':['None', 0, 0, -.10357, 0, 0, 0],
    links = {'torso_lower':{},
            'torso_mid':{},
            'torso_upper':{},
            'backpack':{},
            'neck':{},
            'neck_connection':{},
            'head':{},
            'realsense':{},

            'shoulder_bracket_dexter':{},
            'shoulder_link_dexter':{},
            'arm_upper_dexter':{},
            'forearm_dexter':{},
            'wrist_rot_link_dexter':{},
            'wrist_ball_dexter':{},
            'hand_dexter':{},

            'shoulder_bracket_sinister':{},
            'shoulder_link_sinister':{},
            'arm_upper_sinister':{},
            'forearm_sinister':{},
            'wrist_rot_link_sinister':{},
            'wrist_ball_sinister':{},
            'hand_sinister':{},

            'hip_bracket_dexter':{},
            'hip_motor_link_dexter':{},
            'thigh_dexter':{},
            'calf_dexter':{},
            'ankle_link_dexter':{},
            'foot_dexter':{},

            'hip_bracket_sinister':{},
            'hip_motor_link_sinister':{},
            'thigh_sinister':{},
            'calf_sinister':{},
            'ankle_link_sinister':{},
            'foot_sinister':{}
            
            }    
  
  sensors = {'realsense_sensor':{'link':'realsense'}}
  #link_children = {}
  #for joint in joints:
  #   print('joint:{}'.format(joint))
  #   link_children[joints[joint][2]] = [joints[joint][1], joints[joint][3], joints[joint][4], joints[joint][5], joints[joint][6], joints[joint][7], joints[joint][8]]
  #link_children['torso_lower'] = ['__model__', 0, 0, 0, 0, 0, 0]
  
  #for child in link_children:
  #   print('parent:{} child:{}'.format(link_children[child], child))
  #define STL colors and mesh locations, and any additional required properties
  #properties = {'Black':'0.0 0.0 0.0 1.0'}
  #for link in links:
  #   properties[link + '_mesh'] = 'file://{}/{}/{}.{}'.format(output_location, mesh_type, link, mesh_type.lower())
  #
  #TODO: Add plugins
  
  # Add data from joints into links. For link meshes with origin different from joint origin, this data may need to be edited afterward.
  for joint in joints: 
    links[joints[joint][2]]['ox'] = joints[joint][3]
    links[joints[joint][2]]['oy'] = joints[joint][4]
    links[joints[joint][2]]['oz'] = joints[joint][5]
    links[joints[joint][2]]['roll'] = joints[joint][6]
    links[joints[joint][2]]['pitch'] = joints[joint][7]
    links[joints[joint][2]]['yaw'] = joints[joint][8]
    links[joints[joint][2]]['parent'] = joints[joint][1]
    links[joints[joint][2]]['color'] = None
    links[joints[joint][2]]['sensor'] = {}
  links['torso_lower'] = {'ox':0, 'oy':0, 'oz':0, 'roll':0, 'pitch':0, 'yaw':0, 'parent':'__model__', 'color':None, 'sensor':{}}
  links['neck']['oz'] += .10357
  links['head']['oz'] -= .10357
  links['realsense']['sensor']['name'] = 'realsense_sensor'
  for link in links:
     print(link)
     print(links[link])


  inertia_scale = 100
  collision_scale = .001
  with open('{}/kora.sdf'.format(desc_path), 'w') as outfile:
    create_header(outfile, robot_name, 'kora_world') #Assumes fixed joint between robot & world
    outfile.write('\n\n')
    for link in links:
        create_link(outfile, link, links[link], desc_path, inertia_scale=inertia_scale, collision_scale=collision_scale, collision_mesh_type=collision_mesh_type)
        outfile.write('\n')
    outfile.write('\n')
    for joint in joints:    
        create_joint(outfile, *joints[joint]) # unpack joint
        outfile.write('\n')
    outfile.write('\n</model>\n</world>\n</sdf>')
  

  #os.system('ros2 run xacro xacro {}/kora.urdf.xacro > {}/kora.urdf'.format(desc_path, desc_path)) #generate urdf from xacro

run()
