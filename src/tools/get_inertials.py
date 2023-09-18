"""

Copyright (c) 2023 Kieran Aponte
This software is licensed under the MIT License.

"""

# pymeshlab cannot be used with Anaconda. You may want to install run this file directly from your system without using ROS2.

import pymeshlab
import os


mesh_path = '/home/sunset/sunset_ws/src/kora/src/kora_desc/OBJ/'
with open(mesh_path + 'geometry/geometry.csv', 'w') as outfile:
    outfile.write('Mesh Name;Inertia Tensor;Center of Mass\n') # Separating with ;
    for item in os.listdir(mesh_path):
        
        if item[-3:] == 'obj':
            print(item)
            ms = pymeshlab.MeshSet()
            ms.load_new_mesh(mesh_path + item)
            geometric_measures = ms.get_geometric_measures()
            inertia = geometric_measures['inertia_tensor']
            inertia = str(inertia).replace('\n', '')
            print(inertia)
            inertia = inertia.replace('[ ', '[')
            inertia = inertia.replace(' ', ',')
            inertia = inertia.replace(',,', ',')
            print(inertia)
            com = geometric_measures['center_of_mass']
            com = str(com).replace('  ', ' ')
            com = com.replace(' ', ',')
            
            outfile.write('{};{};{}\n'.format(item, inertia, com))
            #print('inertia tensor: {}\ncenter of mass: {}\n\n'.format(inertia, com))