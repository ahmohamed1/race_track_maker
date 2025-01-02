import os


def add_cones(points, color, file, stl_path, colorName):
    for i, point in enumerate(points):
            model_name = f"model_{colorName}_{i}"
            # color = colors  # Cycle through colors if fewer than points
            file.write(f'    <model name="{model_name}">\n')
            file.write(f'          <pose>{point[0]} {point[1]} 0.005 0 0 0</pose>\n')
            file.write('      <static>true</static>\n')
            file.write('      <link name="body">\n')
            file.write('        <visual name="visual">\n')
            file.write('          <geometry>\n')
            file.write('            <mesh>\n')
            file.write(f'              <uri>file://{stl_path}</uri>\n')
            file.write('              <scale>0.001 0.001 0.001</scale>\n')  # Adjust the scale values as needed
            file.write('            </mesh>\n')
            file.write('          </geometry>\n')
            file.write('          <material>\n')
            file.write(f'            <ambient>{color[0]} {color[1]} {color[2]} 1.0</ambient>\n')  # Ambient color
            file.write(f'             <diffuse>{color[0] * 1.6} {color[1] * 1.6} {color[2] * 1.6} 1.0</diffuse>\n')  # Diffuse color (slightly brighter)
            file.write('            <specular>0.5 0.5 0.5 1.0</specular>\n')  # Shiny appearance
            file.write('            <emissive>0.0 0.0 0.0 1.0</emissive>\n')  # No emission
            file.write('          </material>\n')
            file.write('        </visual>\n')
            file.write('      </link>\n')
            file.write('    </model>\n')


def create_gazebo_world(pointsList , stl_path, output_file, colors):
    with open(output_file, 'w') as f:
        # Basic Gazebo world XML structure
        f.write('<?xml version="1.0" ?>\n')
        f.write('<sdf version="1.6">\n')
        f.write('  <world name="default">\n')
        f.write('    <include>\n')
        f.write('      <uri>model://ground_plane</uri>\n')
        f.write('    </include>\n')
        f.write('    <include>\n')
        f.write('      <uri>model://sun</uri>\n')
        f.write('    </include>\n')
        
        add_cones(pointsList[0], colors[0], f, stl_path, "blue")
        add_cones(pointsList[1], colors[1], f, stl_path, "yellow")
    
        # Closing tags
        f.write('  </world>\n')
        f.write('</sdf>\n')

# def launch_gazebo(world_file):
#     os.system(f'gazebo {world_file}')

# # Example usage
# points = [(0, 0, 0), (1, 0, 0), (2, 1, 0)]  # Replace with your list of points
# colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # Red, Green, Blue
# stl_path = "cone_green.stl"  # Replace with your STL file path
# output_file = "my_world.world"

# create_gazebo_world(points, stl_path, output_file, colors)
# launch_gazebo(output_file)
