import os

def create_gazebo_world(points, stl_path, output_file, colors):
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

        # Loop through each point and add an STL model at that location
        for i, point in enumerate(points):
            model_name = f"model_{i}"
            color = colors[i % len(colors)]  # Cycle through colors if fewer than points
            f.write(f'    <model name="{model_name}">\n')
            f.write(f'          <pose>{point[0]} {point[1]} {point[2] + 0.005} 0 0 0</pose>\n')
            f.write('      <static>true</static>\n')
            f.write('      <link name="body">\n')
            f.write('        <visual name="visual">\n')
            f.write('          <geometry>\n')
            f.write('            <mesh>\n')
            f.write(f'              <uri>file://{stl_path}</uri>\n')
            f.write('              <scale>0.001 0.001 0.001</scale>\n')  # Adjust the scale values as needed
            f.write('            </mesh>\n')
            f.write('          </geometry>\n')
            f.write('          <material>\n')
            f.write(f'            <ambient>{color[0]} {color[1]} {color[2]} 1.0</ambient>\n')  # Ambient color
            f.write(f'            <diffuse>{color[0] * 1.6} {color[1] * 1.6} {color[2] * 1.6} 1.0</diffuse>\n')  # Diffuse color (slightly brighter)
            f.write('            <specular>0.5 0.5 0.5 1.0</specular>\n')  # Shiny appearance
            f.write('            <emissive>0.0 0.0 0.0 1.0</emissive>\n')  # No emission
            f.write('          </material>\n')
            f.write('        </visual>\n')
            f.write('      </link>\n')
            f.write('    </model>\n')

        # Closing tags
        f.write('  </world>\n')
        f.write('</sdf>\n')

def launch_gazebo(world_file):
    os.system(f'gazebo {world_file}')

# Example usage
points = [(0, 0, 0), (1, 0, 0), (2, 1, 0)]  # Replace with your list of points
colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # Red, Green, Blue
stl_path = "cone_green.stl"  # Replace with your STL file path
output_file = "my_world.world"

create_gazebo_world(points, stl_path, output_file, colors)
launch_gazebo(output_file)
