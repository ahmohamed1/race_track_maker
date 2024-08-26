import os

def create_gazebo_world(points, stl_path, output_file):
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
            f.write(f'    <model name="{model_name}">\n')
            f.write(f'          <pose>{point[0]} {point[1]} {point[2] + 0.1} 0 0 0</pose>\n')
            f.write('      <static>true</static>\n')
            f.write('      <link name="body">\n')
            f.write('        <visual name="visual">\n')
            f.write('          <geometry>\n')
            f.write('            <mesh>\n')
            f.write(f'              <uri>file://{stl_path}</uri></mesh>')
            f.write('          </geometry>\n')
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
stl_path = "cone.stl"  # Replace with your STL file path
output_file = "my_world.world"

create_gazebo_world(points, stl_path, output_file)
launch_gazebo(output_file)
