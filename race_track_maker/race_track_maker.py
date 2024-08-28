import numpy as np
import matplotlib.pyplot as plt
import shapely.geometry as shp
from shapely.ops import unary_union
import csv
import os

class TrackGenerator:
    def __init__(self, track_width=4.0, wall_height=1.0, wall_thickness=0.1):
        self.track_width = track_width
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.track_points = []
        self.file_name = None

    def set_track_points(self, track_points, closed_loop=False):
        """
        Set track points and handle whether the track is closed or open.
        """
        if closed_loop and track_points[0] != track_points[-1]:
            # Ensure the track points form a closed loop
            track_points.append(track_points[0])
        self.track_points = track_points
        

    def plot_offset_polygons(self, buffer_value=0.03, coin_track=False):
        """
        Creates a polygon from a list of points and plots the original 
        and offset polygons (representing walls).
        
        Returns the outward and inward polygons.
        """
        track_line = shp.LineString(self.track_points)
        # Create offset lines
        outward_offset_line = track_line.parallel_offset(buffer_value, side='left')
        inward_offset_line = track_line.parallel_offset(buffer_value, side='right')

        # Handle the case where offset lines may not form valid polygons
        if outward_offset_line.is_empty:
            outward_offset_line = None
        if inward_offset_line.is_empty:
            inward_offset_line = None

        # Ensure both lines have enough points
        if outward_offset_line and outward_offset_line.length >= 4:
            outward_polygon = shp.Polygon(outward_offset_line)
        else:
            outward_polygon = None
            print("Warning: Not enough points to create an outward polygon.")
            return None, None
        
        if inward_offset_line and inward_offset_line.length >= 4:
            inward_polygon = shp.Polygon(inward_offset_line)
        else:
            inward_polygon = None
            print("Warning: Not enough points to create an inward polygon.")
            return None, None

        # Convert to polygons if offset lines are valid
        outward_polygon = shp.Polygon(outward_offset_line) if outward_offset_line else None
        inward_polygon = shp.Polygon(inward_offset_line) if inward_offset_line else None
        
        if coin_track:
          oo_points = self.split_line(outward_polygon)
          in_points = self.split_line(inward_polygon)

        # Convert to numpy arrays for plotting
        track_pts = np.array(track_line.coords)
        outward_pts = np.array(outward_polygon.exterior.coords) if outward_polygon else None
        inward_pts = np.array(inward_polygon.exterior.coords) if inward_polygon else None

        # Save the path
        file_name = self.file_name + ".csv"

        # Check if the file exists
        if os.path.exists(file_name):
            os.remove(file_name)  # Delete the file if it exists



        with open(file_name, 'w') as f:
          # using csv.writer method from CSV package
          write = csv.writer(f)
          write.writerows("x,y")
          write.writerows(self.track_points)

        # Plotting
        plt.plot(*track_pts.T, color='black', label='Original Track Line')
        if outward_pts is not None:
            plt.plot(*outward_pts.T, color='red', label='Left Wall (Outward Offset)')
        if inward_pts is not None:
            plt.plot(*inward_pts.T, color='green', label='Right Wall (Inward Offset)')
        
        if coin_track:
          # Extract the x and y coordinates from the MultiPoint object
          oo_x, oo_y = zip(*[(point.x, point.y) for point in oo_points.geoms])
          plt.plot(oo_x, oo_y, 'ro')
          # Extract the x and y coordinates from the MultiPoint object
          in_x, in_y = zip(*[(point.x, point.y) for point in in_points.geoms])
          plt.plot(in_x, in_y, 'go')
        plt.axis('equal')
        plt.legend()
        plt.show()

        return outward_offset_line, inward_offset_line


    '''
      This new function to divide the line based on the delta distance
      https://stackoverflow.com/questions/62990029/how-to-get-equally-spaced-points-on-a-line-in-shapely    
    ''' 
    def split_line(self, polygon_or_line, distance_delta=0.9):
        if isinstance(polygon_or_line, shp.Polygon):
            line = shp.LineString(polygon_or_line.exterior.coords)
        else:
            line = polygon_or_line

        distances = np.arange(0, line.length, distance_delta)
        points = [line.interpolate(distance) for distance in distances]
        
        # Ensure that the first and last points are included
        points = [line.interpolate(0)] + points + [line.interpolate(line.length)]

        multipoint = unary_union(points)
        return multipoint


    def generate_wall_segments(self, line):
        """
        Generates wall segments from an offset line.
        """
        wall_segments = []
        coords = list(line.coords)
        
        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i + 1]
            
            length = np.hypot(x2 - x1, y2 - y1)
            yaw = np.arctan2(y2 - y1, x2 - x1)
            
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            z = self.wall_height / 2

            wall_segments.append({
                'length': length,
                'yaw': yaw,
                'mid_x': mid_x,
                'mid_y': mid_y,
                'z': z,
                'thickness': self.wall_thickness,
                'height': self.wall_height
            })
        
        return wall_segments

    def generate_track_world(self,track_points, file_name_, closed_loop=False, track_width = 4, track_height = 1):
        print("AAAAAAAAAAA: ",file_name_)
        self.file_name = file_name_
        self.track_width = track_width
        self.wall_height = track_height
        self.set_track_points(track_points, closed_loop=closed_loop)
        """
        Generates a Gazebo world file based on track points and offset walls.
        """
        world_template = """<?xml version="1.0"?>
        <sdf version="1.6">
        <world name="default">
            <include>
            <uri>model://ground_plane</uri>
            </include>
            <include>
            <uri>model://sun</uri>
            </include>
            {walls}
        </world>
        </sdf>
        """

        wall_template = """
        <model name="wall_{index}">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <box>
                  <size>{length} {thickness} {height}</size>
                </box>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <box>
                  <size>{length} {thickness} {height}</size>
                </box>
              </geometry>
              <material>
                <script>
                  <uri>file://media/materials/scripts/gazebo.material</uri>
                  <name>Gazebo/Gray</name>
                </script>
              </material>
            </visual>
          </link>
          <pose>{x} {y} {z} 0 0 {yaw}</pose>
        </model>
        """

        walls = ""

        # Plot and get the offset lines
        outward_line, inward_line = self.plot_offset_polygons(buffer_value=self.track_width / 2.0)

        if outward_line:
            # Generate wall segments for left and right walls
            outward_wall_segments = self.generate_wall_segments(outward_line)
            inward_wall_segments = self.generate_wall_segments(inward_line)

            # Combine all wall segments into the final world file
            for i, wall_segment in enumerate(outward_wall_segments):
                walls += wall_template.format(index=f"{i}_outward",
                                              length=wall_segment['length'],
                                              thickness=wall_segment['thickness'],
                                              height=wall_segment['height'],
                                              x=wall_segment['mid_x'],
                                              y=wall_segment['mid_y'],
                                              z=wall_segment['z'],
                                              yaw=wall_segment['yaw'])

            for i, wall_segment in enumerate(inward_wall_segments):
                walls += wall_template.format(index=f"{i}_inward",
                                              length=wall_segment['length'],
                                              thickness=wall_segment['thickness'],
                                              height=wall_segment['height'],
                                              x=wall_segment['mid_x'],
                                              y=wall_segment['mid_y'],
                                              z=wall_segment['z'],
                                              yaw=wall_segment['yaw'])

            world_content = world_template.format(walls=walls)

            file_name_ = self.file_name +".world"
            print(file_name_)
            with open(file_name_, "w") as world_file:
                world_file.write(world_content)
        else:
            print("Unable to generate offset lines. Please check your track points.")

# # Example usage
# track_points = [(0, 0), (10, 0), (15, 5), (15, 10), (10, 15), (0, 15), (-5, 10)]  # Open loop

# # Create an instance of TrackGenerator
# track_generator = TrackGenerator(track_width=4.0)

# # Generate the Gazebo world file with the walls
# track_generator.generate_track_world(track_points, closed_loop=True)
