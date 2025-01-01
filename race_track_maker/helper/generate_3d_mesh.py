from stl import mesh
import numpy as np

def generate_3d_mesh(points, height=1.0, is_closed=False):
    """
    Generate a 3D mesh for given 2D points. 
    Closed shapes are treated as hollow boundaries.

    :param points: List of tuples [(x1, y1), (x2, y2), ...]
    :param height: Extrusion height for the mesh.
    :param is_closed: Boolean indicating if the shape is closed.
    :return: 3D mesh object.
    """
    points = np.array(points)
    num_points = len(points)

    if is_closed:
        points = np.vstack([points, points[0]])  # Close the loop if needed
    
    vertices = []
    faces = []

    # Create vertices for bottom and top surfaces
    for point in points:
        vertices.append([point[0], point[1], 0])  # Bottom layer
        vertices.append([point[0], point[1], height])  # Top layer

    # Create faces for sides
    for i in range(num_points if not is_closed else num_points - 1):
        bottom_start = 2 * i
        bottom_end = (2 * (i + 1)) % (2 * num_points)
        top_start = bottom_start + 1
        top_end = bottom_end + 1

        # Create two triangles per side
        faces.append([bottom_start, bottom_end, top_end])
        faces.append([bottom_start, top_end, top_start])

    # Generate numpy arrays for vertices and faces
    vertices = np.array(vertices)
    faces = np.array(faces)

    # Create mesh object
    mesh_data = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            mesh_data.vectors[i][j] = vertices[face[j], :]

    return mesh_data

def combine_meshes(mesh_list):
    """
    Combine multiple mesh objects into a single mesh.

    :param mesh_list: List of mesh.Mesh objects.
    :return: Combined mesh.Mesh object.
    """
    total_faces = sum(mesh.data.shape[0] for mesh in mesh_list)
    combined_mesh = mesh.Mesh(np.zeros(total_faces, dtype=mesh.Mesh.dtype))

    current_index = 0
    for mesh_obj in mesh_list:
        num_faces = mesh_obj.data.shape[0]
        combined_mesh.data[current_index:current_index + num_faces] = mesh_obj.data
        current_index += num_faces

    return combined_mesh

# Example usage
points1 = [(-1.4, 1.375), (-1.4, 1.4), (-1.375, 1.425), (-1.35, 1.45), (-1.3, 1.525)]
points2 = [(0.5, -0.5), (0.6, -0.6), (0.7, -0.7), (0.8, -0.8), (0.9, -0.9)]

# Generate two meshes
mesh1 = generate_3d_mesh(points1, height=1.0, is_closed=False)
mesh2 = generate_3d_mesh(points2, height=1.0, is_closed=False)

# Combine meshes into one
combined_mesh = combine_meshes([mesh1, mesh2])

# Save the combined mesh to a single STL file
combined_mesh.save('combined_output.stl')