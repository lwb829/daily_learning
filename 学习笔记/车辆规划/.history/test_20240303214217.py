import numpy as np

def compute_equivalent_angle_and_axis(rotation_matrix):
    # Step 1: Extract the third column vector from the rotation matrix
    rotation_axis = rotation_matrix[:, 2]

    # Step 2: Compute equivalent rotation angle using trigonometric functions
    # For theta = 0 degrees case
    if np.allclose(rotation_axis, [0, 0, 1]):
        equivalent_angle = 0
        rotation_axis = [0, 0, 1]  # Choosing z-axis as the rotation axis
    # For theta = 180 degrees case
    elif np.allclose(rotation_axis, [0, 0, -1]):
        equivalent_angle = 180
        # Any axis perpendicular to the z-axis can be chosen
        rotation_axis = [1, 0, 0]  # Choosing x-axis as the rotation axis
    else:
        # Step 4: Determine rotation axis direction
        equivalent_angle = np.arccos(rotation_axis[2])
        rotation_axis = rotation_axis / np.sin(equivalent_angle)

        # Step 5: Compute equivalent rotation angle using trigonometric functions
        equivalent_angle = np.degrees(equivalent_angle)

    return equivalent_angle, rotation_axis


# Example usage
# Example rotation matrix
rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
equivalent_angle, rotation_axis = compute_equivalent_angle_and_axis(
    rotation_matrix)
print("Equivalent Rotation Angle:", equivalent_angle)
print("Equivalent Rotation Axis:", rotation_axis)
