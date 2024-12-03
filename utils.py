import numpy as np
from scipy.interpolate import splprep, splev, BSpline

def cubic_spline_through_joint_angles(knots, num_points=100):
    """
    Creates a cubic polynomial spline passing through the given 7D joint angle knots,
    ensuring the first and last knot points are included in the interpolation.

    Parameters:
    knots (np.ndarray): An array of shape (N, 7), where N is the number of knots,
                        and each row represents a 7D vector of joint angles.
    num_points (int): Number of interpolated points to generate along the spline.

    Returns:
    np.ndarray: An array of shape (num_points, 7) representing the interpolated joint angles.
    """
    if not isinstance(knots, np.ndarray) or knots.shape[1] != 7:
        raise ValueError("Knots should be a numpy array of shape (N, 7)")

    # Initialize the interpolated path
    interpolated_path = np.zeros((num_points, knots.shape[1]))

    # Time parameter for the knots
    n = len(knots)
    t = np.linspace(0, 1, n)

    # Create a clamped knot vector for each joint
    degree = 3  # Cubic B-spline
    t_clamped = np.concatenate((
        np.full(degree, t[0]),  # Repeat the first knot
        t,                     # Original parameter values
        np.full(degree, t[-1]) # Repeat the last knot
    ))

    # Interpolate each joint angle independently
    for i in range(knots.shape[1]):
        spline = splprep([knots[:, i]], u=t, s=0, k=degree)
        interpolated_path[:, i] = splev(np.linspace(0, 1, num_points), spline[0])[0]

    return interpolated_path


def get_bspline_representation(knots):
    """
    Returns the B-spline representation (control points, knot vector, degree)
    for 7D joint angles.

    Parameters:
        knots (np.ndarray): An array of shape (N, 7), where N is the number of knots,
                            and each row represents a 7D vector of joint angles.

    Returns:
        tuple: (control_points, knot_vector, degree)
    """
    if not isinstance(knots, np.ndarray) or knots.shape[1] != 7:
        raise ValueError("Knots should be a numpy array of shape (N, 7)")

    # Number of knots
    n = len(knots)
    degree = 3  # Cubic B-spline

    # Create a uniform knot vector with clamping
    t = np.linspace(0, 1, n - degree + 1)  # Interior knots
    t = np.concatenate((
        [0] * degree,  # Clamped at the start
        t,
        [1] * degree   # Clamped at the end
    ))
    return knots, t, degree


def construct_discretized_bspline(knots, t, degree, num_points):
    # Interpolate each dimension independently
    interpolated_path = []
    u_fine = np.linspace(0, 1, num_points)

    for i in range(knots.shape[1]):
        # Fit a B-spline for each joint
        coeffs = knots[:, i]  # Use the knots as control points
        spline = BSpline(t, coeffs, degree)
        interpolated_path.append(spline(u_fine))

    return np.column_stack(interpolated_path)