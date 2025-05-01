# Built-in Imports
import cv2
import numpy as np
from typing import List, Tuple, Optional

# Custom Library Imports
from scipy.interpolate import CubicSpline
import cvxpy as cp


# ROS Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


def create_bounding_boxes(
        track_image: np.ndarray,
        box_size: int = 50
) -> List[Tuple[int, int, int, int]]:
    """
    Creates bounding boxes over the track map to constrain waypoint sampling.

    Parameters:
        track_image: np.ndarray
            Grayscale image where track boundaries are black (0) and drivable area is white (255).
        box_size: int = 50
            Size of each square bounding box in pixels.

    Returns:
        boxes: List[Tuple[int, int, int, int]]
            List of bounding boxes as (x_min, y_min, x_max, y_max).
    """
    height, width = track_image.shape
    boxes = []

    for y in range(0, height, box_size):
        for x in range(0, width, box_size):
            x_max = min(x + box_size, width)
            y_max = min(y + box_size, height)
            region = track_image[y:y_max, x:x_max]
            if np.any(region == 255):  # Check if any drivable pixels exist
                boxes.append((x, y, x_max, y_max))

    return boxes


def sample_points_in_boxes(
        track_image: np.ndarray,
        boxes: List[Tuple[int, int, int, int]],
        num_samples_per_box: int = 5
) -> List[Tuple[float, float]]:
    """
    Randomly samples points within bounding boxes, ensuring they lie on the drivable area.

    Parameters:
        track_image: np.ndarray
            Grayscale image of the track.
        boxes: List[Tuple[int, int, int, int]]
            List of bounding boxes.
        num_samples_per_box: int = 5
            Number of points to sample per box.

    Returns:
        points: List[Tuple[float, float]]
            List of sampled (x, y) points.
    """
    points = []
    for box in boxes:
        x_min, y_min, x_max, y_max = box
        for _ in range(num_samples_per_box):
            for _ in range(10):  # Try multiple times to find a valid point
                x = np.random.uniform(x_min, x_max)
                y = np.random.uniform(y_min, y_max)
                # Check if the point is on the drivable area
                if (0 <= int(y) < track_image.shape[0] and
                        0 <= int(x) < track_image.shape[1] and
                        track_image[int(y), int(x)] == 255):
                    points.append((x, y))
                    break
    return points


def generate_smooth_spline(
        points: List[Tuple[float, float]],
        num_waypoints: int = 50
) -> np.ndarray:
    """
    Generates a smooth spline through sampled points.

    Parameters:
        points: List[Tuple[float, float]]
            List of (x, y) points to fit the spline.
        num_waypoints: int = 50
            Number of waypoints to generate on the spline.

    Returns:
        waypoints: np.ndarray
            Array of shape (num_waypoints, 2) containing (x, y) waypoints.
    """
    if len(points) < 2:
        return np.array([])

    # Sort points by x-coordinate for simplicity (assumes track is roughly horizontal)
    points = sorted(points, key=lambda p: p[0])
    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])

    # Create a cubic spline
    cs = CubicSpline(x, y, bc_type='clamped')

    # Generate evenly spaced waypoints
    x_new = np.linspace(x[0], x[-1], num_waypoints)
    y_new = cs(x_new)

    return np.vstack((x_new, y_new)).T


def optimize_waypoints(
    waypoints: np.ndarray,
    track_image: np.ndarray,
    max_iter: int = 100
) -> np.ndarray:
    """
    Optimizes waypoints using convex optimization to minimize path length and curvature.

    Parameters:
        waypoints: np.ndarray
            Initial waypoints of shape (N, 2).
        track_image: np.ndarray
            Grayscale image of the track.
        max_iter: int = 100
            Maximum number of optimization iterations.

    Returns:
        optimized_waypoints: np.ndarray
            Optimized waypoints of shape (N, 2).
    """
    N = waypoints.shape[0]
    X = cp.Variable((N, 2))  # Variables for x, y coordinates
    initial_waypoints = waypoints.copy()

    # Objective: Minimize path length
    path_length_terms = [cp.norm(X[i+1] - X[i], 2) for i in range(N-1)]
    path_length = cp.sum(path_length_terms)

    # Approximate curvature as the norm of the difference of consecutive segments
    curvature_terms = [
        cp.norm((X[i+2] - X[i+1]) - (X[i+1] - X[i]), 2)
        for i in range(N-2)
    ]
    curvature = cp.sum(curvature_terms) if curvature_terms else cp.Constant(0.0)

    # Combine objectives with a weight
    objective = cp.Minimize(path_length + 0.1 * curvature)

    # Constraints: Stay within track boundaries
    constraints = []
    for i in range(N):
        x, y = X[i, 0], X[i, 1]
        # Ensure points are within image bounds
        constraints.append(x >= 0)
        constraints.append(x <= track_image.shape[1] - 1)
        constraints.append(y >= 0)
        constraints.append(y <= track_image.shape[0] - 1)
        # Stay close to initial waypoints to maintain shape
        constraints.append(cp.norm(X[i] - initial_waypoints[i], 2) <= 10.0)

    # Solve the optimization problem
    problem = cp.Problem(objective, constraints)
    try:
        problem.solve(max_iter=max_iter)
        if problem.status == cp.OPTIMAL:
            optimized_waypoints = X.value
            # Ensure waypoints are on drivable area
            for i in range(N):
                x, y = optimized_waypoints[i]
                if (0 <= int(y) < track_image.shape[0] and
                    0 <= int(x) < track_image.shape[1] and
                    track_image[int(y), int(x)] != 255):
                    # Snap to nearest drivable pixel
                    optimized_waypoints[i] = initial_waypoints[i]
            return optimized_waypoints
        else:
            return initial_waypoints
    except Exception as e:
        print(f"Optimization failed: {e}")
        return initial_waypoints


class WaypointGenerator(Node):
    """
    Subclass of Node to generate and publish waypoints for Pure Pursuit.

    Generates optimal waypoints from a track map image and publishes them.

    Topics subscribed to:
        None

    Topics published to:
        waypoints: geometry_msgs/Point
            Sequence of waypoints for Pure Pursuit.
    """

    def __init__(self):
        super().__init__('waypoint_generator')
        self.publisher_ = self.create_publisher(Point, 'waypoints', 10)
        self.timer = self.create_timer(1.0, self.generate_and_publish_waypoints)
        self.track_image = None

    def load_track_image(self, image_path: str) -> None:
        """
        Loads and processes the track image.

        Parameters:
            image_path: str
                Path to the PNG image of the track.
        """
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().error("Failed to load track image")
            return
        self.track_image = image

    def generate_and_publish_waypoints(self) -> None:
        """
        Generates waypoints and publishes them.
        """
        if self.track_image is None:
            self.get_logger().warn("Track image not loaded")
            return

        # Generate waypoints
        boxes = create_bounding_boxes(self.track_image)
        sampled_points = sample_points_in_boxes(self.track_image, boxes)
        initial_waypoints = generate_smooth_spline(sampled_points)
        optimized_waypoints = optimize_waypoints(initial_waypoints, self.track_image)

        # Publish waypoints
        for wp in optimized_waypoints:
            point = Point()
            point.x = float(wp[0])
            point.y = float(wp[1])
            point.z = 0.0
            self.publisher_.publish(point)
            self.get_logger().info(f"Published waypoint: ({point.x}, {point.y})")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    # Example: Load a track image (replace with actual path)
    node.load_track_image('src/simple_drive/resource/racetrack.png')  # Assumes track.png exists
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
