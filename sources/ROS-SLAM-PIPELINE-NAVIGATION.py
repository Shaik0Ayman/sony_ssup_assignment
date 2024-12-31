import rospy
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from queue import PriorityQueue

# Global variables for SLAM map
current_map = None

class Node:
    def __init__(self, x, y, cost):
        self.x = x
        self.y = y
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

def map_callback(msg):
    """Callback to store the latest map data."""
    global current_map
    current_map = msg  # Save the OccupancyGrid map

def odom_callback(msg):
    """Extract robot position and orientation."""
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return (position.x, position.y, yaw)

def heuristic(a, b):
    """Heuristic function for A* (Euclidean distance)."""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def plan_path(start, goal, grid_map):
    """A* Path Planning Algorithm."""
    width = grid_map.info.width
    height = grid_map.info.height
    resolution = grid_map.info.resolution
    origin_x = grid_map.info.origin.position.x
    origin_y = grid_map.info.origin.position.y

    # Convert start and goal to grid coordinates
    start_grid = (int((start[0] - origin_x) / resolution), int((start[1] - origin_y) / resolution))
    goal_grid = (int((goal[0] - origin_x) / resolution), int((goal[1] - origin_y) / resolution))

    # Convert map data to a 2D numpy array
    grid = np.array(grid_map.data).reshape((height, width))

    # A* algorithm
    open_list = PriorityQueue()
    open_list.put((0, Node(start_grid[0], start_grid[1], 0)))
    came_from = {}
    cost_so_far = {start_grid: 0}

    while not open_list.empty():
        current = open_list.get()[1]

        if (current.x, current.y) == goal_grid:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-connectivity
            neighbor = (current.x + dx, current.y + dy)

            # Check bounds and obstacles
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                if grid[neighbor[1], neighbor[0]] != 0:  # Non-traversable cell
                    continue

                new_cost = cost_so_far[(current.x, current.y)] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal_grid)
                    open_list.put((priority, Node(neighbor[0], neighbor[1], new_cost)))
                    came_from[neighbor] = (current.x, current.y)

    # Reconstruct path
    current = goal_grid
    path = []
    while current != start_grid:
        path.append(current)
        current = came_from.get(current, start_grid)

    path.reverse()

    # Convert path to world coordinates
    if path:
        next_step = path[0]
        world_x = next_step[0] * resolution + origin_x
        world_y = next_step[1] * resolution + origin_y
        return (world_x, world_y)
    else:
        return goal

def compute_control(robot_position, target_position):
    """Compute control commands for navigation."""
    kp_linear = 1.0
    kp_angular = 1.5

    distance_to_target = math.sqrt((target_position[0] - robot_position[0])**2 + (target_position[1] - robot_position[1])**2)
    target_angle = math.atan2(target_position[1] - robot_position[1], target_position[0] - robot_position[0])

    angular_error = math.degrees(target_angle) - math.degrees(robot_position[2])
    angular_error = (angular_error + 180) % 360 - 180

    angular_output = kp_angular * angular_error
    linear_velocity = kp_linear * distance_to_target if abs(angular_error) < 10 else 0.0

    return linear_velocity, angular_output

def main():
    rospy.init_node('pipeline_navigation_slam')

    # Publishers and subscribers
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    # Target position for testing (dynamically updated using path planning)
    target_position = (5.0, 5.0)

    while not rospy.is_shutdown():
        if current_map is None:
            rospy.loginfo("Waiting for SLAM map...")
            rate.sleep()
            continue

        # Get current robot position
        robot_position = odom_callback(rospy.wait_for_message('/odom', Odometry))

        # Plan path using SLAM map
        planned_target = plan_path(robot_position, target_position, current_map)

        # Compute control commands
        linear_velocity, angular_velocity = compute_control(robot_position, planned_target)

        # Publish velocity commands
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        cmd_vel_pub.publish(cmd)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
