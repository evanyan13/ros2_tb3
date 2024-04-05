import math
import csv
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats
from PIL import Image, ImageDraw

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


# Package wide parameters
pixel_tolerance = 1
MAP_PATH = '/home/evanyan13/colcon_ws/map.csv'
OCC_THRESHOLD = 41


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


def print_path(path: list):
    to_print = [(n.x, n.y) for n in path]
    print(f"[utils] Printing Path: {to_print}")


def read_occupancy_grid_from_csv(filename):
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        data = np.array([list(map(int, row)) for row in reader], dtype=np.int8)

    resolution=1.0
    origin=(0.0, 0.0, 0.0)
    height, width = data.shape

    grid = OccupancyGrid()
    grid.info = MapMetaData()
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin = Pose()
    grid.info.origin.position.x = origin[0]
    grid.info.origin.position.y = origin[1]
    grid.info.origin.position.z = 0.0
    grid.data = [int(value) for value in data.flatten()]

    return grid


def display_occupancy_grid(grid_data):
    plt.imshow(grid_data, cmap='gray', interpolation='none')
    plt.title('Occupancy Grid')
    plt.colorbar(label='Occupancy value')
    plt.show()


def display_with_frontier(grid_data, frontiers):
    cmap = plt.cm.gray
    norm = plt.Normalize(vmin=-1, vmax=100)
    plt.imshow(grid_data, cmap=cmap, norm=norm, origin='lower')

    frontier_x, frontier_y = zip(*frontiers) if frontiers else ([], [])
    plt.scatter(frontier_x, frontier_y, color='red', s=10)

    plt.title('Occupancy Grid with Frontiers')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.colorbar(label='Occupancy value')

    plt.show()


def plot_path_node(map, start, goal, path):
    # Convert path nodes to map coordinates
    x_coords, y_coords = zip(*[(node.x, node.y) for node in path])

    # Plot the occupancy grid
    cmap = plt.cm.gray
    norm = plt.Normalize(vmin=-1, vmax=100)
    plt.imshow(map.data.reshape((map.height, map.width)), cmap=cmap, norm=norm, origin='lower')
    plt.scatter(x_coords, y_coords, c='red')  # Path in red
    plt.scatter(start.x, start.y, c='blue')  # Start in blue
    plt.scatter(goal.x, goal.y, c='green')  # End in green
    plt.title("A* Path Finding")
    plt.show()


def plot_map_helper(map, msg, robot_pos, goal, path, visited):
    # create numpy array
    occdata = np.array(msg.data)
    # compute histogram to identify bins with -1, values between 0 and below 50, 
    # and values between 50 and 100. The binned_statistic function will also
    # return the bin numbers so we can use that easily to create the image 
    occ_bins = [-1, 0, 50, 100]
    occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
    # get width and height of map
    iwidth = msg.info.width
    iheight = msg.info.height

    grid_x, grid_y= map.coordinates_to_indices(robot_pos.x, robot_pos.y)

    # binnum go from 1 to 3 so we can use uint8
    # convert into 2D array using column order
    odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

    # set current robot location to 0
    odata[grid_y][grid_x] = 0
    # create image from 2D array using PIL
    img = Image.fromarray(odata)
    draw = ImageDraw.Draw(img)

    pixel_radius = 2
    draw.ellipse([grid_x - pixel_radius, grid_y - pixel_radius,
                  grid_x + pixel_radius, grid_y + pixel_radius])
    
    if goal:
        goal_x, goal_y = map.coordinates_to_indices(goal.x, goal.y)
        draw.ellipse([goal_x - pixel_radius, goal_y - pixel_radius,
                    goal_x + pixel_radius, goal_y + pixel_radius])
        
    print(f"Path received at map_helper: {path}")
    if path:
        path_pixels = [map.coordinates_to_indices(node.x, node.y) for node in path]
        draw.line(path_pixels, width=1)
    
    print(f"Visited received at map_helper: {path}")
    if visited:
        print("Visited Nodes being plotted")
        visited_radius = 1
        for vx, vy in visited:
            draw.ellipse([vx - visited_radius, vy - visited_radius,
                        vx + visited_radius, vy + visited_radius])

    return img
    