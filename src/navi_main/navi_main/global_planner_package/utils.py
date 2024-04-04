import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

# Package wide parameters
pixel_tolerance = 1
MAP_PATH = '/home/evanyan13/colcon_ws/map.csv'

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
    print(f"Printing Path: {to_print}")


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