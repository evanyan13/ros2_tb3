import csv
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


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