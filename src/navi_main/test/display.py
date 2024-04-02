import csv
import numpy as np
import matplotlib.pyplot as plt

def read_occupancy_grid_from_csv(filename):
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        grid_data = np.array([list(map(int, row)) for row in reader])
    return grid_data

def display_occupancy_grid(grid_data):
    plt.imshow(grid_data, cmap='gray', interpolation='none')
    plt.title('Occupancy Grid')
    plt.colorbar(label='Occupancy value')
    plt.show()

# Usage example
filename = '/home/evanyan13/colcon_ws/src/navi_main/test/map.csv'  # Path to your CSV file
grid_data = read_occupancy_grid_from_csv(filename)
display_occupancy_grid(grid_data)
