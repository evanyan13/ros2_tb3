import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from std_msgs.msg import Header

class GridCellsPublisher(Node):
    def __init__(self):
        super().__init__('grid_cells_publisher')
        self.gridcell_publisher = self.create_publisher(GridCells, 'grid_cells', qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish at 1Hz

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cell_width = 1.0  # in meters
        cell_height = 1.0  # in meters
        cells = GridCells()
        cells.header = header
        cells.cell_width = cell_width
        cells.cell_height = cell_height

        # Generate some points to visualize
        cells.cells = [Point(x=float(x), y=float(y), z=0.0) for x in range(5) for y in range(5)]

        # Publish the GridCells message
        self.gridcell_publisher.publish(cells)
