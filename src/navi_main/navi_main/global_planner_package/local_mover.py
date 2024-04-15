import rclpy
import numpy as np
import math
import cmath
import time
import threading
import rclpy.logging as log
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .utils import LINEAR_VEL, ANGULAR_VEL, STOP_DISTANCE, FRONT_ANGLES

logger = log.get_logger("local_mover")

class LocalMover(Node):
    def __init__(self, robot_pos, planner):
        super().__init__('local_mover')
        # initialize variables
        self.planner = planner
        self.robot_pos = robot_pos
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.laser_range = np.array([])
        self.is_active = False

        # create subscription to track lidar
        self.loval_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        # create publisher for moving TurtleBot
        self.local_vel_publisher = self.create_publisher(Twist,'cmd_vel',10)

        self.move_timer = threading.Timer(1.0, self.random_move).start()

    def scan_callback(self, msg):
        logger.info("SCAN CALLBACK LOCAL MOVER")
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

        logger.info(f"Laser range received with {self.laser_range.size} measurements.")
        if np.isnan(self.laser_range).all():
            logger.info("All laser range data are NaN.")
        elif np.count_nonzero(~np.isnan(self.laser_range)) == 0:
            logger.info("Laser range contains no valid measurements.")

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # create Twist object
        twist = Twist()

        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # logger.info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        # logger.info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * ANGULAR_VEL
        # start rotation
        self.local_vel_publisher.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            # rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # logger.info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        # logger.info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.local_vel_publisher.publish(twist)
        logger.info(f'Complete rotation')

    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            logger.info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            logger.info('No data!')

        logger.info(f'Rotating robot by {lr2i}')
        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        logger.info('Start moving')
        twist = Twist()
        twist.linear.x = LINEAR_VEL
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.local_vel_publisher.publish(twist)

    def stop_moving(self):
        logger.info('STOP MOVING')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        time.sleep(1)
        self.local_vel_publisher.publish(twist)

    def random_move(self):
        self.pick_direction()

        self.is_active = True

        if self.is_active:
            if self.laser_range.size != 0:
                # check distances in front of TurtleBot and find values less
                # than stop_distance
                lri = (self.laser_range[FRONT_ANGLES]<float(STOP_DISTANCE)).nonzero()
                # self.get_logger().info('Distances: %s' % str(lri))

                # if the list is not empty
                if(len(lri[0])>0):
                    # stop moving
                    self.stop_moving()
                    # find direction with the largest distance from the Lidar
                    # rotate to that direction
                    # start moving
                    self.pick_direction()

            # allow the callback functions to run
            # rclpy.spin_once(self)
        else:
            return

    def on_shutdown(self):
        self.stop_moving()
        self.is_active = False  # Stop the thread from doing further work
        if self.move_timer:
            self.move_timer.cancel()  # Ensure timer is stopped
        self.local_vel_publisher.destroy()
        self.loval_scan_subscriber.destroy()
