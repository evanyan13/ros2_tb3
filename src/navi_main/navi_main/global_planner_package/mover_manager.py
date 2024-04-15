import rclpy
from rclpy.node import Node
import rclpy.logging as log
from threading import Timer

from .global_mover import GlobalMover
from .local_mover import LocalMover

logger = log.get_logger("mover_manager")

class MoverManager(Node):
    def __init__(self, planner):
        super().__init__('mover_manager')
        self.robot_pos = None
        self.local_mover = LocalMover(self.robot_pos, planner)
        self.global_mover = GlobalMover(self.robot_pos, planner)
        self.timer = None
        # logger.info("Mover Manager Initialised")

        self.check_path_availability()

    def check_path_availability(self):
        logger.info(f"Global Mover Path: {self.global_mover.current_path}")
        if self.global_mover.current_path:
            self.switch_to_global_mover()
        else:
            self.switch_to_local_mover()
        self.reset_timer()

    def switch_to_global_mover(self):
        if self.local_mover.is_active:
            self.local_mover.stop_moving()
            self.local_mover.is_active = False
        self.global_mover.is_active = True
        self.global_mover.follow_path()
        logger.info("Switching to global mover")

    def switch_to_local_mover(self):
        if self.global_mover.is_active:
            self.global_mover.stop_moving()
            self.global_mover.is_active = False
        self.local_mover.is_active = True
        self.local_mover.random_move()
        logger.info("Switching to local mover")

    def update_robot_pos(self):
        self.local_mover.robot_pos = self.robot_pos
        self.global_mover.robot_pos = self.robot_pos

    def reset_timer(self):
        if self.timer:
            self.timer.cancel()
        self.timer = Timer(10.0, self.check_path_availability)  # check every 10 seconds
        self.timer.start()

    def on_shutdown(self):
        if self.timer:
            self.timer.cancel()
        self.local_mover.on_shutdown()
        self.global_mover.on_shutdown()

        self.local_mover.destroy_node()
        self.global_mover.destroy_node()

