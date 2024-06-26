import rclpy
import threading
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import rotate
from rclpy.executors import MultiThreadedExecutor
import rclpy.logging as log

from navi_main.global_planner_package.frontier_explorer import FrontierExplorer
from navi_main.global_planner_package.global_planner import GlobalPlanner

logger = log.get_logger("global_planner_main")

def main(args=None):
    rclpy.init(args=args)
    
    global_planner = GlobalPlanner()
    # Spin GlobalPlanner in a separate thread to start processing callbacks
    threading.Thread(target=spin_node_in_thread, args=(global_planner,), daemon=True).start()
    
    is_ready = check_planner_ready(global_planner)
    if not is_ready:
        logger.error("GlobalPlanner not ready")

    frontier_explorer = FrontierExplorer(global_planner)
    mover = global_planner.mover

    executor = MultiThreadedExecutor()
    executor.add_node(global_planner)
    executor.add_node(frontier_explorer)
    executor.add_node(mover)
    logger.info("MultiThreadedExecutor created")

    try:
        logger.info("Start executor spin")
        if is_ready:
            while rclpy.ok():
                executor.spin_once()
                if global_planner.check_shutdown():
                    logger.info("Shutdown signal received, STOPPING NAVI")
                    break
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received. STOPPING")
        mover.stop_moving()
    finally:
        global_planner.map.generate_occupancy() # Generate occupancy map to csv

        mover.destroy_node()
        frontier_explorer.destroy_node()
        global_planner.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        logger.info("ROS Shutdown complete")


def spin_node_in_thread(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def check_planner_ready(global_planner):
    max_wait_time = 10
    wait_interval = 1
    elapsed_time = 0

    while not global_planner.planner_ready.is_set() and elapsed_time < max_wait_time:
        logger.info("Waiting for GlobalPlanner to become ready...")
        is_ready = global_planner.planner_ready.wait(timeout=wait_interval)

        if is_ready:
            logger.info("GlobalPlanner Ready")
            break

        elapsed_time += wait_interval
    return is_ready


if __name__ == '__main__':
    main()