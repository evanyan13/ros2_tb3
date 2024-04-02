import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
import rclpy.logging as log

from navi_main.global_planner_package.frontier_explorer import FrontierExplorer
from navi_main.global_planner_package.global_planner import GlobalPlanner


def main(args=None):
    rclpy.init(args=args)

    global_planner = GlobalPlanner()

    # Spin GlobalPlanner in a separate thread to start processing callbacks
    threading.Thread(target=spin_node_in_thread, args=(global_planner,), daemon=True).start()
    is_ready = check_ready(global_planner)
    if not is_ready:
        log.get_logger("global_planner_main").error("GlobalPlanner not ready")
    
    frontier_explorer = FrontierExplorer(global_planner.map, global_planner.mover.robot_pos)
    global_mover = global_planner.mover

    executor = MultiThreadedExecutor()
    executor.add_node(global_planner)
    executor.add_node(frontier_explorer)
    executor.add_node(global_mover)
    log.get_logger("global_planner_main").info("MultiThreadedExecutor created")

    try:
        log.get_logger("global_planner_main").info("Start executor spin")
        executor.spin()
    except KeyboardInterrupt:
        log.get_logger("global_planner_main").info("KeyboardInterrupt received. STOPPING")
        pass
    finally:
        csv = global_planner.map.generate_occupancy() # Generate occupancy map to csv
        log.get_logger("global_planner_main").info(f"Occupancy Grid Generated {csv}")
        frontier_explorer.destroy_node()
        global_planner.destroy_node()        
        rclpy.shutdown()
        log.get_logger("global_planner_main").info("ROS Shutdown complete")


def spin_node_in_thread(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def check_ready(global_planner):
    max_wait_time = 10
    wait_interval = 1
    elapsed_time = 0

    while not global_planner.planner_ready.is_set() and elapsed_time < max_wait_time:
        log.get_logger("global_planner_main").info("Waiting for GlobalPlanner to become ready...")
        is_ready = global_planner.planner_ready.wait(timeout=wait_interval)

        if is_ready:
            log.get_logger("global_planner_main").info("GlobalPlanner Ready")
            break

        elapsed_time += wait_interval
    return is_ready


if __name__ == '__main__':
    main()