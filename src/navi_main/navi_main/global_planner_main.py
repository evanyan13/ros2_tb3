import rclpy
from rclpy.executors import MultiThreadedExecutor
import rclpy.logging as log

from navi_main.global_planner_package.frontier_explorer import FrontierExplorer
from navi_main.global_planner_package.global_planner import GlobalPlanner


def main(args=None):
    rclpy.init(args=args)

    global_planner = GlobalPlanner()
    log.get_logger("global_planner_main").info("GlobalPlanner node created")

    if global_planner.map and global_planner.mover.robot_pos:
        global_map = global_planner.map
        global_node = global_planner.mover.robot_pos
        frontier_explorer = FrontierExplorer(global_map, global_node)
        log.get_logger("global_planner_main").info("FrontierExplorer node created")
    else:
        frontier_explorer = None
        log.get_logger("global_planner_main").warn("Map/Mover not initiated properly")
    
    global_mover = global_planner.mover

    executor = MultiThreadedExecutor()
    executor.add_node(global_planner)
    if frontier_explorer:
        executor.add_node(frontier_explorer)
    executor.add_node(global_mover)
    log.get_logger("global_planner_main").info("MultiThreadedExecutor created")

    try:
        log.get_logger("global_planner_main").info("Start executor spin")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        frontier_explorer.destroy_node()
        global_planner.destroy_node()        
        rclpy.shutdown()


if __name__ == '__main__':
    main()