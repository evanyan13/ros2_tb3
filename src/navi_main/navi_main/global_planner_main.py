import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from navi_main.global_planner_package.frontier_explorer import FrontierExplorer
from navi_main.global_planner_package.global_planner import GlobalPlanner


def main(args=None):
    rclpy.init(args=args)

    global_planner = GlobalPlanner()

    global_map = global_planner.map
    global_node = global_planner.mover.robot_pos
    frontier_explorer = FrontierExplorer(global_map, global_node)
    global_mover = global_planner.mover

    executor = MultiThreadedExecutor()
    executor.add_node(global_planner)
    executor.add_node(frontier_explorer)
    executor.add_node(global_mover)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        frontier_explorer.destroy_node()
        global_planner.destroy_node()        
        rclpy.shutdown()


if __name__ == '__main__':
    main()