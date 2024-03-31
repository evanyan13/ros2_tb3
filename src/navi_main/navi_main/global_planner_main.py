#!/usr/bin/env python3

import rclpy
from .global_planner_package.global_path_planner import GlobalPathPlanner

def main():
    rclpy.init()
    node = GlobalPathPlanner()

    print("Path Planner is running...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occured {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

