import heapq
import rclpy.logging as log

from .global_map import GlobalMap
from .global_node import GlobalPlannerNode


def find_astar_path(map: GlobalMap, start_node: GlobalPlannerNode, goal_node: GlobalPlannerNode) -> list:
    """
    Find path from start to goal node using Astar Algorithm
    """
    # Check if are at the destination
    if start_node.equals(goal_node):
        print("We are already at the destination")
        return [start]
    
    # Convert coordinates to indices
    start_i, start_j = map.coordinates_to_indices(start_node.x, start_node.y)
    goal_i, goal_j = map.coordinates_to_indices(goal_node.x, goal_node.y)

    start = GlobalPlannerNode(start_i, start_j, start_node.theta)
    goal = GlobalPlannerNode(goal_i, goal_j, goal_node.theta)

    log.get_logger("find_astar_path").info(f"Finding path for: ({start_i}, {start_j}) -> ({goal_i}, {goal_j}))")

    open_nodes = []
    open_set = set()
    visited_set = set()

    start.g = 0
    start.h = start.calculate_distance(goal)
    start.f = start.g + start.h
    heapq.heappush(open_nodes, (start.f, start))
    open_set.add((start.x, start.y))
 
    while open_nodes:
        _, curr_node = heapq.heappop(open_nodes)
        open_set.remove((curr_node.x, curr_node.y))
        print(curr_node.x, curr_node.y)
        occ_value = map.get_occupancy_value_by_indices(curr_node.x, curr_node.y)
        log.get_logger("find_astar_path").info(f"Processing Node: ({curr_node.x}, {curr_node.y}), f: {curr_node.f}, occ: {occ_value}")

        # Skip if current node has been visited before
        if (curr_node.x, curr_node.y) in visited_set:
            continue

        # Return when current node is the goal
        if curr_node.equals(goal):
            log.get_logger("find_astar_path").info(f"Goal reached: ({curr_node.x}, {curr_node.y})")
            return curr_node.backtrack_path()

        visited_set.add((curr_node.x, curr_node.y))

        for neighbour in curr_node.generate_neighbours(map.resolution):
            n_index = neighbour.x, neighbour.y
            if n_index in visited_set:
                continue

            if map.is_node_valid(neighbour) and map.is_node_avail(neighbour):
                g_new = curr_node.g + curr_node.calculate_distance(neighbour)
                h_new = neighbour.calculate_distance(goal)
                f_new = g_new + h_new

                if (f_new < neighbour.f) or n_index not in open_set:
                    neighbour.g = g_new
                    neighbour.h = h_new
                    neighbour.f = f_new
                    neighbour.parent = curr_node

                    if n_index not in open_set:
                        heapq.heappush(open_nodes, (neighbour.f, neighbour))
                        open_set.add(n_index)
                    else:
                        visited_set.add(n_index)

    return []