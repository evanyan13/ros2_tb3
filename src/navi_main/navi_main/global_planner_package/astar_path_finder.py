import heapq
import rclpy.logging as log

from .global_map import GlobalMap
from .global_node import GlobalPlannerNode


def find_astar_path(map: GlobalMap, start: GlobalPlannerNode, goal: GlobalPlannerNode) -> list:
    """
    Find path from start to goal node using Astar Algorithm
    """
    # Check if start and goal nodes are valid
    # if not (map.is_node_valid(start) and map.is_node_valid(goal)):
    #     log.get_logger('find_astar_path').info("Start or goal node is invalid")
    #     return []

    # Check if start and goal nodes are avail
    if not map.is_node_avail(start) or not map.is_node_avail(goal):
        log.get_logger("find_astar_path").info(f"--------FAILED TO CHECK-------")
        start_index = map.coordinates_to_indices(start.x, start.y)
        goal_index = map.coordinates_to_indices(goal.x, goal.y)
        log.get_logger("find_astar_path").info(f"CHECK indices ({start_index}, {goal_index}) in map (h x w): ({map.height}, {map.width})")

        start_index_valid = map.is_node_valid_by_indices(*start_index)
        goal_index_valid = map.is_node_valid_by_indices(*goal_index)
        start_coord_valid = map.is_node_valid(start)
        goal_coord_valid = map.is_node_valid(goal)      
        log.get_logger("find_astar_path").info(f"CHECK valid...")
        log.get_logger("find_astar_path").info(f"Valid by indices: start: {start_index_valid}, goal: {goal_index_valid}")
        log.get_logger("find_astar_path").info(f"Valid by coord: start: {start_coord_valid}, goal: {goal_coord_valid}")

        start_index_occ = map.get_occupancy_value_by_indices(*start_index)
        goal_index_occ = map.get_occupancy_value_by_indices(*goal_index)
        start_coord_occ = map.get_occupancy_value_by_coordinates(start.x, start.y)
        goal_coord_occ = map.get_occupancy_value_by_coordinates(goal.x, goal.y) 
        log.get_logger("find_astar_path").info(f"CHECK occupancy...")
        log.get_logger("find_astar_path").info(f"Occ by indices: start: {start_index_occ}, goal: {goal_index_occ}")
        log.get_logger("find_astar_path").info(f"Occ by coord: start: {start_coord_occ}, goal: {goal_coord_occ}")
        log.get_logger("find_astar_path").info(f"--------CHECK COMPLETE-------")
        return []
    
    # Check if are at the destination
    if start.equals(goal):
        print("We are already at the destination")
        return []

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
        occ_value = map.get_occupancy_value_by_coordinates(curr_node.x, curr_node.y)
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
            # print(f"Checking neighbour: {n_index}")
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

    print("Path not found")
    return []