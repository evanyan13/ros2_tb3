import heapq

from .global_map import GlobalMap
from .global_node import GlobalPlannerNode


def find_astar_path(map: GlobalMap, start: GlobalPlannerNode, goal: GlobalPlannerNode) -> list:
    """
    Find path from start to goal node using Astar Algorithm
    """
    # Check if start and goal nodes are valid
    if not (map.is_node_valid(start) and map.is_node_valid(goal)):
        print("Start or goal node is invalid")
        return []

    # Check if start and goal nodes are avail
    if not map.is_node_avail(start) or  not map.is_node_avail(goal):
        print("Start or goal node is occupied")
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
        _, current_node = heapq.heappop(open_nodes)
        open_set.remove((current_node.x, current_node.y))
        print(f"\nProcessing Node: ({current_node.x}, {current_node.y}), f: {current_node.f}")

        # Skip if current node has been visited before
        if (current_node.x, current_node.y) in visited_set:
            continue

        # Return when current node is the goal
        if current_node.equals(goal):
            print(f"Goal reached: ({current_node.x}, {current_node.y})")
            return current_node.backtrack_path()

        visited_set.add((current_node.x, current_node.y))

        for neighbour in current_node.generate_neighbours(map.resolution):
            n_index = neighbour.x, neighbour.y
            print(f"Checking neighbour: {n_index}")
            if n_index in visited_set:
                continue

            if map.is_node_valid(neighbour) and map.is_node_avail(neighbour):
                g_new = current_node.g + current_node.calculate_distance(neighbour)
                h_new = neighbour.calculate_distance(goal)
                f_new = g_new + h_new

                if (f_new < neighbour.f) or n_index not in open_set:
                    neighbour.g = g_new
                    neighbour.h = h_new
                    neighbour.f = f_new
                    neighbour.parent = current_node

                    if n_index not in open_set:
                        heapq.heappush(open_nodes, (neighbour.f, neighbour))
                        open_set.add(n_index)
                    else:
                        visited_set.add(n_index)

    print("Path not found")
    return []