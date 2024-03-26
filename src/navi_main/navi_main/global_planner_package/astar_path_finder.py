import heapq
from .global_map import GlobalMap
from .global_planner_node import GlobalPlannerNode


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

    no_path = True
    open_nodes = []
    found_path = []
    visited_nodes = set()
    start.g = 0
    start.h = start.calculate_distance(goal)
    start.f = start.g + start.h
    heapq.heappush(open_nodes, (start.f, start))
 
    while open_nodes:
        current_node = heapq.heappop(open_nodes)[1]
        print(f"\nProcessing Node: ({current_node.x}, {current_node.y}), f: {current_node.f}")

        # Check if current node has been visited before
        if (current_node.x, current_node.y) in visited_nodes:
            continue

        if current_node.equals(goal):
            no_path = False
            print(f"Goal reached: ({current_node.x}, {current_node.y})")
            found_path = current_node.backtrack_path()
            break

        visited_nodes.add((current_node.x, current_node.y))

        for neighbour in current_node.generate_neighbours(map.resolution):
            print(f"Checking neighbour: ({neighbour.x}, {neighbour.y})")

            if map.is_node_valid(neighbour) and map.is_node_avail(neighbour):
                g_new = current_node.g + current_node.calculate_distance(neighbour)
                h_new = neighbour.calculate_distance(goal)
                f_new = g_new + h_new

                in_open_nodes = any(n.x == neighbour.x and n.y == neighbour.y for _, n in open_nodes)

                if (f_new < neighbour.f) or not in_open_nodes:
                    neighbour.g = g_new
                    neighbour.h = h_new
                    neighbour.f = f_new
                    neighbour.parent = current_node

                    if not in_open_nodes:
                        heapq.heappush(open_nodes, (neighbour.f, neighbour))
                    else:
                        visited_nodes.add((neighbour.x, neighbour.y))

    # Else returns empty list
    if no_path:
        print("Path not found")
        return []
    return found_path

def print_path(path: list):
    print([(n.x, n.y) for n in path])