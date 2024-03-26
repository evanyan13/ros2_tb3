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

    open_nodes = []
    visited_nodes = set()
    heapq.heappush(open_nodes, (start.f, start))
 
    while open_nodes:
        current_node = heapq.heappop(open_nodes)[1]
        print(f"\nProcessing Node: ({current_node.x}, {current_node.y}), f: {current_node.f}")

        visited_nodes.add(current_node)

        for neighbour in current_node.generate_neighbours(map.resolution):
            print(f"Checking neighbour: ({neighbour.x}, {neighbour.y})")

            if map.is_node_valid(neighbour) and map.is_node_avail(neighbour) and neighbour not in visited_nodes:
                if neighbour.equals(goal):
                    print(f"Neighbour ({neighbour.x, neighbour.y}) is the goal")
                    neighbour.parent = current_node
                    return neighbour.backtrack_path()
                else:
                    g_new = current_node.g + current_node.calculate_distance(neighbour)
                    h_new = neighbour.calculate_distance(goal)
                    f_new = g_new + h_new

                    if (f_new < neighbour.f) or (neighbour not in [n[1] for n in open_nodes]):
                        heapq.heappush(open_nodes, (f_new, neighbour))
                        neighbour.g = g_new
                        neighbour.h = h_new
                        neighbour.f = f_new
                        neighbour.parent = current_node
            else:
                print(f"Neighbour ({neighbour.x, neighbour.y}) is unavailable")

    # Else returns empty list
    print("Path not found")
    return []

def print_path(path: list):
    print([(n.x, n.y) for n in path])