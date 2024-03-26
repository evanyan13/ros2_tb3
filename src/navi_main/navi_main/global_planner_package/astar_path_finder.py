import heapq
from .global_map import GlobalMap
from .global_planner_node import GlobalPlannerNode
from . import pixel_tolerance

def find_astar_path(map: GlobalMap, start: GlobalPlannerNode, goal: GlobalPlannerNode) -> list:
    # To find path from start to goal node using Astar Algorithm
    open_nodes = []
    visited_nodes = set()
    heapq.heappush(open_nodes, (start.f, start))
 
    while open_nodes:
        current_node = heapq.heappop(open_nodes)[1]
        print(f"Processing Node: ({current_node.x}, {current_node.y}), f: {current_node.f}")

        if current_node.equals(goal):
            print("Goal reached")
            return current_node.backtrack_path()
        
        visited_nodes.add(current_node)

        for neighbour in current_node.generate_neighbours(map.resolution):
            print(f"Checking neighbour: ({neighbour.x}, {neighbour.y})")
            if neighbour in visited_nodes or not map.is_node_free(neighbour):
                print(f"Neighbour is unavailable: ({neighbour.x}, {neighbour.y})")
                continue # Skip visited nodes

            # Current costs for the neighbour node
            g_score = current_node.g + current_node.calculate_distance(neighbour)
            print(f"Neighbour is free: ({neighbour.x}, {neighbour.y})")

            if (g_score < neighbour.g) or (neighbour not in [n[1] for n in open_nodes]):
                neighbour.g = g_score
                neighbour.h = neighbour.calculate_distance(goal)
                neighbour.f = neighbour.g + neighbour.h
                neighbour.parent = current_node

                if neighbour not in [n[1] for n in open_nodes]:
                    heapq.heappush(open_nodes, (neighbour.f, neighbour))
                    print(f"Pushed to open_nodes: ({neighbour.x}, {neighbour.y}), f: {neighbour.f}")

            # print(f"Checking neighbour: ({neighbour.x}, {neighbour.y})")
            # if map.is_node_free(neighbour):
            #     print(f"Neighbour is free: ({neighbour.x}, {neighbour.y})")
            #     neighbour.g = current_node.g + neighbour.calculate_distance(current_node)
            #     neighbour.h = neighbour.calculate_distance(goal)
            #     neighbour.f = neighbour.g + neighbour.h
            #     neighbour.parent = current_node

            #     heapq.heappush(open_nodes, (neighbour.f, neighbour))
            #     print(f"Pushed to open_nodes: ({neighbour.x}, {neighbour.y}), f: {neighbour.f}")
            # else:
            #     print(f"Neighbour is unavailable: ({neighbour.x}, {neighbour.y})")


    # Else returns empty list
    print("Path not found")
    return []