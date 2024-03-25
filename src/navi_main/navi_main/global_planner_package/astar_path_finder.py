import heapq
from .global_map import GlobalMap
from . import pixel_tolerance

def find_astar_path(map: GlobalMap, start: float, goal: float) -> list:
    # To find path from start togoal node using Astar Algorithm
    open_node = []
    visited_node = []
    init_pos = (0.0, start)
    heapq.heappush(open_node, init_pos)
    tolerance = (pixel_tolerance - 1) * map.cresolution

    while open_node:
        current_node = heapq.heappop(open_node)[1]

        for neighbour in current_node.generate_neighbours(map.resolution):
            if map.is_node_free(neighbour):
                if neighbour.calculate_distance(goal) < tolerance:
                    neighbour.parent = current_node
                    return neighbour.backtrack_path()
                
                neighbour.g = current_node.g + neighbour.calculate_distance(current_node)
                neighbour.h = neighbour.calculate_distance(goal)
                neighbour.f = neighbour.g + neighbour.h
                neighbour.parent = current_node

                if not any(neighbour == node and f <= neighbour.f for f, node in open_node):
                    if not any(neighbour == node and node.f <= neighbour.f for node in visited_node):
                        heapq.heappush(open_node, (neighbour.f, neighbour))
        
        visited_node.append(current_node)

    # Else returns empty list
    return []