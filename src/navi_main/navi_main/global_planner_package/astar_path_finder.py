import heapq
from .global_map import GlobalMap
from .global_planner_node import GlobalPlannerNode
from . import pixel_tolerance

def find_astar_path(map: GlobalMap, start: GlobalPlannerNode, goal: GlobalPlannerNode) -> list:
    # To find path from start to goal node using Astar Algorithm
    open_node = []
    visited_node = []
    heapq.heappush(open_node, (start.f, start))
 
    while open_node:
        current_node = heapq.heappop(open_node)[1]

        if current_node == goal:
            return current_node.backtrack_path()

        for neighbour in current_node.generate_neighbours(map.resolution):
            if map.is_node_free(neighbour) and neighbour not in visited_node:
                neighbour.g = current_node.g + neighbour.calculate_distance(current_node)
                neighbour.h = neighbour.calculate_distance(goal)
                neighbour.f = neighbour.g + neighbour.h
                neighbour.parent = current_node

                heapq.heappush(open_node, (neighbour.f, neighbour))
        
        visited_node.append(current_node)

    # Else returns empty list
    return []