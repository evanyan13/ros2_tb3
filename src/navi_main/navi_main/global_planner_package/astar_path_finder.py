import heapq
import rclpy.logging as log

from .global_map import GlobalMap
from .global_node import GlobalPlannerNode
from .cell import Cell
from .utils import plot_path_node

logger = log.get_logger("find_astar_path")

def find_astar_path(map: GlobalMap, start: GlobalPlannerNode, goal: GlobalPlannerNode) -> list:
    start_i, start_j = map.coordinates_to_indices(start.x, start.y)
    goal_i, goal_j = map.coordinates_to_indices(goal.x, goal.y)
    start_cell = Cell(start_i, start_j)
    goal_cell = Cell(goal_i, goal_j)
    logger.info(f"Finding path ({start_i}, {start_j}) -> ({goal_i}, {goal_j})")

    start_cell.g = 0
    start_cell.h = start_cell.calculate_heuristic(goal_cell)
    start_cell.f = start_cell.h

    open_set = []
    heapq.heappush(open_set, (start_cell.f, start_cell))
    visited_set = set()
    cells = {(start_cell.i, start_cell.j): start_cell}

    while open_set:
        current_f, current = heapq.heappop(open_set)

        if (current.i, current.j) in visited_set:
            continue

        if current == goal_cell:
            path = current.backtrack_path()
            logger.info(f"Path found (indice): {path}")
            path_nodes = [map.indices_to_node(i, j) for i, j in path]
            path_coord = [(node.x, node.y) for node in path_nodes]
            logger.info(f"Path found (coord): {path_coord}")
            return path_nodes

        visited_set.add((current.i, current.j))
        neighbours = current.get_neighbours(map)

        for neighbour in neighbours:
            if (neighbour.i, neighbour.j) in visited_set:
                continue

            tentative_g = current.g + 1
            if tentative_g < neighbour.g:
                neighbour.parent = current
                neighbour.g = tentative_g
                neighbour.h = neighbour.calculate_heuristic(goal_cell)
                neighbour.f = neighbour.g + neighbour.h

                cells[(neighbour.i, neighbour.j)] = neighbour
                heapq.heappush(open_set, (neighbour.f, neighbour))
                logger.info(f"Neighbour added to open_set: ({neighbour.i}, {neighbour.j})")
    
    logger.info(f"No path found")
    return []


# def find_astar_path(map: GlobalMap, start_node: GlobalPlannerNode, goal_node: GlobalPlannerNode) -> list:
#     """
#     Find path from start to goal node using Astar Algorithm
#     """
#     logger = log.get_logger("find_astar_path")

#     # Check if are at the destination
#     if start_node.equals(goal_node):
#         print("We are already at the destination")
#         return [start_node]
    
#     # Convert coordinates to indices
#     start = start_node.coord_node_to_indice_node(map)
#     goal = goal_node.coord_node_to_indice_node(map)
#     logger.info(f"Finding path for: ({start.x}, {start.y}) -> ({goal.x}, {goal.y}))")

#     open_nodes = []
#     open_set = set()
#     visited_set = set()

#     start.g = 0
#     start.h = start.calculate_distance(goal)
#     start.f = start.g + start.h
#     heapq.heappush(open_nodes, (start.f, start))
#     open_set.add((start.x, start.y))
 
#     while open_nodes:
#         _, curr_node = heapq.heappop(open_nodes)
#         open_set.remove((curr_node.x, curr_node.y))
#         occ_value = map.get_occupancy_value_by_indices(curr_node.x, curr_node.y)
#         # log.get_logger("find_astar_path").info(f"Processing Node: ({curr_node.x}, {curr_node.y}), f: {curr_node.f}, occ: {occ_value}")

#         # Skip if current node has been visited before
#         if (curr_node.x, curr_node.y) in visited_set:
#             continue

#         # Return when current node is the goal
#         if curr_node.equals(goal):
#             logger.info(f"Goal reached: ({curr_node.x}, {curr_node.y})")
#             indice_path = curr_node.backtrack_path()
#             coord_path = []
#             for node in indice_path:
#                     coord_node = node.indice_node_to_coord_node(map)
#                     coord_path.append(coord_node)
#             return coord_path

#         visited_set.add((curr_node.x, curr_node.y))

#         for neighbour in curr_node.generate_neighbours(1):
#             if (neighbour.x, neighbour.y) in visited_set:
#                 continue

#             if map.is_indice_valid(neighbour.x, neighbour.y) and map.is_indice_avail(neighbour.x, neighbour.y):
#                 g_new = curr_node.g + curr_node.calculate_distance(neighbour)
#                 h_new = neighbour.calculate_distance(goal)
#                 f_new = g_new + h_new

#                 if (f_new < neighbour.f) or (neighbour.x, neighbour.y) not in open_set:
#                     neighbour.g = g_new
#                     neighbour.h = h_new
#                     neighbour.f = f_new
#                     neighbour.parent = curr_node

#                     if (neighbour.x, neighbour.y) not in open_set:
#                         heapq.heappush(open_nodes, (neighbour.f, neighbour))
#                         open_set.add((neighbour.x, neighbour.y))

#     explored_path = curr_node.backtrack_path()
#     logger.info(f"No path found: {[(n.x, n.y) for n in explored_path]}")
#     return []