import heapq
import rclpy.logging as log

from .global_map import GlobalMap
from .global_node import GlobalPlannerNode
from .cell import Cell

logger = log.get_logger("find_astar_path")

def find_astar_path(map: GlobalMap, start: GlobalPlannerNode, goal: GlobalPlannerNode) -> list:
    start_i, start_j = map.coordinates_to_indices(start.x, start.y)
    goal_i, goal_j = map.coordinates_to_indices(goal.x, goal.y)
    start_cell = Cell(start_i, start_j)
    goal_cell = Cell(goal_i, goal_j)

    logger.info(f"Finding path ({start_i}, {start_j}) -> ({goal_i}, {goal_j}) ... ")

    start_cell.g = 0
    start_cell.h = start_cell.calculate_heuristic_cell(goal_cell)
    start_cell.f = start_cell.h
    start_cell.parent = None

    open_set = []
    heapq.heappush(open_set, (start_cell.f, start_cell))
    visited_set = set()
    cells = {(start_cell.i, start_cell.j): start_cell}

    turning_cost = 2
    wall_penalty = 2

    while open_set:
        current_f, current = heapq.heappop(open_set)

        if (current.i, current.j) in visited_set:
            continue

        if current == goal_cell:
            path = current.backtrack_path()
            path_nodes = [map.indices_to_node(i, j) for i, j in path]
            return path_nodes

        visited_set.add((current.i, current.j))
        neighbours = current.get_neighbours_cells(map)

        for neighbour in neighbours:
            if (neighbour.i, neighbour.j) in visited_set or not map.is_indice_avail(neighbour.i, neighbour.j):
                continue

            # Determine if there is a turn
            if current.parent:
                parent_direction = (current.i - current.parent.i, current.j - current.parent.j)
                current_direction = (neighbour.i - current.i, neighbour.j - current.j)
                # print(f"parent direcion : {parent_direction}, current_direction: {current_direction}")
                if parent_direction != current_direction:
                    turn_cost = turning_cost
                else:
                    turn_cost = 0
            else:
                turn_cost = 0

            # Determine if there is a wall nearby
            wall_cost = 0
            for di in range(-1, 2):
                for dj in range(-1, 2):
                    ni, nj = neighbour.i + di, neighbour.j + dj
                    if not map.is_indice_avail(ni, nj):
                        wall_cost += wall_penalty / (abs(di) + abs(dj) + 1)

            tentative_g = current.g + 1 + turn_cost + wall_cost

            # print(f"tentative_g {tentative_g} turning cost: {turn_cost}")
            if (neighbour.i, neighbour.j) not in cells or tentative_g < neighbour.g:
                neighbour.parent = current
                neighbour.g = tentative_g
                neighbour.h = neighbour.calculate_heuristic_cell(goal_cell)
                neighbour.f = neighbour.g + neighbour.h

                cells[(neighbour.i, neighbour.j)] = neighbour
                heapq.heappush(open_set, (neighbour.f, neighbour))

    return None