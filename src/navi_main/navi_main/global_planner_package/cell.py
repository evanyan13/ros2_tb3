import math

from .utils import EXPLORER_STEPS

class Cell:
    def __init__(self, i, j, parent=None) -> None:
        self.i = i
        self.j = j
        self.parent = parent
        self.g = float('inf')
        self.h = 0.0
        self.f = float('inf')
    
    def calculate_heuristic_cell(self, goal):
        """
        Returns euclidean distance btw two nodes
        """
        return math.sqrt((self.i - goal.i) ** 2 + (self.j - goal.j) ** 2)
    
    def calculate_heuristic_man(self, goal):
        """
        Returns manhattan distance btw two nodes
        """
        return abs(self.i - goal.i) + abs(self.j - goal.j)
    
    def update_costs(self, g_cost, goal):
        """
        If new g cost is lower, update the node
        """
        if g_cost < self.g:
            self.g = g_cost
            self.f = self.g + self.h
            return True
        return False
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.i == other.i and self.j == other.j
    
    def __hash__(self):
        return hash((self.i, self.j))
    
    def get_neighbours_cells(self, map):
        """
        Get neighbours of 
        """
        neighbours = []
        step = EXPLORER_STEPS
        directions = [(step, 0), (0, -step), (-step, 0), (0, step),
                      (step, step), (step, -step), (-step, step), (-step, -step)]
        
        for di, dj in directions:
            ni, nj = self.i + di, self.j + dj
            if map.is_indice_avail(ni, nj):
                neighbours.append(Cell(ni, nj, self))
        return neighbours

    def backtrack_path(self):
        """"
        Backtrack to find path from start to current cell
        """
        path = []
        current = self
        while current:
            path.append((current.i, current.j))
            current = current.parent

        return path[::-1]



