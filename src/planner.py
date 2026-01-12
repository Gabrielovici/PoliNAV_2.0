import heapq
import numpy as np


class AStarPlanner:
    def __init__(self, grid, resolution, origin_x, origin_y):
        self.grid = grid
        self.resolution = resolution
        self.origin_x = origin_x #in config coltul stanga jos
        self.origin_y = origin_y
        self.height = len(grid)
        self.width = len(grid[0])

#din simulare in matrice - adica daca e la 2 metrii e coloana 4 din matrice
    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

#din matrice in simulare
    def grid_to_world(self, gx, gy):
        wx = (gx * self.resolution) + self.origin_x
        wy = (gy * self.resolution) + self.origin_y
        return wx, wy

#calculza drumul
    def plan(self, start_x, start_y, goal_x, goal_y):
        start_node = self.world_to_grid(start_x, start_y)
        goal_node = self.world_to_grid(goal_x, goal_y)

        #sa nu iasa din harta
        if not (0 <= start_node[0] < self.width and 0 <= start_node[1] < self.height):
            print("Planner: Startul e in afara hartii!")
            return []
        if not (0 <= goal_node[0] < self.width and 0 <= goal_node[1] < self.height):
            print("Planner: Destinatia e in afara hartii!")
            return []

        # Algoritmul A*
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        f_score = {start_node: self.heuristic(start_node, goal_node)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_node:
                return self.reconstruct_path(came_from, current)

            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                # Verificam limitele si obstacolele
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[1]][neighbor[0]] == 1:
                        continue

                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_node)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print("Planner: Nu am gasit drum!")
        return []

    def heuristic(self, a, b):
        # Distanta Manhattan
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            # Salvam coordonatele reale (in metri)
            wx, wy = self.grid_to_world(current[0], current[1])
            path.append((wx, wy))
            current = came_from[current]
        path.reverse()
        return path