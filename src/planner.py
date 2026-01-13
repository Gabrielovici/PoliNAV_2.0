import heapq
import math
import numpy as np

class AStarPlanner:
    def __init__(self, grid, resolution, origin_x, origin_y):
        self.grid = grid
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.height = len(grid)
        self.width = len(grid[0])

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = (gx * self.resolution) + self.origin_x
        wy = (gy * self.resolution) + self.origin_y
        return wx, wy

    def find_nearest_walkable(self, start_gx, start_gy, max_radius=10):
        """ Cauta un punct liber in jurul celui blocat """
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    nx, ny = start_gx + dx, start_gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.grid[ny][nx] == 0:
                            return nx, ny
        return None

    def plan(self, start_x, start_y, goal_x, goal_y):
        start_node = self.world_to_grid(start_x, start_y)
        goal_node = self.world_to_grid(goal_x, goal_y)

        print(f"[PLANNER 4-WAY] Start: {start_node}, Goal: {goal_node}")

        # 1. VERIFICARE SI CORECTIE START
        if not (0 <= start_node[0] < self.width and 0 <= start_node[1] < self.height):
             return []
        if self.grid[start_node[1]][start_node[0]] == 1:
            print("[PLANNER] Robot in zid -> Caut punct liber...")
            new_start = self.find_nearest_walkable(start_node[0], start_node[1])
            if new_start: start_node = new_start
            else: return []

        # 2. VERIFICARE SI CORECTIE GOAL
        if not (0 <= goal_node[0] < self.width and 0 <= goal_node[1] < self.height):
             return []
        if self.grid[goal_node[1]][goal_node[0]] == 1:
            print(f"[PLANNER] Tinta in zid -> Caut punct liber...")
            new_goal = self.find_nearest_walkable(goal_node[0], goal_node[1])
            if new_goal: goal_node = new_goal
            else: return []

        # 3. ALGORITMUL A* (MANHATTAN - FARA DIAGONALE)
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        f_score = {start_node: self.heuristic(start_node, goal_node)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_node:
                return self.reconstruct_path(came_from, current)

            # --- SCHIMBARE MAJORA: DOAR 4 DIRECTII (SUS, JOS, STANGA, DREAPTA) ---
            # Am scos diagonalele ca sa nu mai faca zig-zag
            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                cost = 1.0 # Cost constant pentru miscare dreapta

                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[1]][neighbor[0]] == 1: continue

                    tentative_g_score = g_score[current] + cost
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_node)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print("[PLANNER] Nu exista drum!")
        return []

    def heuristic(self, a, b):
        # Distanta Manhattan (Pentru miscare in L)
        # Asta il incurajeaza sa mearga drept pe axe
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            wx, wy = self.grid_to_world(current[0], current[1])
            path.append((wx, wy))
            current = came_from[current]
        path.reverse()
        return path