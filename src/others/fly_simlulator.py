from heapq import heappush, heappop
from typing import Tuple, List, Set

Point = Tuple[int, int, int]

class Grid3D:
    def __init__(self, size_x, size_y, size_z):
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.obstacles: Set[Point] = set()

    def in_bounds(self, p: Point) -> bool:
        x, y, z = p
        return (
            0 <= x < self.size_x and
            0 <= y < self.size_y and
            0 <= z < self.size_z
        )

    def is_free(self, p: Point) -> bool:
        return p not in self.obstacles

def add_building(grid: Grid3D, x_range, y_range, z_range):
    for x in x_range:
        for y in y_range:
            for z in z_range:
                grid.obstacles.add((x, y, z))

def heuristic(a: Point, b: Point) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

def neighbors(p: Point) -> List[Point]:
    x, y, z = p
    return [
        (x + 1, y, z),
        (x - 1, y, z),
        (x, y + 1, z),
        (x, y - 1, z),
        (x, y, z + 1),
        (x, y, z - 1),
    ]

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar_3d(grid: Grid3D, start: Point, goal: Point) -> List[Point]:
    open_set = []
    heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for nxt in neighbors(current):
            if not grid.in_bounds(nxt):
                continue
            if not grid.is_free(nxt):
                continue

            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                f = tentative_g + heuristic(nxt, goal)
                heappush(open_set, (f, nxt))

    return []


grid = Grid3D(20, 20, 10)

# 建筑：x=8~11, y=8~11, 高度 0~6
add_building(
    grid,
    range(8, 12),
    range(8, 12),
    range(0, 7)
)

start = (0, 0, 2)
goal = (18, 18, 2)

path = astar_3d(grid, start, goal)

print("Path length:", len(path))
for p in path:
    print(p)