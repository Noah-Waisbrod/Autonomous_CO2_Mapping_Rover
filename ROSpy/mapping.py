import heapq
        

def close_point(grid, visited_points):
    '''Chatgpt version of my close point function \n grid: 2d array of points \n visited ppoints: list of visteded points'''
    grid_density = 0.025 #0.025m
    visit_density = 0.5 #0.5m
    resolution = int(round(visit_density / grid_density))
    points = set()
    for i, row in enumerate(grid):
        if i % resolution == 0:
            for j, val in enumerate(row):
                if j % resolution == 0 and val == 0:
                    points.add((i, j))
    close = None
    small = float('inf')
    for i in points:
        for j in visited_points:
            d = abs(i[0] - j[0]) + abs(i[1] - j[1])
            if d < 0.025:
                continue
            if d < small:
                small = d
                close = i
    return close

    




import heapq
import math

class Node:
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position

def find_path(start, end, grid):
    open_set = []
    closed_set = set()
    start_node = Node(start)
    end_node = Node(end)
    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)
        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node)

        for neighbor in get_neighbors(current_node.position, grid):
            if neighbor in closed_set:
                continue

            g = current_node.g + 1
            h = math.sqrt((neighbor.x - end.x)**2 + (neighbor.y - end.y)**2)
            neighbor_node = Node(neighbor, g, h, current_node)

            if neighbor_node in open_set:
                existing_node = open_set[open_set.index(neighbor_node)]
                if existing_node.g > neighbor_node.g:
                    existing_node.g = neighbor_node.g
                    existing_node.f = existing_node.g + existing_node.h
                    existing_node.parent = neighbor_node.parent
            else:
                heapq.heappush(open_set, neighbor_node)

    return None

def get_neighbors(position, grid):
    neighbors = []
    x, y = position.x, position.y
    if x > 0 and grid[x-1][y] != 1:
        neighbors.append(Point(x-1, y))
    if x < len(grid)-1 and grid[x+1][y] != 1:
        neighbors.append(Point(x+1, y))
    if y > 0 and grid[x][y-1] != 1:
        neighbors.append(Point(x, y-1))
    if y < len(grid[0])-1 and grid[x][y+1] != 1:
        neighbors.append(Point(x, y+1))
    return neighbors

def manhattan_distance(point1, point2):
    """
    Computes the Manhattan distance between two points.
    """
    x1, y1 = point1
    x2, y2 = point2
    return abs(x1 - x2) + abs(y1 - y2)


