import heapq

# def close_point(grid,visited_points):
#     grid_density = 0.025 #0.025m
#     visit_density = 0.5 #0.5m
#     resolution = visit_density/grid_density
#     points =[[],[]]
#     for i in grid:
#         for j in grid[i]:
#             if grid[i][j] ==1 or grid[i][j] ==-1:
#                 pass
#             if i%resolution== 0 and j%resolution == 0:
                
#                 points[0].append(i)
#                 points[1].append(j)
#     close = [0,0]
#     small = 30000000 #big number
#     for i in points:
#         for j in visited_points:
#             d = manhattan_distance(i,j)
#             if d < 0.025:
#                 points.pop(points.index(i))
#                 continue
#             if d<small:
#                 small = d
#                 close = i
#     return close
        

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

    




def astar(start, goal, grid):
    """
    A* algorithm implementation for 2D array without weightings.
    
    Parameters:
        - start: starting point (tuple of row and column indices)
        - goal: goal point (tuple of row and column indices)
        - grid: a 2D array representing the grid, where 0 denotes a passable cell and 1 denotes an impassable cell
    
    Returns:
        - A list of tuples representing the intermediate points along the shortest path from start to goal.
    """
    
    # Initialize the open and closed sets
    open_set = []
    closed_set = set()
    
    # Initialize the g-score and f-score dictionaries
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    
    # Add the start point to the open set
    heapq.heappush(open_set, (f_score[start], start))
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            # Reconstruct the path and return it
            path = [current]
            while current in g_score:
                current = g_score[current]
                path.append(current)
            path.reverse()
            return path
        
        closed_set.add(current)
        
        for neighbor in get_neighbors(current, grid):
            if neighbor in closed_set:
                continue
            
            tentative_g_score = g_score[current] + 1  # 1 is the cost of moving to a neighboring cell
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # If the goal is not reachable, return an empty list
    return []

def manhattan_distance(point1, point2):
    """
    Computes the Manhattan distance between two points.
    """
    x1, y1 = point1
    x2, y2 = point2
    return abs(x1 - x2) + abs(y1 - y2)

def get_neighbors(point, grid):
    """
    Returns the passable neighboring points of the given point in the grid.
    """
    row, col = point
    height, width = len(grid), len(grid[0])
    neighbors = []
    for d_row, d_col in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        neighbor_row, neighbor_col = row + d_row, col + d_col
        if 0 <= neighbor_row < height and 0 <= neighbor_col < width and grid[neighbor_row][neighbor_col] == 0:
            neighbors.append((neighbor_row, neighbor_col))
    return neighbors
