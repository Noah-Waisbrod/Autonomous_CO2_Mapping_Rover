import numpy as np
import mapping

# Assume the lidar data is stored in an array called 'lidar_map'
lidar_map = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [1, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                      [1, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]])

# Define a function to check if a given point is a valid target point
def is_valid_target(i, j):
    return lidar_map[i][j] != 1 and (i+j) % 10 == 0

# Create an empty adjacency list
graph = {i: [] for i in range(lidar_map.size) if is_valid_target(i // lidar_map.shape[1], i % lidar_map.shape[1])}

# Iterate over all valid target points and add edges to the graph
for i in graph.keys():
    row, col = i // lidar_map.shape[1], i % lidar_map.shape[1]
    for ni, nj in [(row-1, col), (row, col-1), (row+1, col), (row, col+1)]:
        if 0 <= ni < lidar_map.shape[0] and 0 <= nj < lidar_map.shape[1] and is_valid_target(ni, nj):
            j = ni * lidar_map.shape[1] + nj
            
            graph[i].append(j)

# Print the graph
print(graph)
# mapping.astar(0,0,graph)