

#///todo sub to rplidar get sensor_msgs/LaserScan.msg
#todo take laser scan list and make map http://wiki.ros.org/hector_mapping
#todo map goes out, write A* to get positions 
#todo publish positions to topic

import heapq

# define a function to calculate the Manhattan distance between two points
def manhattan_distance(start, end):
    return abs(start[0] - end[0]) + abs(start[1] - end[1])

# define the A* algorithm function
def astar(start, end, graph):
    open_list = [(0, start)] # priority queue of nodes to visit
    closed_set = set() # set of visited nodes
    came_from = {} # dictionary to store each node's parent
    g_score = {start: 0} # cost of getting from start to a given node
    f_score = {start: manhattan_distance(start, end)} # estimated total cost of getting from start to the end

    while open_list:
        current = heapq.heappop(open_list)[1]
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        closed_set.add(current)

        for neighbor in graph[current]:
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + manhattan_distance(current, neighbor)

            if neighbor not in open_list:
                heapq.heappush(open_list, (tentative_g_score + manhattan_distance(neighbor, end), neighbor))
            elif tentative_g_score >= g_score[neighbor]:
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, end)

    return None

