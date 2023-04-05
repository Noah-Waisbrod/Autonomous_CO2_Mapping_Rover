#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion


def close_point(grid, visited_points):
    '''Returns the closest point in the grid to the robot's current position'''
    resolution = int(round(0.5 / grid.info.resolution))
    points = set()
    for i in range(grid.info.width):
        if i % resolution == 0:
            for j in range(grid.info.height):
                if j % resolution == 0 and grid.data[i + j * grid.info.width] == 0:
                    points.add((i, j))
    closest_point = None
    min_distance = float('inf')
    for point in points:
        for visited_point in visited_points:
            distance = abs(point[0] - visited_point.pose.position.x) + abs(point[1] - visited_point.pose.position.y)
            if distance < 0.025:
                continue
            if distance < min_distance:
                min_distance = distance
                closest_point = point
    return closest_point


class MapSubscriber:
    def __init__(self):
        self.map_data = OccupancyGrid()
        rospy.init_node('map_subscriber')
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, data):
        self.map_data = data


class PathSubscriber:
    def __init__(self):
        self.path_data = Path()
        rospy.init_node('path_subscriber')
        rospy.Subscriber('/trajectory', Path, self.path_callback)

    def path_callback(self, data):
        if len(data.poses) > 0:
            self.path_data = data


if __name__ == '__main__':
    rospy.init_node('movebase_goal_publisher')
    pub = rospy.Publisher('/move_base/goal', MoveBaseAction, queue_size=10)
    map_sub = MapSubscriber()
    path_sub = PathSubscriber()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Get the current map and path
        current_map = map_sub.map_data
        current_path = path_sub.path_data

        # Check if the map and path are available
        if current_map.header.stamp == rospy.Time(0) or current_path.header.stamp == rospy.Time(0):
            rate.sleep()
            continue

        # Find the closest point to the robot's current position
        visited_points = current_path.poses[:len(current_path.poses)//2]
        closest_point = close_point(current_map, visited_points)

        # Create the MoveBaseGoal message
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = current_map.header.frame_id
        goal.target_pose.pose.position = Point(
            closest_point[0] * current_map.info.resolution + current_map.info.origin.position.x,
            closest_point[1] * current_map.info.resolution + current_map.info.origin.position.y,
            0
        )
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        # Publish the goal message
        pub.publish(goal)

        # Sleep for a short time to avoid overloading the system
        rospy.sleep(0.1)
