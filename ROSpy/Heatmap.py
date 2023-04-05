#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry, OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np

class HeatmapUpdater:
    def __init__(self):
        self.co2_sub = rospy.Subscriber('/co2_sensor', Range, self.co2_callback)
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.grid_sub = rospy.Subscriber('/map', OccupancyGrid, self.grid_callback)
        self.fig, self.ax = plt.subplots()
        self.heatmap = np.zeros((100, 100))
        self.grid = None
        self.cax = self.ax.imshow(self.heatmap, cmap='hot', interpolation='nearest')
        self.wall_cax = self.ax.imshow(np.zeros((100, 100)), cmap='binary', interpolation='nearest', alpha=0.5)
        plt.show(block=False)

    def co2_callback(self, data):
        # Get the current CO2 reading and the robot's position
        co2 = data.range
        x, y = self.current_position()

        # Convert the robot's position to heatmap coordinates
        ix, iy = int(round(x * 10)), int(round(y * 10))

        # Update the heatmap with the current CO2 reading
        self.heatmap[iy, ix] += co2

        # Update the color map and redraw the heatmap
        self.cax.set_data(self.heatmap)
        plt.draw()

    def pose_callback(self, data):
        self.current_pose = data.pose.pose

    def current_position(self):
        # Return the x, y position of the robot in meters
        return self.current_pose.position.x, self.current_pose.position.y

    def grid_callback(self, data):
        self.grid = data
        self.wall_cax.set_data(np.reshape(np.array(self.grid.data), (self.grid.info.width, self.grid.info.height)).T)
        plt.draw()

if __name__ == '__main__':
    rospy.init_node('heatmap_updater')
    hm_updater = HeatmapUpdater()
    rospy.spin()
