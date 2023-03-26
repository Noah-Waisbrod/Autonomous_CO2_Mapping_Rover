import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Int32
from tkinter import *

class CO2SensorNode:
    def __init__(self):
        self.co2_data = 0
        self.sub = rospy.Subscriber('co2_topic', Float32, self.co2_callback)

    def co2_callback(self, msg):
        self.co2_data = msg.data

class MapNode:
    def __init__(self):
        self.map_data = Point()
        self.sub = rospy.Subscriber('map_topic', Point, self.map_callback)

    def map_callback(self, msg):
        self.map_data = msg

def stop():
    global root
    drive_pub.publish(Point(x=0, y=0, z=0)) # publish x,x,x to stop robot movement
    root.destroy()

rospy.init_node('robot_driver', anonymous=True)
sensor_node = CO2SensorNode()
map_node = MapNode()

root = Tk()
label1 = Label(root, text='CO2 Sensor Reading:')
label1.pack()
co2_label = Label(root, text='0')
co2_label.pack()
label2 = Label(root, text='Map Data:')
label2.pack()
map_label = Label(root, text='(0, 0, 0)')
map_label.pack()
button = Button(root, text='Stop', command=stop)
button.pack()

drive_pub = rospy.Publisher('/drive_to_topic', Point, queue_size=10)

points = [(1, 2), (3, 4), (5, 6)]

for point in points:
    drive_to_point = Point()
    drive_to_point.x = point[0]
    drive_to_point.y = point[1]
    drive_pub.publish(drive_to_point)

    while not rospy.is_shutdown():
        co2_label.config(text=str(sensor_node.co2_data))
        map_label.config(text='({:.2f}, {:.2f}, {:.2f})'.format(map_node.map_data.x, map_node.map_data.y, map_node.map_data.z))
        root.update()

        # Check if the robot has reached the point
        # (e.g., using feedback from the robot's sensors)
        # If the robot has reached the point, break out of the loop
        # and proceed to the next point.

        pass

rospy.spin()
