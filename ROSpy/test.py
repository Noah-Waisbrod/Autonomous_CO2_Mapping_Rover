import rospy
from geometry_msgs.msg import Point,Occuapncy_grid,MapMetaData
from std_msgs.msg import Float32, Int32
from tkinter import *
import matplotlib as plt
import mapping

class CO2SensorNode:
    def __init__(self):
        self.co2_data = 0
        self.sub = rospy.Subscriber('co2_topic', Float32, self.co2_callback)

    def co2_callback(self, msg):
        self.co2_data = msg

class MapNode:
    def __init__(self):
        self.map_data = Point()
        self.map_metadata = MapMetaData()
        self.width =0
        self.height = 0
        self.sub = rospy.Subscriber('map_topic', Occuapncy_grid, self.map_callback)

    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_metadata= msg.MapMetaData
        self.width = map_metadata.width
        self.height = map_metadata.height
        self.two_d_array = [[0 for x in range(width)] for y in range(height)]
        index =0
        for y in range(height):
            for x in range(width):
                two_d_array[y][x] = map_data[index]
                index += 1
        plt.plot(two_d_array)
class Current_pos:
    def __init__(self):
        self.cp =Point()
        self.POSE =POSE() #todo fix this shit
        self.orientation = Orientation()
        self.cp_x =0
        self.cp_y =0
        self.sub = rospy.Subscriber('map_pose', POSE, self.currentPos_callback)

    def currentPos_callback(self,msg):
        self.pose = msg
        self.cp = self.POSE.point
        self.cp_x=self.cp.y
        self.cp_y =self.cp.y




def stop():
    global root
    kill_pub.publish(1)
    root.destroy()
    while True:
        rospy.logwarn("Stop pushed, Function frozen")

def update_heatmap(data,map):
    global canvas
    co2Min = 100000
    co2Max =0

    for i in data[3]:
        if c02Min>i:
            co2Min =i
        if c02Max<i:
            co2Max =i
    co2_norm = (co2 - co2Min) / (co2Max - co2Min)
    heatmap = np.zeros((map.width, map.height, 3))
    heatmap[:, :, 0] = co2_norm  # set red channel to co2 data
    heatmap[:, :, 2] = map_data / 100.0  # set blue channel to map data (normalized)
    img = PhotoImage(data=np.uint8(heatmap * 255))
    return(img)




    

visited_point =[[],[],[]]
drive_pub = rospy.Publisher('/drive_to_topic', Point, queue_size=10)
kill_pub = rospy.Publisher('/Kill', Int32, queue_size=10)

rospy.init_node('robot_driver', anonymous=True)

sensor_node = CO2SensorNode()
map_node = MapNode()
current_point = Current_pos()
visited_point[0].append(current_point.cp_x)
visited_point[1].append(current_point.cp_x)
visited_point[2].append(sensor_node.co2_data)

root = Tk()
canvas = Canvas(root, width=map_node.map_data.info.width, height=map_node.map_data.info.height)
canvas.pack()
label1 = Label(root, text='CO2 Sensor Reading:')
label1.pack()
co2_label = Label(root, text='0')
co2_label.text = sensor_node.co2_data
co2_label.pack()
label2 = Label(root, text='Map Data:')
label2.pack()
map_label = Label(root, text='(0, 0, 0)')
map_label.pack()
button = Button(root, text='Stop', command=stop)
button.pack()


goal = mapping.close_point(map_node.two_d_array,visited_point)
points = mapping.astar(current_point.cp,goal,map_node.two_d_array)

for point in points:
    drive_to_point = Point()
    drive_to_point.x = point[0]
    drive_to_point.y = point[1]
    drive_pub.publish(drive_to_point)

    while not rospy.is_shutdown():
        co2_label.config(text=str(sensor_node.co2_data))
        map_label.config(text='({:.2f}, {:.2f}, {:.2f})'.format(map_node.map_data.x, map_node.map_data.y, map_node.map_data.z))
        img = update_heatmap(visited_point,map_node)
        canvas.create_image(0, 0, anchor=NW, image=img)
        canvas.img = img
        root.update()

        # Check if the robot has reached the point
        # (e.g., using feedback from the robot's sensors)
        # If the robot has reached the point, break out of the loop
        # and proceed to the next point.

        pass


rospy.spin()

