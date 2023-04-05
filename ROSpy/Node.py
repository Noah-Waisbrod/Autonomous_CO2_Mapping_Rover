#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Float32

SENSOR_PIN = 5  # GPIO pin connected to sensor

def read_sensor():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SENSOR_PIN, GPIO.IN)
    sensor_reading = GPIO.input(SENSOR_PIN)
    GPIO.cleanup()
    return sensor_reading

def main():
    rospy.init_node('sensor_publisher', anonymous=True)
    pub = rospy.Publisher('sensor_reading', Float32, queue_size=10)
    rate = rospy.Rate(10)  # publish at 10Hz

    while not rospy.is_shutdown():
        sensor_reading = read_sensor()
        rospy.loginfo('Sensor reading: %s', sensor_reading)
        pub.publish(Float32(sensor_reading))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
