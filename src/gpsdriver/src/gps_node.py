#!/usr/bin/env python
import serial
import pynmea2
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geographic_msgs.msg import GeoPoseStamped

def start():
    pub = rospy.Publisher('location', GeoPoseStamped, queue_size=10)
    rospy.init_node('gps_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    locationFound = False

    while not rospy.is_shutdown():
        # Start serial
        port='/dev/ttyACM0'
        ser=serial.Serial(port,baudrate=115200,timeout=0.09)
        dataout =pynmea2.NMEAStreamReader()
        newdata=""

        # Read the entire buffer until timeout
        newdata=str(ser.read(999999999))

        # Decode with utf-8 to remove unreadable binary data from stream
        text_data = bytes(newdata, 'utf-8').decode('utf-8', errors='ignore')
        text_data = str(text_data)

        # There is still a lot of converted binary data so we must find the correct part of the string
        for line in text_data.split('\n'):
            index=line.find('$GNGGA')
            if(index!=-1):
                line = line[int(index): int(index) + 60]
                locationFound = True

                # Parse!
                try:
                    msg=pynmea2.parse(line)
                except:
                    locationFound = False
            else:
                locationFound = False

        if(locationFound):
            # Extract the latitude and longitude from the parsed message
            message = GeoPoseStamped()
            message.pose.position.latitude = msg.latitude
            message.pose.position.longitude = msg.longitude
            message.pose.position.altitude = msg.altitude

            # Publish the latitude and longitude and sleep till next tick
            pub.publish(message)
        else:
            rospy.loginfo("no pos")
            
        rate.sleep()

if __name__ == '__main__':
    try: 
        start()
    except rospy.ROSInterruptException:
        pass