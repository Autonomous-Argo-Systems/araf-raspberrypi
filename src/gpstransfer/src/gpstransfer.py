import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import HilGPS
from geographic_msgs.msg import GeoPoint
from pynmea2 import parse
import serial
import _thread

gpsBuffer = HilGPS() 
nmeaBuffer = NavSatFix()

def read_nmea():
    # Starting serial
    port='/dev/ttyACM1'
    ser= serial.Serial(port, baudrate=115200, stopbits=1)

    while not rospy.is_shutdown():
        # Parsing from serial bus.
        line = ser.readline().decode('utf-8')
        rospy.loginfo(line)
        if line.startswith('$'):
            try:
                data = parse(line)
            except Exception as e:
                 rospy.loginfo(f"Error parsing NMEA sentence: {e}")

        # Putting parsed data in buffer
        if hasattr(data, 'longitude'):
            if data.longitude is not None:
                gpsBuffer.geo.longitude = float(data.longitude)
                nmeaBuffer.longitude = gpsBuffer.geo.longitude
        if hasattr(data, 'latitude'):
            if data.latitude is not None:
                gpsBuffer.geo.latitude = float(data.latitude)
                nmeaBuffer.latitude = gpsBuffer.geo.latitude
        if hasattr(data, 'altitude') :
            if data.altitude is not None:
                gpsBuffer.geo.altitude = float(data.altitude) 
                nmeaBuffer.altitude =gpsBuffer.geo.altitude 
        if hasattr(data, 'spd_over_grnd'):
            if data.spd_over_grnd is not None:
                gpsBuffer.vel = int(data.spd_over_grnd)
        if hasattr(data, 'true_course'):
            if data.true_course is not None:
                gpsBuffer.cog = int(data.true_course)
        if hasattr(data, 'num_sats'):
            if data.num_sats is not None:
                gpsBuffer.satellites_visible = int(data.num_sats)
        if hasattr(data, 'fix_type'):
            if data.fix_type is not None:
                gpsBuffer.fix_type = int(data.fix_type)
        if hasattr(data, 'hdop'):
            if data.hdop is not None:
                rospy.loginfo(data.hdop)
                gpsBuffer.eph = int(float(data.hdop)*100.0)
        if hasattr(data, 'vdop'):
            if data.vdop is not None:

                gpsBuffer.epv = int(float(data.vdop)*100.0)



if __name__ == '__main__':
    rospy.init_node('gpstransfer')

    # Starting publishers
    hil_gps_pub = rospy.Publisher('/mavros/hil/gps', HilGPS, queue_size=10)
    nmea_pub = rospy.Publisher('/mavros/global_position/raw/fix', NavSatFix, queue_size=10)

    # Start reading serial
    # _thread.start_new_thread(read_nmea, ())

    # Publising every 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        gpsBuffer.header.stamp = rospy.Time.now()
        gpsBuffer.fix_type = 3  # GPS fix type: 3 for simulated data
        gpsBuffer.eph = 1  # GPS HDOP
        gpsBuffer.epv = 1  # GPS VDOP
        gpsBuffer.vel = 0  # GPS ground speed in m/s
        gpsBuffer.vn = 0  # GPS velocity north in m/s
        gpsBuffer.ve = 0  # GPS velocity east in m/s
        gpsBuffer.vd = 0  # GPS velocity down in m/s
        gpsBuffer.cog = 0  # Course over ground in degrees
        gpsBuffer.satellites_visible = 10  # Number of visible satellites
        gpsBuffer.geo = GeoPoint()
        gpsBuffer.geo.latitude = 51.7749
        gpsBuffer.geo.longitude = 4.77749
        gpsBuffer.geo.altitude = 0.0

        # Create the NavSatFix message for NMEA stream
        nmeaBuffer.latitude = gpsBuffer.geo.latitude
        nmeaBuffer.longitude = gpsBuffer.geo.longitude
        nmeaBuffer.altitude = gpsBuffer.geo.altitude

        # rospy.loginfo("%s", gpsBuffer)
        hil_gps_pub.publish(gpsBuffer)
        nmea_pub.publish(nmeaBuffer)
        rate.sleep()
