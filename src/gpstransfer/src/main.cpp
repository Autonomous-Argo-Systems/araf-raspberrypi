#include <ros/ros.h>
#include <nmeaparse/nmea.h>
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <mavros_msgs/HilGPS.h>
#include <thread>
#include <string.h>
#include <chrono>
#include <fstream>
#include <iomanip>

using namespace LibSerial;

constexpr const char *const SERIAL_PORT_UBLOX = "/dev/ttyACM0";
SerialPort serialPort;

nmea::NMEAParser parser;
nmea::GPSService gps(parser);

mavros_msgs::HilGPS gpsMessage;

void readUBLOX()
{

    while (ros::ok())
    {
        if (serialPort.GetNumberOfBytesAvailable() == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Reading sentence
        std::string nmeaSentence;
        serialPort.ReadLine(nmeaSentence, '\n', 10);

        // Parsing sentence
        try
        {
            if (
                nmeaSentence.find("GGA") != std::string::npos ||
                nmeaSentence.find("GSA") != std::string::npos ||
                nmeaSentence.find("GSV") != std::string::npos ||
                nmeaSentence.find("RMC") != std::string::npos ||
                nmeaSentence.find("VTG") != std::string::npos)
            {
                parser.readLine(nmeaSentence);
            }
        }
        catch (nmea::NMEAParseError &e)
        {
            ROS_INFO("Error in parsing sentence %s\n%s", e.message.c_str(), e.nmea.text.c_str());
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpstransfer");
    ros::NodeHandle nh;

    // Opening serial ports
    try
    {
        serialPort.Open(SERIAL_PORT_UBLOX);
    }
    catch (const OpenFailed &)
    {
        ROS_ERROR("The serial ports did not open correctly.");
        return EXIT_FAILURE;
    }

    // Setting correct settings
    serialPort.SetBaudRate(BaudRate::BAUD_115200);
    serialPort.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serialPort.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serialPort.SetParity(Parity::PARITY_NONE);
    serialPort.SetStopBits(StopBits::STOP_BITS_1);

    // Starting reading ublox
    std::thread ubloxThread(readUBLOX);

    // Starting publiser
    ros::Publisher gpsPublisher = nh.advertise<mavros_msgs::HilGPS>("/mavros/hil/gps", 1000);

    // Publishing to px4
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // Creating message
        gpsMessage.geo.latitude = gps.fix.latitude;
        gpsMessage.geo.longitude = gps.fix.longitude;
        gpsMessage.geo.altitude = gps.fix.altitude;
        gpsMessage.eph = gps.fix.horizontalDilution * 100;
        gpsMessage.epv = gps.fix.verticalDilution * 100;
        gpsMessage.cog = gps.fix.travelAngle;
        gpsMessage.fix_type = gps.fix.type;
        gpsMessage.vel = (gps.fix.speed / 3.6f) * 100;
        gpsMessage.satellites_visible = gps.fix.visibleSatellites;

        // Printing debug
        ROS_INFO("%s", gps.fix.toString().c_str());

        // Sending message
        gpsPublisher.publish(gpsMessage);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Closing ports
    serialPort.Close();

    return 0;
}