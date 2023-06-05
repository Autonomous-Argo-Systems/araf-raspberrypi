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
constexpr const char *const SERIA_PORT_ZIGBEE = "/dev/ttyACM1";

SerialPort serialPortUblox;
SerialPort serialPortZigee;

nmea::NMEAParser parser;
nmea::GPSService gps(parser);

mavros_msgs::HilGPS gpsMessage;

void readZigbee()
{
    while (!serialPortZigee.IsOpen())
    {
        try
        {
            serialPortZigee.Open(SERIA_PORT_ZIGBEE);
        }
        catch (const OpenFailed &)
        {
            ROS_WARN("Zigbee serial port not opend correctly");
            sleep(5);
        }
    }

    while (ros::ok())
    {
        if (serialPortUblox.GetNumberOfBytesAvailable() == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Reading sentence
        //serialPort.ReadLine(nmeaSentence, '\n');

        
        
    }
}

void readUblox()
{
    // Opening serial port
    while (!serialPortUblox.IsOpen())
    {
        try
        {
            serialPortUblox.Open(SERIAL_PORT_UBLOX);
        }
        catch (const OpenFailed &)
        {
            ROS_ERROR("The serial ports did not open correctly.");
            sleep(1);
        }
    }

     // Setting correct settings
    serialPortUblox.SetBaudRate(BaudRate::BAUD_115200);
    serialPortUblox.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serialPortUblox.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serialPortUblox.SetParity(Parity::PARITY_NONE);
    serialPortUblox.SetStopBits(StopBits::STOP_BITS_1);

    // Start reading
    while (ros::ok())
    {
        if (serialPortUblox.GetNumberOfBytesAvailable() == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Reading sentence
        std::string nmeaSentence;
        serialPortUblox.ReadLine(nmeaSentence, '\n');

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

    // Starting reading ublox
    std::thread ubloxThread(readUblox);

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
        gpsMessage.eph = 0;
        gpsMessage.epv = 0;
        gpsMessage.cog = gps.fix.travelAngle;
        gpsMessage.fix_type = gps.fix.type;
        gpsMessage.vel = (gps.fix.speed / 3.6f);
        gpsMessage.satellites_visible = gps.fix.trackingSatellites;

        // Printing debug
        ROS_INFO("%s", gps.fix.toString().c_str());

        // Sending message
        gpsPublisher.publish(gpsMessage);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Closing ports
    serialPortUblox.Close();

    return 0;
}