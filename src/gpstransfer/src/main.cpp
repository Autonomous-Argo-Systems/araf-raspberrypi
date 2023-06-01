#include <ros/ros.h>
#include <nmea/nmea.h>
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <mavros_msgs/HilGPS.h>
#include <thread>
#include <string.h>
#include <chrono>

using namespace LibSerial ;


constexpr const char* const SERIAL_PORT_UBLOX = "/dev/ttyUSB0";    
SerialPort serialPort;

nmeaINFO gpsInfo;
mavros_msgs::HilGPS gpsMessage;

double ndegToDecimalUBLOX(double ndegValue)
{
  double degrees = std::floor(ndegValue / 100);     
  double minutes = std::fmod(ndegValue, 100);          
  double decimalDegrees = degrees + (minutes / 60);     
  return decimalDegrees;
}

void readUBLOX(){
    // Starting parser
    nmeaPARSER parser;
    nmea_zero_INFO(&gpsInfo);
    nmea_parser_init(&parser);
   
    while (ros::ok()){
        if (serialPort.GetNumberOfBytesAvailable() == 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Reading sentence
        std::string nmeaSentence;
        serialPort.ReadLine(nmeaSentence, '\n', 10);

        // Parsing sentence
        if (nmeaSentence[0] == '$'){
            nmea_parse(&parser, nmeaSentence.c_str(), nmeaSentence.length(), &gpsInfo);
        }
    }

    // Destroying parser if program quits
    nmea_parser_destroy(&parser);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gpstransfer");
    ros::NodeHandle nh;
    
    // Opening serial ports
    try
    {
        serialPort.Open(SERIAL_PORT_UBLOX);
    }
    catch (const OpenFailed&)
    {
        ROS_ERROR("The serial ports did not open correctly.");
        return EXIT_FAILURE ;
    }

    // Setting correct settings
    serialPort.SetBaudRate(BaudRate::BAUD_115200);
    serialPort.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serialPort.SetFlowControl(FlowControl::FLOW_CONTROL_NONE); 
    serialPort.SetParity(Parity::PARITY_NONE);
    serialPort.SetStopBits(StopBits::STOP_BITS_1);

    // Starting reading ublox
    std::thread ubloxThread (readUBLOX);

    // Starting publiser
    ros::Publisher gpsPublisher = nh.advertise<mavros_msgs::HilGPS>("/mavros/hil/gps", 1000);

    // Publishing to px4
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // Creating message
        gpsMessage.geo.latitude = ndegToDecimalUBLOX(gpsInfo.lat);
        gpsMessage.geo.longitude = ndegToDecimalUBLOX(gpsInfo.lon);
        gpsMessage.geo.altitude = gpsInfo.elv;
        gpsMessage.eph = gpsInfo.HDOP / 100.0f;
        gpsMessage.epv = gpsInfo.VDOP / 100.0f;
        gpsMessage.cog = gpsInfo.direction;
        gpsMessage.fix_type = gpsInfo.fix;
        gpsMessage.vel = gpsInfo.speed / 3.6f;
        gpsMessage.satellites_visible = gpsInfo.satinfo.inview;

        // Printing debug
        ROS_INFO("Latitude: %8f, Longtitude: %8f, Fix: %d, Visible: %d", 
        gpsMessage.geo.latitude, gpsMessage.geo.longitude,
         gpsMessage.fix_type, gpsMessage.satellites_visible);

        // Sending message
        gpsPublisher.publish(gpsMessage);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Closing ports
    serialPort.Close();

    return 0;
}