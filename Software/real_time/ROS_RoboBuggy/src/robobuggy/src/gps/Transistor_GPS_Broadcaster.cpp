//
// Created by bhai on 9/29/17.
//

#include <gps/Transistor_GPS_Broadcaster.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

const std::string Transistor_GPS_Broadcaster::NODE_NAME = "Transistor_GPS_Broadcaster";
Transistor_GPS_Broadcaster::Transistor_GPS_Broadcaster()
{
    gps_pub = nh.advertise<robobuggy::GPS>("GPS", 1000);

    if (!nh.getParam(NODE_NAME + "/serial_port", serial_port))
    {
        ROS_INFO_STREAM("Serial Port Parameter not found, using default");
        serial_port = "/dev/ttyACM0";
    }
    if (!nh.getParam(NODE_NAME + "/serial_baud", serial_baud))
    {
        ROS_INFO_STREAM("Serial Baud parameter not found, using default");
        serial_baud = 9600;
    }
}

int Transistor_GPS_Broadcaster::handle_serial_messages()
{
    serial::Serial gps_serial;
    try
    {
        gps_serial.setPort(serial_port);
        gps_serial.setBaudrate(serial_baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        gps_serial.setTimeout(to);
        gps_serial.open();
    }
    catch (serial::IOException e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        ROS_ERROR_STREAM(serial_port);
        return -1;
    }

    while (ros::ok())
    {
        if (gps_serial.available())
        {
            gps_serial_buffer += gps_serial.read(gps_serial.available());

            size_t gpgga_pos = 0;
            if ((gpgga_pos = gps_serial_buffer.find("$GPGGA")) != std::string::npos)
            {
                gps_serial_buffer = gps_serial_buffer.substr(gpgga_pos);

                std::istringstream ss(gps_serial_buffer);
                std::string tokens[10];

                for (int i = 0; i < 10; i++)
                {
                    std::getline(ss, tokens[i], ',');
                }

                if (tokens[0] != "$GPGGA")
                {
                    ROS_ERROR_STREAM("read a GPGGA when it wasn't!");
                    return -1;
                }
                else if (atoi(tokens[6].c_str()) == 0)
                {
                    ROS_ERROR_STREAM("Not locked yet!");
                }
                else
                {
                    double latitude_deg = convert_to_latitude(tokens[2]);
                    double longitude_deg = convert_to_longitude(tokens[4]);

                    if (tokens[5] == "W")
                    {
                        longitude_deg = -1*longitude_deg;
                    }

                    geographic_msgs::GeoPoint gps_point;
                    gps_point.latitude = latitude_deg;
                    gps_point.longitude = longitude_deg;

                    geodesy::UTMPoint utm_point(gps_point);

                    robobuggy::GPS gps_message;
                    gps_message.Lat_deg = static_cast<float>(latitude_deg);
                    gps_message.Long_deg = static_cast<float>(longitude_deg);
                    gps_message.Lat_m = static_cast<float>(utm_point.northing);
                    gps_message.Long_m = static_cast<float>(utm_point.easting);
                    gps_pub.publish(gps_message);
                }

                size_t total_bytes_read = tokens->length();
                if (gps_serial_buffer.length() > total_bytes_read)
                {
                    gps_serial_buffer = gps_serial_buffer.substr(total_bytes_read);
                }
                else
                {
                    gps_serial_buffer = "";
                }

            }
        }
    }

    return 0;
}

double Transistor_GPS_Broadcaster::convert_to_latitude(std::string str)
{
    double degrees = atof(str.substr(0, 2));
    double minutes = atof(str.substr(2));

    return degrees + minutes /  60.0;
}

double Transistor_GPS_Broadcaster::convert_to_longitude(std::string str)
{
    double degrees = atof(str.substr(0, 3));
    double minutes = atof(str.substr(3));

    return degrees + minutes / 60.0;
}
