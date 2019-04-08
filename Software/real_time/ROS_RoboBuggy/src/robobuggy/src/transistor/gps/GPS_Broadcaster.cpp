
#include <transistor/gps/GPS_Broadcaster.h>

const std::string GPS_Broadcaster::NODE_NAME = "Transistor_GPS_Broadcaster";
GPS_Broadcaster::GPS_Broadcaster()
{
    gps_pub = nh.advertise<robobuggy::GPS>("GPS", 1000);

    //Initialize serial port baud
    if (!nh.getParam(NODE_NAME + "/serial_port", serial_port))
    {
        ROS_INFO_STREAM("Serial Port Parameter not found, using default");
        serial_port = "/dev/buggygps";
    }
    if (!nh.getParam(NODE_NAME + "/serial_baud", serial_baud))
    {
        ROS_INFO_STREAM("Serial Baud parameter not found, using default");
        serial_baud = 9600;
    }



}

int GPS_Broadcaster::initialize_hardware()
{
    try
    {
        gps_serial.setPort(serial_port);
        gps_serial.setBaudrate(serial_baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        gps_serial.setTimeout(to);
        gps_serial.open();
        return 0;
    }
    catch (serial::IOException e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        ROS_ERROR_STREAM(serial_port);
        return -1;
    }
}

//parse ONE NMEA string
void GPS_Broadcaster::read_gps_message()
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

            robobuggy::GPS* gps_message = parse_tokens(tokens);


            if (gps_message != NULL)
            {
                gps_pub.publish(*gps_message);
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

robobuggy::GPS* GPS_Broadcaster::parse_tokens(std::string tokens[])
{

    //parse token 0 (check if = 'GPGGA')
    if (tokens[0] != "$GPGGA")
    {
        ROS_ERROR_STREAM("read a GPGGA when it wasn't!");
        return NULL;
    }

    //we do not parse token 1 in the GPGGA string, since it represents time


    //parse token 2 --> obtain Latitude value
    double latitude_deg = convert_to_latitude(tokens[2]);

    //parse token 3 - 'N' label for Latitude (check if = N)
    if (tokens[3] != "N")
    {
        latitude_deg *= -1;
    }

    //parse token 4 --> obtain Longitude value
    double longitude_deg = convert_to_longitude(tokens[4]);

    //parse token 5 - 'E' label for Longitude (check if = to E))
    if (tokens[5] != "E")
    {
        longitude_deg *= -1;
    }

    //parse token 6 - Fix quality (check if != 0)
    if (atoi(tokens[6].c_str()) == 0)
    {
        ROS_ERROR_STREAM("Not locked yet!");
        return NULL;
    }

    // create the message with Latitude, Longitude and publish message

    geographic_msgs::GeoPoint gps_point;
    gps_point.latitude = latitude_deg;
    gps_point.longitude = longitude_deg;

    geodesy::UTMPoint utm_point(gps_point);

    gps_message.Lat_deg = static_cast<float>(latitude_deg);
    gps_message.Long_deg = static_cast<float>(longitude_deg);
    gps_message.northing = static_cast<float>(utm_point.northing);
    gps_message.easting = static_cast<float>(utm_point.easting);

    return &gps_message;
}

double GPS_Broadcaster::convert_to_latitude(std::string str)
{
    //special case if gps not locked, string is length 0
    if (str.size() == 0)
    {
        return 0; //special case will get addressed later on
    }
    double degrees = atof(str.substr(0, 2).c_str());
    double minutes = atof(str.substr(2).c_str());

    return degrees + minutes /  60.0;
}

double GPS_Broadcaster::convert_to_longitude(std::string str)
{
    //special case if gps not locked, string is length 0
    if (str.size() == 0)
    {
        return 0; //special case will get addressed later on
    }

    double degrees = atof(str.substr(0, 3).c_str());
    double minutes = atof(str.substr(3).c_str());

    return degrees + minutes / 60.0;
}
