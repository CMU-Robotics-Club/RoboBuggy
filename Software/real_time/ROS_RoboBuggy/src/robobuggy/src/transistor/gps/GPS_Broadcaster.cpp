//include ros library
int main() {
    // initialize ROS Node Handle

    //initialize GPS Publisher on node channel


    //ROS event loop
    //flag for parsing tokens set to true
    //change to while ros ok
    while(true) {

        //get NMEA string from receiver

        //parse token 1 (check if = 'GPGGA'), set flag = false if not

        //parse token 3 --> obtain Latitude value

        //parse token 4 - 'N' label for Latitude (check if = N), set flag = false if not

        //parse token 5 --> obtain Longitude value

        //parse token 6 - 'E' label for Longitude (check if = to E)), set flag = false if not

        //parse token 7 - Fix quality (check if != 0), set flag = false if not

        //check if all tokens still valid, create the message with Latitude, Longitude and publish message

    }

    //ROS shutdown

    return 0;
}