#include <iostream>

int main() {
    std::cout << "init !" << std::endl;
    // initialize ROS Node

    //initialize GPS Publisher

    //ROS event loop

    //flag set to true
    while(true) {

        //get NMEA string from receiver

        //parse token 1 (check if = 'GPGGA'), set flag = false if not

        //parse token 2

        //parse token 3 --> obtain Latitude

        //parse token 4 (check if = N), set flag = false if not

        //parse token 5 --> obtain Longitude

        //parse token 6 (check if = to E)), set flag = false if not

        //parse token 7 (check if != 0), set flag = false if not

        //check if flag still true, and if so create the message with Latitude, Longitude and publish message

        //reset flag to true

    }

    //ROS shutdown

    return 0;
}