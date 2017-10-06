#include <gtest/gtest.h>
#include "gps/Transistor_GPS_Broadcaster.h"


TEST(GPS_Test_Suite, gps_test_lat_normal)
{
}

TEST(GPS_Test_Suite, gps_test_lat_null)
{
    Transistor_GPS_Broadcaster broadcaster;
    broadcaster.convert_to_latitude(NULL);
}

TEST(GPS_Test_Suite, gps_test_lat_emptystr)
{
    Transistor_GPS_Broadcaster broadcaster;
    broadcaster.convert_to_latitude("");
}

TEST(GPS_Test_Suite, gps_test_lat_outofworld)
{

}

TEST(GPS_Test_Suite, gps_test_lon_normal)
{

}

TEST(GPS_Test_Suite, gps_test_lon_null)
{
    Transistor_GPS_Broadcaster broadcaster;
    broadcaster.convert_to_longitude(NULL);
}

TEST(GPS_Test_Suite, gps_test_lon_emptystr)
{
    Transistor_GPS_Broadcaster broadcaster;
    broadcaster.convert_to_longitude("");
}

TEST(GPS_Test_Suite, gps_test_lon_outofworld)
{

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "GPS_Tester");
    return RUN_ALL_TESTS();
}