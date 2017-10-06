#include <gtest/gtest.h>
#include "gps/Transistor_GPS_Broadcaster.h"


TEST(GPS_Test_Suite, gps_test_lat_normal)
{
    Transistor_GPS_Broadcaster broadcaster;
    ASSERT_NEAR(broadcaster.convert_to_latitude("4124.8963"), 41.414938, 0.000001);
}

TEST(GPS_Test_Suite, gps_test_lat_emptystr)
{
    Transistor_GPS_Broadcaster broadcaster;
    ASSERT_EQ(broadcaster.convert_to_latitude(""), 0);
}

TEST(GPS_Test_Suite, gps_test_lon_normal)
{
    Transistor_GPS_Broadcaster broadcaster;
    ASSERT_NEAR(broadcaster.convert_to_longitude("08151.6838"), 81.86139667, 0.00000001);
}

TEST(GPS_Test_Suite, gps_test_lon_emptystr)
{
    Transistor_GPS_Broadcaster broadcaster;
    broadcaster.convert_to_longitude("");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "GPS_Latitude_Tester");
    return RUN_ALL_TESTS();
}