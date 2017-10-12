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

TEST(GPS_Test_Suite, gps_test_gpgga_valid)
{
    Transistor_GPS_Broadcaster broadcaster;
    std::string valid[] = {"$GPGGA", "i am not an int but function should not care"};
    EXPECT_TRUE(broadcaster.gpgga_valid(valid));

    std::string invalid1[] = {"$GPGGN"};
    EXPECT_FALSE(broadcaster.gpgga_valid(invalid1));
    std::string invalid2[] = {"ABCD"};
    EXPECT_FALSE(broadcaster.gpgga_valid(invalid2));
}

TEST(GPS_Test_Suite, gps_test_lock)
{
    Transistor_GPS_Broadcaster broadcaster;
    std::string locked[] = {"", "", "", "", "", "", "1"};
    EXPECT_TRUE(broadcaster.gps_locked(locked));

    std::string not_locked[] = {"", "", "", "", "", "", "0"};
    EXPECT_FALSE(broadcaster.gps_locked(not_locked));
}

TEST(GPS_Test_Suite, gps_test_message)
{
    Transistor_GPS_Broadcaster broadcaster;
    geodesy::UTMPoint utm_point = broadcaster.gps_get_location(41.414938, 81.86139667);
    robobuggy::GPS gps_message = broadcaster.gps_gen_message(41.414938, 81.86139667, utm_point);

    EXPECT_NEAR(gps_message.Lat_deg, 41.414938, 0.000002);
    EXPECT_NEAR(gps_message.Long_deg, 81.86139667, 0.0000002);
    EXPECT_NEAR(gps_message.Lat_m, utm_point.northing, 1);
    EXPECT_NEAR(gps_message.Long_m, utm_point.easting, 1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "GPS_Latitude_Tester");
    return RUN_ALL_TESTS();
}
