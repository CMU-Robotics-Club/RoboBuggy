#include <gtest/gtest.h>
#include <transistor/gps/GPS_Broadcaster.h>

// Tests a standard message from the NMEA examples
// expects a normal, correct message to be generated
TEST(GPS_Tests, standard_message)
{
    GPS_Broadcaster broadcaster;
    std::string token_string = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

    std::istringstream ss(token_string);
    std::string tokens[10];

    for (int i = 0; i < 10; i++)
    {
        std::getline(ss, tokens[i], ',');
    }

    robobuggy::GPS* msg = broadcaster.parse_tokens(tokens);

    ASSERT_NEAR(msg->Lat_deg, 48.1173, 0.001);
    ASSERT_NEAR(msg->Long_deg, 11.51667, 0.001);
}

// Tests a message where the data is garbage but we don't have a fix
// expects no message to be generated
TEST(GPS_Tests,invalid_data_unfixed)
{
    GPS_Broadcaster broadcaster;
    std::string token_string = "$GPGGA,1,4.0,N,011.1,E,0,08,0.9,545.4,M,46.9,M,,*47";

    std::istringstream ss(token_string);
    std::string tokens[10];

    for (int i = 0; i < 10; i++)
    {
    std::getline(ss, tokens[i], ',');
    }

    robobuggy::GPS* msg = broadcaster.parse_tokens(tokens);
    ASSERT_TRUE(msg==NULL);
}

// Tests a message where the data is valid but we don't have a fix
// Expects no message to be generated
TEST(GPS_Tests,valid_data_unfixed)
{
    GPS_Broadcaster broadcaster;
    std::string token_string = "$GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,*47";

    std::istringstream ss(token_string);
    std::string tokens[10];

    for (int i = 0; i < 10; i++)
    {
    std::getline(ss, tokens[i], ',');
    }

    robobuggy::GPS* msg = broadcaster.parse_tokens(tokens);
    ASSERT_TRUE(msg==NULL);
}

// Tests a message where it isn't a format we're looking for
// Expects no message to be generated
TEST(GPS_Tests,non_GGA_format)
{
    GPS_Broadcaster broadcaster;
    std::string token_string = "$GPVTG,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,*47";

    std::istringstream ss(token_string);
    std::string tokens[10];

    for (int i = 0; i < 10; i++)
    {
    std::getline(ss, tokens[i], ',');
    }

    robobuggy::GPS* msg = broadcaster.parse_tokens(tokens);
    ASSERT_TRUE(msg==NULL);
}

// Tests an empty message where we don't have a fix
// Expects no message to be generated
TEST(GPS_Tests, not_fixed)
{
    GPS_Broadcaster broadcaster;
    std::string token_string = "$GPGGA,,,,,,0,,,,,,,,";

    std::istringstream ss(token_string);
    std::string tokens[10];

    for (int i = 0; i < 10; i++)
    {
        std::getline(ss, tokens[i], ',');
    }

    robobuggy::GPS* msg = broadcaster.parse_tokens(tokens);
    ASSERT_TRUE(msg==NULL);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "IMU_Tester");
    return RUN_ALL_TESTS();
}
