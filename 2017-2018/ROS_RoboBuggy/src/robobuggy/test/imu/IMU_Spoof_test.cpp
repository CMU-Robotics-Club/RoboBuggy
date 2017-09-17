#include <gtest/gtest.h>
#include "imu/IMU_Broadcaster.h"

// Declare a test
TEST(TestSuite, testCase1)
{
    IMU_Broadcaster broadcaster;

    ASSERT_EQ(broadcaster.get_spoofed_x_accel(), 1.5);
    ASSERT_EQ(broadcaster.get_spoofed_y_accel(), 2.5);
    ASSERT_EQ(broadcaster.get_spoofed_z_accel(), 1.5);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "IMU_Tester");
    return RUN_ALL_TESTS();
}