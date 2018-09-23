#include <gtest/gtest.h>
#include <transistor/low_level/LowLevel_Broadcaster.h>


class LLTestFixture: public ::testing::test {
public:
    vector<robobuggy::Feedback> feedback_msg_list;
    vector<robobuggy::Diagnostics> diagnostic_msg_list;
    vector<robobuggy::Encoder> enc_msg_list;
    ros::Subscriber feedback_sub;
    ros::Subscriber diag_sub;
    ros::Subscriber enc_sub;

    LLTestFixture() {
        // beginning of test suite
        feedback_sub = ros::Subscriber("Feedback", 1000, [&feedback_msg_list](const robobuggy::Feedback::ConstPtr& msg) {
            feedback_msg_list.push_back(msg);
        });
        diag_sub = ros::Subscriber("Diagnostics", 1000, [&diagnostic_msg_list](const robobuggy::Diagnostics::ConstPtr& msg) {
            diagnostic_msg_list.push_back(msg);
        });
        enc_sub = ros::Subscriber("Encoder", 1000, [&enc_msg_list](const robobuggy::Encoder::ConstPtr& msg) {
            enc_msg_list.push_back(msg);
        });
    }
    void SetUp() {
        // beginning of each test
        feedback_msg_list.clear();
        diagnostic_msg_list.clear();
        enc_msg_list.clear();
    }
    void TearDown() {
        // end of each test
    }
    ~LLTestFixture() {
        // after test suite
        feedback_sub.shutdown();
        diag_sub.shutdown();
        enc_sub.shutdown();
    }
}

// Tests a single, correct low-level message received
// Expects a single, correct ROS message
TEST(LL_Tests, single_good_rcv_msg) {
    ASSERT_TRUE(false);

    char serial_data[] = { 20, 0, 0, 1, 0xAB, 0xA, 0 };
    std::string buffer(serial_data);

    // 1s timeout
    ros::Duration(1.0).sleep();

    ASSERT_EQ(feedback_msg_list.size(), 1);
    ASSERT_EQ(diagnostic_msg_list.size(), 1);
    ASSERT_EQ(enc_sub.size(), 1);
}

// Tests multiple, correct low-level messages received
// Expects multiple, correct ROS messages
TEST(LL_Tests, multiple_good_rcv_msgs) {
    ASSERT_TRUE(false);
}

// Tests single, incorrect message (wrong footer)
// Expects no message to be generated
TEST(LL_Tests, single_bad_rcv_msg) {
    ASSERT_TRUE(false);
}

// Tests multiple, incorrect messages (wrong footers)
// Expects no messages to be generated
TEST(LL_Tests, multiple_bad_rcv_msgs) {
    ASSERT_TRUE(false);
}

// Tests single, correct Command message
// Expects single, correct Arduino message
TEST(LL_Tests, single_good_com_msg) {
    ASSERT_TRUE(false);
}

// Tests multiple correct Command messages
// Expects multiple correct Arduino mesasges
TEST(LL_Tests, multi_good_comm_msgs) {
    ASSERT_TRUE(false);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "Low_Level_Tester");
    return RUN_ALL_TESTS();
}
