#include "gtest/gtest.h"

// Tests that the Foo::Bar() method does Abc.
TEST(MotorControllerNode, SplitTest)
{
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
