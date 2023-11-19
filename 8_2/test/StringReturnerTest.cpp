#include <StringReturner.hpp>
#include <gtest/gtest.h>

TEST(StringReturner, HelloWorld)
{
    auto strReturner = StringReturner();

    const std::string hello = "hello";
    const std::string world = "world";

    EXPECT_EQ(hello, strReturner.getHello());
    EXPECT_EQ(world, strReturner.getWorld());
}