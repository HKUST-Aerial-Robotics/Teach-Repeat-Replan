#include "mpaux.hpp"

#include <cstdio>
#include <gtest/gtest.h>

TEST(tuple, language_test)
{
  mocka::aux::Tuple<double, int, const char*> tss(12.1, 42, "hello world");

  EXPECT_EQ(mocka::aux::__impl::get<0>(&tss), 12.1);
  EXPECT_EQ(mocka::aux::__impl::get<1>(&tss), 42);
  EXPECT_EQ(strcmp("hello world", mocka::aux::__impl::get<2>(&tss)), 0);

  mocka::aux::__impl::get<0>(&tss) = 13;

  tss.get<1>() = 44;

  EXPECT_EQ(tss.get<0>(), 13);
  EXPECT_EQ(tss.get<1>(), 44);
  EXPECT_EQ(strcmp(tss.get<2>(), "hello world"), 0);
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
