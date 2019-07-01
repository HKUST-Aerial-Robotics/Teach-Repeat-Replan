#include <gtest/gtest.h>

#include "eventspinner.hpp"

TEST(event_spin, language_test)
{
  mocka::Event<double, int> e(
    [](mocka::Event<double, int>* t) -> int {
      return (t->get<0>() + t->get<1>()) == 10 ? 1 : 0;
    },
    1.0, 1);
  EXPECT_EQ(e.spin(), 0);
  e.get<0>() = 9;
  EXPECT_EQ(e.spin(), 1);
}

TEST(spinner, feature_test)
{
  //! @note cannot allocate the eventspinner item 'class Event' from heap,
  //! since we used shared_ptr
  auto e = new mocka::Event<double, int>(
    [](mocka::Event<double, int>* t) -> int {
      return (t->get<0>() + t->get<1>()) == 10 ? 1 : 0;
    },
    1.0, 1);

  auto e2 = new mocka::Event<double, int>(
    [](mocka::Event<double, int>* t) -> int {
      return (t->get<0>() + t->get<1>()) == 10 ? 2 : 0;
    },
    1.0, 2);
  auto e3 = new mocka::Event<double, int>(
    [](mocka::Event<double, int>* t) -> int {
      return (t->get<0>() + t->get<1>()) == 10 ? 3 : 0;
    },
    1.0, 3);

  mocka::EventSpinner sp;
  sp.push(e);
  sp.push(e2);
  sp.push(e3);
  EXPECT_EQ(sp.getEvent()->length(), 0);
  sp.spin();
  EXPECT_EQ(sp.getEvent()->length(), 1);
  EXPECT_EQ(sp.getEvent()->pop(), 0);
  EXPECT_EQ(sp.getEvent()->length(), 0);

  e->get<0>() = 9.0;
  sp.spin();
  EXPECT_EQ(sp.getEvent()->length(), 1);
  EXPECT_EQ(sp.getEvent()->pop(), 1);
  EXPECT_EQ(sp.getEvent()->length(), 0);

  e2->get<0>() = 8.0;
  sp.spin();
  EXPECT_EQ(sp.getEvent()->length(), 2);
  EXPECT_EQ(sp.getEvent()->pop(), 1);
  EXPECT_EQ(sp.getEvent()->pop(), 2);
  EXPECT_EQ(sp.getEvent()->length(), 0);

  e3->get<0>() = 7.0;
  sp.spin();
  EXPECT_EQ(sp.getEvent()->length(), 3);
  EXPECT_EQ(sp.getEvent()->pop(), 1);
  EXPECT_EQ(sp.getEvent()->pop(), 2);
  EXPECT_EQ(sp.getEvent()->pop(), 3);
  EXPECT_EQ(sp.getEvent()->length(), 0);
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
