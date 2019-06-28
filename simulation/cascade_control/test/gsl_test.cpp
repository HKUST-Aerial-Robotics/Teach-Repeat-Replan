#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <ctime>
#include <math.h>

#include "frequencyalign.hpp"

TEST(functional_test, fft_r2)
{
  mocka::Signal s;
  s.setSize(256);
  for (int i = 1; i <= 256; ++i)
  {
    s.push(std::cos(i * M_PIl / 32.0));
  }
  s.fft();
  for (int i = 0; i < 256; ++i)
  {
    EXPECT_NEAR((i == 4 || i == 252) ? 1 : 0, s.abs(i), 1e-10);
    EXPECT_NEAR(0, s.im(i), 1e-10);
    EXPECT_NEAR((i == 4 || i == 252) ? 1 : 0, s.re(i), 1e-10);
  }
  EXPECT_NEAR(0, s.arg(4), 1e-10);
  EXPECT_NEAR(0, s.arg(252), 1e-10);
}

TEST(functional_test, fft_not_r2)
{
  mocka::Signal s;
  s.setSize(400);
  for (int i = 1; i <= 400; ++i)
  {
    s.push(std::cos(i * M_PIl / 40));
  }
  s.fft();
  for (int i = 0; i < 400; ++i)
  {
    EXPECT_NEAR((i == 5 || i == 395) ? 1 : 0, s.abs(i), 1e-6);
    EXPECT_NEAR(0, s.im(i), 1e-10);
    EXPECT_NEAR((i == 5 || i == 395) ? 1 : 0, s.re(i), 1e-10);
  }
  EXPECT_NEAR(0, s.arg(5), 1e-10);
  EXPECT_NEAR(0, s.arg(395), 1e-10);
}

TEST(functioinal_test, ifft_r2)
{
  mocka::Signal s;
  mocka::Signal rs;
  s.setSize(256);
  rs.setSize(256);
  for (int i = 1; i <= 256; ++i)
  {
    s.push(std::cos(i * M_PIl / 32.0));
    rs.push(std::cos(i * M_PIl / 32.0));
  }
  s.fft()->ifft();

  for (int i = 0; i < 256; ++i)
  {
    EXPECT_NEAR(s.value(i), rs.value(i), 1e-10);
  }
}

TEST(functional_test, ifft_not_r2)
{
  mocka::Signal s;
  mocka::Signal rs;
  s.setSize(400);
  rs.setSize(400);
  for (int i = 1; i <= 400; ++i)
  {
    s.push(std::cos(i * M_PIl / 40.0));
    rs.push(std::cos(i * M_PIl / 40.0));
  }
  s.fft()->ifft();

  for (int i = 0; i < 400; ++i)
  {
    EXPECT_NEAR(s.value(i), rs.value(i), 1e-10);
  }
}

TEST(functional_test, lag_1)
{
  mocka::Signal s1;
  mocka::Signal s2;
  s1.setSize(200);
  s2.setSize(200);

  for (int i = 0; i < 200; ++i)
  {
    s1.push(std::cos(i * M_PIl / 20.0));
    s2.push(std::cos((i + 18) * M_PIl / 20.0));
  }

  EXPECT_EQ(s1.lag(s2), 18);
  EXPECT_EQ(s2.lag(s1), 22);
}

TEST(functional_test, lag_2)
{
  mocka::Signal s1;
  mocka::Signal s2;
  s1.setSize(200);
  s2.setSize(200);

  std::srand(std::time(nullptr));

  std::vector<double> noise;

  int var = 15;

  for (int i = 0; i < var; ++i)
  {
    int    r = std::rand() % 200;
    double t = 2 * M_PIl / (r + 1);
    noise.push_back(t);
  }

  for (int i = 0; i < 200; ++i)
  {
    int    r = std::rand() % 200;
    double t = 2 * M_PIl / (r + 1);
    noise.push_back(t);
    s1.push(t);
    s2.push(noise[i]);
  }

  EXPECT_EQ(s1.lag(s2), -var);
  EXPECT_EQ(s2.lag(s1), var);
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
