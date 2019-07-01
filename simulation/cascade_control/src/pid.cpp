#include "pid.hpp"
#include <iostream>

using namespace mocka;

PID::PID()
  : kp(1)
  , ki(0)
  , kd(0)
  , error(0)
  , lastError(0)
  , intergrade(0)
  , min(-1e20)
  , max(1e20)
  , intergradeThreshold(1e20)
{
  setFeeds(0);
  setInput(0);
  clearOutput();
}

PID::~PID()
{
}

bool
PID::checkNAN(double& ans)
{
  if (ans != ans) //! @note NAN
  {
    ans = 0;
    return true;
  }
  return false;
}

PID::Output
PID::update()
{
  double ans;
  checkNAN(in);
  error = in - feed;
  checkNAN(error);
  ans = P() + I() + D();
  checkNAN(ans);
  if (ans > max)
    return max;
  if (ans < min)
    return min;
  return ans;
}

void
PID::updateInput()
{
  //! @note ZOH nothing todo here
}

void
PID::updateFeeds()
{
  //! @note ZOH nothing todo here
}

PID::Output
PID::P()
{
  return error * kp;
}

PID::Output
PID::I()
{
  if (std::fabs(error) < intergradeThreshold)
  {
    intergrade += error;
  }
  else
  {
    intergrade = 0;
  }
  checkNAN(intergrade);
  return intergrade * ki;
}

PID::Output
PID::D()
{
  double tmp = lastError - error;
  if (checkNAN(tmp))
  {
    lastError = 0;
  }
  lastError = error;
  return tmp * kd;
}

double
PID::getError() const
{
  return error;
}

double
PID::getIntergradeThreshold() const
{
  return intergradeThreshold;
}

void
PID::setIntergradeThreshold(double value)
{
  intergradeThreshold = value;
}

double
PID::getMin() const
{
  return min;
}

void
PID::setMin(double value)
{
  min = value;
}

double
PID::getMax() const
{
  return max;
}

void
PID::setMax(double value)
{
  max = value;
}

double
PID::getIntergrade() const
{
  return intergrade;
}

void
PID::setCoeff(const Eigen::Vector3d pid)
{
  setKp(pid(0));
  setKi(pid(1));
  setKd(pid(2));
}

void
PID::setLimitaion(const Eigen::Vector3d limit)
{
  setMax(limit(0));
  setMin(limit(1));
  setIntergradeThreshold(limit(2));
}

double
PID::getKd() const
{
  return kd;
}

void
PID::setKd(const double value)
{
  kd = value;
}

double
PID::getKi() const
{
  return ki;
}

void
PID::setKi(const double value)
{
  ki = value;
}

double
PID::getKp() const
{
  return kp;
}

void
PID::setKp(const double value)
{
  kp = value;
}
