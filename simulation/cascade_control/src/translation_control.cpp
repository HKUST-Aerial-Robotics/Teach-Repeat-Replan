#include "translation_control.hpp"

#include <stdexcept>

using namespace mocka;

TranslationController::TranslationController()
  : x(new PID())
  , y(new PID())
  , z(new PID())
{
}

TranslationController::~TranslationController()
{
  delete x;
  delete y;
  delete z;
}

PID*
TranslationController::getController(const int& pos) const
{
  switch (pos)
  {
    case 0:
      return getXController();
    case 1:
      return getYController();
    case 2:
      return getZController();
    default:
      throw std::runtime_error("unexpected controller getter");
      break;
  }
}

TranslationController::Output
TranslationController::update()
{
  Eigen::Vector3d output;

  Eigen::Vector3d input     = getInput();
  uint64_t        inputTime = getInputTime_ns();

  x->setInput(input(0), inputTime);
  y->setInput(input(1), inputTime);
  z->setInput(input(2), inputTime);

  x->spinOnce();
  y->spinOnce();
  z->spinOnce();

  output(0) = x->getOutput();
  output(1) = y->getOutput();
  output(2) = z->getOutput();

  return output;
}

void
TranslationController::updateInput()
{
}

void
TranslationController::updateFeeds()
{
}

double
TranslationController::getError() const
{
  return Eigen::Vector3d(x->getError(), y->getError(), z->getError()).norm();
}

PID*
TranslationController::getZController() const
{
  return z;
}

PID*
TranslationController::getYController() const
{
  return y;
}

PID*
TranslationController::getXController() const
{
  return x;
}

namespace mocka
{

void
TranslationController::setFreq(double value)
{
  Controller<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>::setFreq(value);
  x->setFreq(value);
  y->setFreq(value);
  z->setFreq(value);
}

void
TranslationController::setFeeds(const Controller::Feeds& value)
{
  this->setFeeds(value, getCurrent());
}

void
TranslationController::setFeeds(const Controller::Feeds& value,
                                const uint64_t&          time)
{
  feed         = value;
  feedsTime_ns = time;
  x->setFeeds(value(0), time);
  y->setFeeds(value(1), time);
  z->setFeeds(value(2), time);
}

void
TranslationController::setCoeff(const Eigen::Matrix3d value)
{
  x->setCoeff(value.row(0));
  y->setCoeff(value.row(1));
  z->setCoeff(value.row(2));
}

void
TranslationController::setLimitation(const Eigen::Matrix3d value)
{
  x->setLimitaion(value.row(0));
  y->setLimitaion(value.row(1));
  z->setLimitaion(value.row(2));
}
} // namespace mocka
