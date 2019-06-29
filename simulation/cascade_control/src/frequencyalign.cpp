#include "frequencyalign.hpp"

/*#include <gsl/gsl_errno.h>
#include <gsl/gsl_fft_complex.h>*/

#include <algorithm>
#include <cstring>
#include <math.h>
#include <stdexcept>
#include <vector>

using namespace mocka;

Signal::Signal()
  : timeDomain(nullptr, ArrayDeleter())
  , frequencyDomain(nullptr, ArrayDeleter())
  , size(0)
  , current(0)
{
}

Signal::~Signal()
{
}

Signal
Signal::xcorr(const Signal& other) const
{
  Signal ans;
  if (other.getSize() != getSize())
  {
    throw std::runtime_error("unexpected xcorr input length");
  }
  ans.setSize(getSize());
  for (int i = 0; i < size; ++i)
  {
    double ans_i = 0;
    for (int j = 0; j < (size - i); ++j)
    {
      ans_i += value(j) * other.value(i + j);
    }
    ans.push(ans_i / static_cast<double>(size));
  }
  return ans;
}

std::pair<int, double>
Signal::forwardLag(const Signal& other) const
{
  std::pair<int, double> ans;

  std::vector<double> helpler;

  Signal xc(std::move(xcorr(other)));
  std::memcpy(xc.frequencyDomain.get(), xc.timeDomain.get(),
              size * sizeof(double));
  helpler.assign(xc.timeDomain.get(), xc.timeDomain.get() + size);
  std::sort(helpler.begin(), helpler.end(), std::greater<double>());
  double max = helpler[0];
  ans.first  = 0;
  for (int i = 0; i < size; ++i)
  {
    if (std::abs(xc.frequencyDomain.get()[i] - max) < 1e-10)
      ans.first = size - i - 1;
  }
  ans.second = 0.0;
  for (int i = 0; i < size - ans.first; ++i)
  {
    ans.second += abs(value(i) - other.value(i + ans.first));
  }
  ans.second /= static_cast<double>(size - ans.first);
  return ans;
}

int
Signal::lag(const Signal& other) const
{
  auto a1 = forwardLag(other);
  auto a2 = other.forwardLag(*this);

  if (a2.second < a1.second)
    return -a2.first;
  return a1.first;
}

void
Signal::fill(const double& data)
{
  for (int i = 0; i < (2 * size); ++i)
  {
    timeDomain.get()[i]      = data;
    frequencyDomain.get()[i] = data;
  }
}

void
Signal::push(const double& data)
{
  current--;
  if (current < 0)
    current   = size - 1;
  value(0)    = data;
  value(size) = data;
}

int
Signal::getSize() const
{
  return size;
}

double
Signal::value(int pos) const
{
  return timeDomain.get()[current + pos];
}

double
Signal::re(int pos) const
{
  return frequencyDomain.get()[2 * pos];
}

double
Signal::im(int pos) const
{
  return frequencyDomain.get()[2 * pos + 1];
}

double
Signal::abs(int pos) const
{
  double r = re(pos);
  double i = im(pos);
  return std::sqrt(r * r + i * i);
}

double
Signal::arg(int pos) const
{
  double r = re(pos);
  double i = im(pos);
  double l = abs(pos);

  if (i == 0)
  {
    return r > 0 ? 0 : M_PI;
  }
  else
    return std::atan2(i, l + r) * 2;
}

double&
Signal::value(int pos)
{
  return timeDomain.get()[current + pos];
}

double&
Signal::re(int pos)
{
  return frequencyDomain.get()[2 * pos];
}

double&
Signal::im(int pos)
{
  return frequencyDomain.get()[2 * pos + 1];
}

void
Signal::setSize(int value)
{
  if (value != size)
  {
    size = value;
    timeDomain.reset(new double[size * 2], ArrayDeleter());
    frequencyDomain.reset(new double[size * 2], ArrayDeleter());

    fill(0.0);
  }
}

FrequencyAlign::FrequencyAlign()
  : smeasure()
  , sobserve()
{
}

void
FrequencyAlign::pushMeasuerment(const Eigen::Vector3d& data)
{
  smeasure[0].push(data(0));
  smeasure[1].push(data(1));
  smeasure[2].push(data(2));
}

void
FrequencyAlign::pushObservation(const Eigen::Vector3d& data)
{
  sobserve[0].push(data(0));
  sobserve[1].push(data(1));
  sobserve[2].push(data(2));
}

Eigen::Vector3d
FrequencyAlign::getLag()
{
  return Eigen::Vector3d(sobserve[0].lag(smeasure[0]),
                         sobserve[1].lag(smeasure[1]),
                         sobserve[2].lag(smeasure[2]));
}

void
FrequencyAlign::setSize(const int& s)
{
  smeasure[0].setSize(s);
  sobserve[0].setSize(s);
  smeasure[1].setSize(s);
  sobserve[1].setSize(s);
  smeasure[2].setSize(s);
  sobserve[2].setSize(s);
}
