#ifndef FREQUENCYALIGN_HPP
#define FREQUENCYALIGN_HPP

#include <Eigen/Eigen>
#include <memory>

namespace mocka
{

class Signal
{
public:
  Signal();
  ~Signal();

public:
  Signal xcorr(const Signal& other) const;

  int lag(const Signal& other) const;

private:
  // helpler for signal processing
  std::pair<int, double> forwardLag(const Signal& other) const;

public:
  // buffer manegement
  void fill(const double& data);
  void push(const double& data);

public:
  // getters
  int getSize() const;

  double value(int pos) const;
  double re(int pos) const;
  double im(int pos) const;
  double abs(int pos) const;
  double arg(int pos) const;

  double& value(int pos);
  double& re(int pos);
  double& im(int pos);

public:
  // setters
  void setSize(int value);

public:
  int size;
  int current;

  std::shared_ptr<double> timeDomain;
  std::shared_ptr<double> frequencyDomain;

private:
  struct ArrayDeleter
  {
    void operator()(double const* p)
    {
      delete[] p;
    }
  };
};

class FrequencyAlign
{
public:
  FrequencyAlign();

public:
  void pushMeasuerment(const Eigen::Vector3d& data);
  void pushObservation(const Eigen::Vector3d& data);

public:
  Eigen::Vector3d getLag();

public:
  void setSize(const int& s);

private:
  Signal smeasure[3];
  Signal sobserve[3];
};

} // namespace mocka

#endif // FREQUENCYALIGN_HPP
