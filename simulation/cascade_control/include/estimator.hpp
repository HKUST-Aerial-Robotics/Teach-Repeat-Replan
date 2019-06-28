#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

namespace mocka
{

template <typename Input, typename Output>
class Estimator
{
public:
  typedef Input  In;
  typedef Output Out;

public:
  Estimator();
  virtual ~Estimator();

public:
  virtual Out update(const In& in) = 0;
  virtual void reset(const Out& outInit, const In& inInit)
  {
    in          = inInit;
    out         = outInit;
    initialized = true;
  }
  //! @todo add EKF fusion
  //  virtual void feedback(const Out& outMeasuerment) = 0;

public:
  bool isInitialized() const
  {
    return initialized;
  }

  void setInvaild()
  {
    initialized = false;
  }

protected:
  bool initialized;
  Out  out;
  In   in;
};

template <typename Input, typename Output>
Estimator<Input, Output>::Estimator()
  : initialized(false)
  , out()
  , in()
{
}

template <typename Input, typename Output>
Estimator<Input, Output>::~Estimator()
{
}

} // namespace  mocka

#endif // ESTIMATOR_HPP
