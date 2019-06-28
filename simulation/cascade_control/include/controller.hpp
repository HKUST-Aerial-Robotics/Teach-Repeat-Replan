#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <stdint.h>
//#include <cstdint>

namespace mocka
{

template <typename Input_, typename Output_, typename Feeds_>
class Controller
{
public:
  typedef Input_  Input;
  typedef Output_ Output;
  typedef Feeds_  Feeds;

public:
  Controller();
  virtual ~Controller();

protected:
  virtual uint64_t getCurrent();

  virtual Output update()      = 0;
  virtual void   updateInput() = 0;
  virtual void   updateFeeds() = 0;

public:
  void spinOnce();

private:
  void updateTime();

public:
  // getters
  double   getFreq() const;
  Feeds    getFeeds() const;
  Output   getOutput() const;
  Input    getInput() const;
  uint64_t getInputTime_ns() const;
  uint64_t getFeedsTime_ns() const;

  // setters
  void setFreq(double value);
  void setInput(const Input& value);
  void setInput(const Input& value, const uint64_t& time);
  void setFeeds(const Feeds& value);
  void setFeeds(const Feeds& value, const uint64_t& time);
  void clearOutput();

protected:
  uint64_t time_ns;      // last spin time
  uint64_t inputTime_ns; // latest update time
  uint64_t feedsTime_ns; // latest update time

  Input  in;
  Output out;
  Feeds  feed;

  double freq;
}; // class Controller

template <typename Input_, typename Output_, typename Feeds_>
Controller<Input_, Output_, Feeds_>::Controller()
{
}

template <typename Input_, typename Output_, typename Feeds_>
Controller<Input_, Output_, Feeds_>::~Controller()
{
}

template <typename Input_, typename Output_, typename Feeds_>
uint64_t
Controller<Input_, Output_, Feeds_>::getCurrent()
{
  return 0; //! @todo implement
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::spinOnce()
{
  updateTime();
  updateInput();
  updateFeeds();
  out = update();
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::updateTime()
{
  //! @todo implement
}

template <typename Input_, typename Output_, typename Feeds_>
double
Controller<Input_, Output_, Feeds_>::getFreq() const
{
  return freq;
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::setFreq(double value)
{
  freq = value;
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::setInput(const Input& value)
{
  in           = value;
  inputTime_ns = getCurrent();
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::setInput(const Input& value,
                                              const uint64_t& time)
{
  in           = value;
  inputTime_ns = time;
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::setFeeds(const Feeds& value)
{
  feed         = value;
  feedsTime_ns = getCurrent();
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::setFeeds(const Feeds& value,
                                              const uint64_t& time)
{
  feed         = value;
  feedsTime_ns = time;
}

template <typename Input_, typename Output_, typename Feeds_>
void
Controller<Input_, Output_, Feeds_>::clearOutput()
{
  out = 0;
}

template <typename Input_, typename Output_, typename Feeds_>
typename Controller<Input_, Output_, Feeds_>::Feeds
Controller<Input_, Output_, Feeds_>::getFeeds() const
{
  return feed;
}

template <typename Input_, typename Output_, typename Feeds_>
typename Controller<Input_, Output_, Feeds_>::Output
Controller<Input_, Output_, Feeds_>::getOutput() const
{
  return out;
}

template <typename Input_, typename Output_, typename Feeds_>
typename Controller<Input_, Output_, Feeds_>::Input
Controller<Input_, Output_, Feeds_>::getInput() const
{
  return in;
}

template <typename Input_, typename Output_, typename Feeds_>
uint64_t
Controller<Input_, Output_, Feeds_>::getInputTime_ns() const
{
  return inputTime_ns;
}

template <typename Input_, typename Output_, typename Feeds_>
uint64_t
Controller<Input_, Output_, Feeds_>::getFeedsTime_ns() const
{
  return feedsTime_ns;
}

} // namespace
#endif // CONTROLLER_HPP
