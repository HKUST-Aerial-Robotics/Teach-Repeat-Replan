#ifndef SAFEFIFO_HPP
#define SAFEFIFO_HPP

#include <mutex>
#include <queue>

namespace mocka
{

template <class T>
class SafeFIFO
{
public:
  SafeFIFO();

public:
  void push(const T& e);
  T    pop();
  int  length() const;
  void clear();
  //! @note SafeFIFO have no front method, to avoid thread safty issue

private:
  std::queue<T> fifo;
  std::mutex    mutex;
}; // class SafeFIFO

template <class T>
SafeFIFO<T>::SafeFIFO()
{
}

template <class T>
void
SafeFIFO<T>::push(const T& e)
{
  mutex.lock();
  fifo.push(e);
  mutex.unlock();
}

template <class T>
T
SafeFIFO<T>::pop()
{
  T ans{ 0 };
  if (fifo.empty())
    return ans;
  mutex.lock();
  ans = fifo.front();
  fifo.pop();
  mutex.unlock();
  return ans;
}

template <class T>
int
SafeFIFO<T>::length() const
{
  return fifo.size();
}

} // namespace mocka

#endif // SAFEFIFO_HPP
