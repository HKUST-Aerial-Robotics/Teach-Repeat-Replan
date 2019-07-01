/** @file dji_singleton.hpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Singleton Template Class implementation for use with the DJI OSDK
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef SINGLETON_H
#define SINGLETON_H

namespace DJI
{
namespace OSDK
{

template <class T>
class Singleton
{
public:
  typedef T type;

protected:
  Singleton();

public:
  virtual ~Singleton()
  {
  }

public:
  static T& instance();
  static T* instancePTR();

protected:
  static T* singleInstance;
}; // class Singleton<T>

// template implementation
template <class T>
Singleton<T>::Singleton()
{
}

template <class T>
T&
Singleton<T>::instance()
{
  return *Singleton<T>::singleInstance;
}

template <class T>
T*
Singleton<T>::instancePTR()
{
  return Singleton<T>::singleInstance;
}

template <class T>
T* Singleton<T>::singleInstance = new T();

} // namespace OSDK
} // namespace DJI

#endif // SINGLETON_H
