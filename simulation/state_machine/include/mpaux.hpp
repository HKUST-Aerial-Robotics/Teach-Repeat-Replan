#ifndef MPAUX_HPP
#define MPAUX_HPP

#include <cstdint>
#include <type_traits>

/*! @note meta programming aux
 *  @todo move to an MPL library
 * */

namespace mocka
{

namespace aux
{

template <class... Ts>
struct Tuple;

namespace __impl
{
template <int k, class... Ts>
struct TypeHolder;

template <int k, class... Ts>
typename std::enable_if<k == 0,
                        typename TypeHolder<0, Tuple<Ts...> >::type&>::type
get(Tuple<Ts...>* t)
{
  return t->tail;
}

template <int k, class T, class... Ts>
typename std::enable_if<k != 0,
                        typename TypeHolder<k, Tuple<T, Ts...> >::type&>::type
get(Tuple<T, Ts...>* t)
{
  Tuple<Ts...>* base = t;
  return get<k - 1>(base);
}

template <int k, class T, class... Ts>
struct TypeHolder<k, Tuple<T, Ts...> >
{
  static_assert(k <= (sizeof...(Ts)), "Unreachable data segment");
  typedef typename TypeHolder<k - 1, Tuple<Ts...> >::type type;
};

template <class T, class... Ts>
struct TypeHolder<0, Tuple<T, Ts...> >
{
  typedef T type;
};

} // namespace __impl

//! @note std::tuple is stupid, so I add my own tuple

template <class... Ts>
struct Tuple
{
  template <int k>

  auto& get()
  {
    return __impl::get<k>(this);
  }
};

template <class T, class... Ts>
struct Tuple<T, Ts...> : Tuple<Ts...>
{
  Tuple()
    : Tuple<Ts...>()
    , tail()
  {
  }

  Tuple(T t, Ts... ts)
    : Tuple<Ts...>(ts...)
    , tail(t)
  {
  }

  T tail;

  template <int k>

  auto& get()
  {
    return __impl::get<k>(this);
  }
};

} // namespace aux

} // namespace mocka

#endif // MPAUX_HPP
