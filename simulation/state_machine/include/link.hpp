#ifndef LINK_HPP
#define LINK_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mpaux.hpp"

namespace mocka
{

struct Bandwidth
{
  int usedBandwidth;
  int usedBandwidthMax;
  int usedBandwidthMin;
};

class LinkItemBase
{
public:
  LinkItemBase();
  virtual ~LinkItemBase();

public:
  Bandwidth getBandwidth() const;

public:
  void setBandwidth(const Bandwidth& value);

private:
  Bandwidth bandwidth;

}; // class LinkItemBase

template <class... Ts>
class LinkItem : public LinkItemBase
{
public:
  LinkItem();
  LinkItem(Ts... ts);

public:
  aux::Tuple<Ts...>& getResource();

  template <int k>
  auto&         get()
  {
    return aux::__impl::get<k>(&resource);
  }

private:
  aux::Tuple<Ts...> resource;
};

template <class... Ts>
LinkItem<Ts...>::LinkItem()
  : resource()
{
}

template <class... Ts>
LinkItem<Ts...>::LinkItem(Ts... ts)
  : resource(ts...)
{

} // class LinkItem

//! @todo remove, this feature moved to lambound
class Link
{
public:
  Link();

public:
  void poll();
  bool addInput(std::string& name, LinkItemBase* clause);
  bool addInput(std::string&& name, LinkItemBase* clause);
  bool addOutput(std::string& name, LinkItemBase* clause);
  bool addOutput(std::string&& name, LinkItemBase* clause);

public:
  std::string getName() const;

  //! @note shared_ptr reference cannot cast, so we return the raw pointer
  template <class T = LinkItemBase>
  T* getInput(std::string&& name)
  {
    return reinterpret_cast<T*>(input[name].get());
  }

  template <class T = LinkItemBase>
  T* getOutput(std::string&& name)
  {
    return reinterpret_cast<T*>(output[name].get());
  }

public:
  void setName(const std::string& value);

private:
  // basic info
  std::string name;
  std::string MAC;

  int maxBandwidthUpload;
  int maxBandwidthDownload;

  Bandwidth upload;
  Bandwidth download;

  std::map<std::string, std::shared_ptr<LinkItemBase> > input;
  std::map<std::string, std::shared_ptr<LinkItemBase> > output;
}; // class LinkItem

// template LinkItem function definition below

} // namespace mocka

#endif // LINK_HPP
