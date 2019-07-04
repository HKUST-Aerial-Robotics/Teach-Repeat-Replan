#include "link.hpp"

#include <iostream>

using namespace mocka;

Link::Link()
  : input()
  , output()
{
}

bool
Link::addInput(std::string& name, LinkItemBase* clause)
{
  if (input.find(name) != input.end())
    return false;
  input[name] = std::shared_ptr<LinkItemBase>(clause);
  return true;
}

bool
Link::addInput(std::string&& name, LinkItemBase* clause)
{
  if (input.find(name) != input.end())
    return false;
  input[name] = std::shared_ptr<LinkItemBase>(clause);
  return true;
}

bool
Link::addOutput(std::string& name, LinkItemBase* clause)
{
  if (output.find(name) != output.end())
    return false;
  output[name] = std::shared_ptr<LinkItemBase>(clause);
  return true;
}

bool
Link::addOutput(std::string&& name, LinkItemBase* clause)
{
  if (output.find(name) != output.end())
    return false;
  output[name] = std::shared_ptr<LinkItemBase>(clause);
  return true;
}

LinkItemBase::LinkItemBase()
{
}

LinkItemBase::~LinkItemBase()
{
}

Bandwidth
LinkItemBase::getBandwidth() const
{
  return bandwidth;
}

void
LinkItemBase::setBandwidth(const Bandwidth& value)
{
  bandwidth = value;
}

std::string
Link::getName() const
{
  return name;
}

void
Link::setName(const std::string& value)
{
  name = value;
}
