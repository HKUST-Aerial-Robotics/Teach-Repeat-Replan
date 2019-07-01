#include "self.hpp"

#include <net/if.h>
#include <netinet/in.h>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>

using namespace mocka;

Self::Self(std::shared_ptr<mocka::Link> l)
  : link(l)
{
  if (!readMAC())
    throw std::runtime_error("Cannot access to an available MAC address");
  if (!Self::addInput("global_rc", new LinkItem<sensor_msgs::Joy>()))
    throw std::runtime_error("fail to add input");
  if (!Self::addOutput("position", new LinkItem<geometry_msgs::Pose>()))
    throw std::runtime_error("fail to add output");
}

void
Self::poll()
{
  if (syncedFellow < networkSize)
  {
    syncedFellow += syncLinkage();
  }
}

bool
Self::addInput(std::string&& name, LinkItemBase* clause)
{
  if (link->addInput(name, clause))
    syncedFellow = 0;
  else
    return false;
  return true;
}

bool
Self::addOutput(std::string&& name, LinkItemBase* clause)
{
  if (link->addOutput(name, clause))
    syncedFellow = 0;
  else
    return false;
  return true;
}

std::shared_ptr<Link>
Self::getLink() const
{
  return link;
}

int
Self::syncLinkage()
{
  return 0;
}

bool
Self::readMAC()
{
  struct ifreq  ifr;
  struct ifconf ifc;
  char          buf[1024];
  int           success = 0;

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock == -1)
  { /* handle error*/
    return false;
  };

  ifc.ifc_len = sizeof(buf);
  ifc.ifc_buf = buf;
  if (ioctl(sock, SIOCGIFCONF, &ifc) == -1)
  { /* handle error */
    return false;
  }

  struct ifreq*             it  = ifc.ifc_req;
  const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

  for (; it != end; ++it)
  {
    strcpy(ifr.ifr_name, it->ifr_name);
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0)
    {
      if (!(ifr.ifr_flags & IFF_LOOPBACK))
      { // don't count loopback
        if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0)
        {
          success = 1;
          break;
        }
      }
    }
    else
    { /* handle error */
      return false;
    }
  }

  unsigned char mac_address[6];

  if (success)
  {
    memcpy(mac_address, ifr.ifr_hwaddr.sa_data, 6);
  }
  else
    return false;

  std::stringstream s;

  s << std::hex << std::uppercase << std::setfill('0');
  for (int i = 0; i < 5; ++i)
  {
    s << std::setw(2) << (int)mac_address[i] << "_";
  }
  s << std::setw(2) << (int)mac_address[5];

  std::string name = s.str();

  std::cout << "MAC: " << name << std::endl;
  link->setName(name);

  return true;
}

void
Self::setNetworkSize(int value)
{
  networkSize = value;
}
