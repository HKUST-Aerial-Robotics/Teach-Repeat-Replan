#ifndef VELOCITY_HPP
#define VELOCITY_HPP

#include "mixer.hpp"
#include "translation_control.hpp"

namespace mocka
{

class VelocityController : public TranslationController
{
public:
  VelocityController();

  // protected:
  //  virtual Output update();
};
} // namespace mocka
#endif // VELOCITY_HPP
