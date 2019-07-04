#ifndef ACCELERATION_HPP
#define ACCELERATION_HPP

#include "translation_control.hpp"

namespace mocka
{

class AccelerationController : public TranslationController
{

public:
  AccelerationController();
};

} // namespace mocka

#endif // ACCELERATION_HPP
