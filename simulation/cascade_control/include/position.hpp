#ifndef POSITION_HPP
#define POSITION_HPP

#include "translation_control.hpp"

namespace mocka
{

class PositionController : public TranslationController
{
public:
  PositionController();
};

} // namespace mocka
#endif // POSITION_HPP
