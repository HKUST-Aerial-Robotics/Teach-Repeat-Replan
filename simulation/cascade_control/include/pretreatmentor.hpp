#ifndef PRETREATMENTOR_HPP
#define PRETREATMENTOR_HPP

namespace mocka
{

template <typename Input_, typename Output_>
class Pretreatmentor
{
public:
  typedef Input_  Input;
  typedef Output_ Output;

public:
  Pretreatmentor();
  virtual ~Pretreatmentor();

public:
  virtual Output update(Input input) = 0;
};

template <typename Input_, typename Output_>
Pretreatmentor<Input_, Output_>::Pretreatmentor()
{
}

template <typename Input_, typename Output_>
Pretreatmentor<Input_, Output_>::~Pretreatmentor()
{
}

} // namespace mocka

#endif // PRETREATMENTOR_HPP
