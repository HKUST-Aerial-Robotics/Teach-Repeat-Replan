#ifndef TRANSLATION_HPP
#define TRANSLATION_HPP

#include <Eigen/Core>

#include "controller.hpp"
#include "pid.hpp"
#include "pretreatmentor.hpp"

namespace mocka
{

class TranslationController
  : public Controller<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
{
public:
  TranslationController();
  virtual ~TranslationController();

public:
  //! @note getters
  PID* getController(const int& pos) const;
  PID* getXController() const;
  PID* getYController() const;
  PID* getZController() const;

public:
  //! @note setters
  void setFreq(double value);

  /*! @note coverage class Controller setters
   *        templeate implemented in cascadecontrol.cpp,
   *        not necessary to be annouced here
   */
  void setFeeds(const Feeds& value);
  void setFeeds(const Feeds& value, const uint64_t& time);

  void setCoeff(const Eigen::Matrix3d value);
  void setLimitation(const Eigen::Matrix3d value);

  double getError() const;

protected:
  virtual Output update();
  virtual void   updateInput();
  virtual void   updateFeeds();

private:
  Pretreatmentor<TranslationController::Feeds, Eigen::Vector3d>* feedback;

  PID* x;
  PID* y;
  PID* z;
};

} // namespace  mocka

#endif // TRANSLATION_HPP
