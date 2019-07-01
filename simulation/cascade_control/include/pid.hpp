#ifndef PID_HPP
#define PID_HPP

#include "controller.hpp"

#include <Eigen/Core>

namespace mocka
{

class PID : public Controller<double, double, double>
{
public:
  PID();
  virtual ~PID();

public:
  //! @note getters
  double getKp() const;
  double getKi() const;
  double getKd() const;
  double getIntergrade() const;

  double getMax() const;
  double getMin() const;
  double getError() const;

  double getIntergradeThreshold() const;

public:
  //! @note setters
  void setCoeff(const Eigen::Vector3d pid);
  void setLimitaion(const Eigen::Vector3d limit);

  void setKp(const double value);
  void setKi(const double value);
  void setKd(const double value);

  void setMax(double value);
  void setMin(double value);

  void setIntergradeThreshold(double value);

  bool checkNAN(double& ans);

protected:
  virtual Output update();
  virtual void   updateInput();
  virtual void   updateFeeds();

private:
  Output P();
  Output I();
  Output D();

private:
  double max;
  double min;

  double error;
  double lastError;

  double kp;
  double ki;
  double kd;

  double intergradeThreshold;
  double intergrade;
};

} // namespace  mocka

#endif // PID_HPP
