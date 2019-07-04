#ifndef FLIGHTACTIONS_HPP
#define FLIGHTACTIONS_HPP

class FlightActions
{
public:
  FlightActions();

  typedef enum EVENT {
    EVENT_NONE,

  } EVENT;

public:
  void takeoff(double height);
  void landing();
};

#endif // FLIGHTACTIONS_HPP
