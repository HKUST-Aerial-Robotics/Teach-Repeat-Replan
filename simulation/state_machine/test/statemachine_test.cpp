#include <gtest/gtest.h>

#include "boost/sml/utility/dispatch_table.hpp"
#include "statemachine.hpp"

namespace sml = boost::sml;

TEST(empty_test, critical_value_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger    logger;
  StateMachine sm{ logger };
  auto         dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  OperationalLogic::RuntimeEvent e{ 0 };
  for (int i = 0; i < 20; ++i)
    dispatch_event(e, e.id);

  // clang-format off
  EXPECT_EQ(true, sm.is("power_on"_s));
  // clang-format on
}

TEST(empty_event_test, critical_value_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger         logger;
  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  auto              dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  logic->getFIFO()->push(OperationalLogic::EmptyEvent::id);
  logic->getFIFO()->push(OperationalLogic::EmptyEvent::id);
  logic->getFIFO()->push(OperationalLogic::EmptyEvent::id);

  for (int i = 0; i < 20; ++i)
  {
    int id = logic->getFIFO()->pop();

    OperationalLogic::RuntimeEvent e{ id };

    if (id)
      dispatch_event(e, e.id);
    else
      sm.process_event(OperationalLogic::EmptyEvent{});
  }
  // clang-format off
  EXPECT_EQ(true, sm.is("power_on"_s ));
  // clang-format on
}

TEST(quit_rc_in_land, rc_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger         logger;
  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  auto              dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  logic->getFIFO()->push(OperationalLogic::RegisterSDK::id);
  logic->getFIFO()->push(OperationalLogic::REQEnterRC::id);
  logic->getFIFO()->push(OperationalLogic::REQQuitRC::id);

  for (int i = 0; i < 20; ++i)
  {
    int id = logic->getFIFO()->pop();

    OperationalLogic::RuntimeEvent e{ id };

    dispatch_event(e, e.id);
  }
  // clang-format off
  EXPECT_EQ(true, sm.is("ground_idle"_s ));
  // clang-format on
}

TEST(quit_rc_in_sky, rc_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger         logger;
  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  auto              dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  logic->getFIFO()->push(OperationalLogic::RegisterSDK::id);
  logic->getFIFO()->push(OperationalLogic::REQEnterRC::id);
  logic->getFIFO()->push(OperationalLogic::TakeOffRunning::id);
  logic->getFIFO()->push(OperationalLogic::REQQuitRC::id);

  for (int i = 0; i < 20; ++i)
  {
    int id = logic->getFIFO()->pop();

    OperationalLogic::RuntimeEvent e{ id };

    dispatch_event(e, e.id);
  }
  // clang-format off
  EXPECT_EQ(true, sm.is("sky_idle"_s ));
  // clang-format on
}

TEST(quit_mission_in_land, mission_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger         logger;
  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  auto              dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  logic->getFIFO()->push(OperationalLogic::RegisterSDK::id);
  logic->getFIFO()->push(OperationalLogic::REQEnterMission::id);
  logic->getFIFO()->push(OperationalLogic::REQQuitMission::id);

  for (int i = 0; i < 20; ++i)
  {
    int id = logic->getFIFO()->pop();

    OperationalLogic::RuntimeEvent e{ id };

    dispatch_event(e, e.id);
  }
  // clang-format off
  EXPECT_EQ(true, sm.is("ground_idle"_s ));
  // clang-format on
}

TEST(quit_mission_in_sky, mission_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger         logger;
  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  auto              dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  logic->getFIFO()->push(OperationalLogic::RegisterSDK::id);
  logic->getFIFO()->push(OperationalLogic::REQEnterMission::id);
  logic->getFIFO()->push(OperationalLogic::TakeOffRunning::id);
  logic->getFIFO()->push(OperationalLogic::REQQuitMission::id);

  for (int i = 0; i < 20; ++i)
  {
    int id = logic->getFIFO()->pop();

    OperationalLogic::RuntimeEvent e{ id };

    dispatch_event(e, e.id);
  }
  // clang-format off
  EXPECT_EQ(true, sm.is("sky_idle"_s ));
  // clang-format on
}

TEST(sky_idle, crash_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger         logger;
  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  auto              dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  logic->getFIFO()->push(OperationalLogic::RegisterSDK::id);
  logic->getFIFO()->push(OperationalLogic::TakeOffRunning::id);
  logic->getFIFO()->push(OperationalLogic::CrashDetected::id);

  for (int i = 0; i < 20; ++i)
  {
    int id = logic->getFIFO()->pop();

    OperationalLogic::RuntimeEvent e{ id };

    dispatch_event(e, e.id);
  }
  // clang-format off
  EXPECT_EQ(true, sm.is("crashed"_s ));
  // clang-format on
}

TEST(rescue_sky_idle, crash_test)
{
  using namespace mocka;
  using namespace sml;

  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

  SMLLogger         logger;
  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  auto              dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 0, 20>(
      sm);

  logic->getFIFO()->push(OperationalLogic::RegisterSDK::id);
  logic->getFIFO()->push(OperationalLogic::TakeOffRunning::id);
  logic->getFIFO()->push(OperationalLogic::CrashDetected::id);

  for (int i = 0; i < 20; ++i)
  {
    int id = logic->getFIFO()->pop();

    OperationalLogic::RuntimeEvent e{ id };

    dispatch_event(e, e.id);
  }
  // clang-format off
  EXPECT_EQ(true, sm.is("crashed"_s ));
  // clang-format on

  sleep(1);
  sm.process_event(OperationalLogic::EmptyEvent{});

  sleep(1);
  //  usleep(5e5);
  sm.process_event(OperationalLogic::EmptyEvent{});

  // clang-format off
  EXPECT_EQ(true, sm.is("crashed"_s ));
  // clang-format on
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
