#include <gtest/gtest.h>
#include <map>

#include "mission.hpp"
#include "mpaux.hpp"

TEST(mission_manager, language_test)
{
  static std::map<size_t, char> tst;

  class CDMission : public mocka::MissionBase
  {
  public:
    CDMission()
    {
      std::cout << "~~~ constructor " << (size_t) this << std::endl;
      tst[(size_t) this] = '0';
    }
    ~CDMission()
    {
      std::cout << "~~~ disstructor " << (size_t) this << std::endl;
      tst.erase((size_t) this);
    }

    virtual mocka::RESULT poll()
    {
      return mocka::MISSION_RESULT_FINISHED;
    }
  };
  mocka::MissionManager mgr;

  mgr.push(new CDMission);
  mgr.push(new CDMission);
  EXPECT_EQ(mgr.pop()->poll(), mocka::MISSION_RESULT_FINISHED);
  EXPECT_EQ(tst.size(), 1);
  mgr.push(new CDMission);
  mgr.pop();
  mgr.pop();
  mgr.pop();

  EXPECT_EQ(tst.empty(), true);
}

TEST(getter_setter_and_poll, language_test)
{
  auto function = [](mocka::Mission<double, int>* t) -> mocka::RESULT {
    return (t->get<0>() + t->get<1>()) > 3 ? mocka::MISSION_RESULT_FINISHED
                                           : mocka::MISSION_RESULT_WIP;
  };

  mocka::Mission<double, int> m(function, 1.0, 2);

  //  EXPECT_EQ(m.getResource())
  EXPECT_EQ(m.poll(), mocka::MISSION_RESULT_WIP);

  m.get<0>() = 2.0;
  m.get<1>() = 1;

  EXPECT_EQ(m.poll(), mocka::MISSION_RESULT_WIP);

  m.getResource().get<1>() = 5;

  EXPECT_EQ(m.poll(), mocka::MISSION_RESULT_FINISHED);
}

TEST(mission_manager, feature_test)
{
  mocka::MissionManager mgr;
  auto                  function =
    [](mocka::Mission<double, std::shared_ptr<int> >* t) -> mocka::RESULT {
    return (t->get<0>() + *t->get<1>()) > 3 ? mocka::MISSION_RESULT_FINISHED
                                            : mocka::MISSION_RESULT_WIP;
  };
  auto ptr = std::make_shared<int>(2);
  auto m   = new mocka::Mission<double, std::shared_ptr<int> >(
    function, 1.0, std::shared_ptr<int>(new int(2)));
  mgr.push(m);
  mgr.push(new mocka::Mission<double, std::shared_ptr<int> >(
    function, 1.0, std::shared_ptr<int>(ptr)));
  mgr.push(new mocka::Mission<double, std::shared_ptr<int> >(
    function, 1.0, std::shared_ptr<int>(ptr)));

  EXPECT_EQ(mgr.length(), 3);
  mgr.poll();
  EXPECT_EQ(mgr.length(), 2);
  m->get<0>() = 2;
  mgr.poll();
  EXPECT_EQ(mgr.length(), 1);
  mgr.poll();
  EXPECT_EQ(mgr.length(), 1);
  *ptr = 3;
  mgr.poll();
  EXPECT_EQ(mgr.length(), 0);
  EXPECT_EQ(mgr.isEmpty(), false);
  mgr.poll();
  EXPECT_EQ(mgr.length(), 0);
  EXPECT_EQ(mgr.isEmpty(), true);
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
