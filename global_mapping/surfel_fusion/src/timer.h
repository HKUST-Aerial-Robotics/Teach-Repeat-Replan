#pragma once
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <string>

using namespace std;

class Timer
{
  public:
    Timer(string this_timer_name)
    {
        start_time = std::chrono::system_clock::now();
        middle_time = std::chrono::system_clock::now();
        timer_name = this_timer_name;
        middle_name = this_timer_name;
    }

    void middle(string new_middle_name)
    {
        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
        std::chrono::duration<double> used_time = end - middle_time;
        printf("from %s to %s cost %f ms.\n", middle_name.c_str(), new_middle_name.c_str(), used_time.count() * 1000.0);
        middle_time = std::chrono::system_clock::now();
        middle_name = new_middle_name;
    }

    void end()
    {
        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
        std::chrono::duration<double> used_time = end - start_time;
        printf("%s total cost %f ms.\n", timer_name.c_str(), used_time.count() * 1000.0);
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start_time;
    std::chrono::time_point<std::chrono::system_clock> middle_time;
    string timer_name;
    string middle_name;
};
