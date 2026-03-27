#pragma once
#include <atomic>
#include <cstdint>
#include <pigpiod_if2.h>

namespace cloud_robot
{

class Encoder
{
public:

  Encoder(int pi, int pin_a, int pin_b);
  
  void update();

  int32_t get_ticks() const;

  void reset();

private:

  int pi_;

  int pin_a_;
  int pin_b_;

  int last_a_;
  int last_b_;

  std::atomic<int32_t> ticks_;
};

}// namespace cloud_robot


