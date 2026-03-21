#include "cloud_robot/encoder.hpp"

namespace cloud_robot
{

Encoder::Encoder(int pi, int pin_a, int pin_b)
: pi_(pi),
  pin_a_(pin_a),
  pin_b_(pin_b),
  last_a_(0),
  last_b_(0),
  ticks_(0)
{
  set_mode(pi_, pin_a_, PI_INPUT);
  set_mode(pi_, pin_b_, PI_INPUT);

  set_pull_up_down(pi_, pin_a_, PI_PUD_UP);
  set_pull_up_down(pi_, pin_b_, PI_PUD_UP);

  last_a_ = gpio_read(pi_, pin_a_);
  last_b_ = gpio_read(pi_, pin_b_);

  callback_ex(
    pi_,
    pin_a_,
    EITHER_EDGE,
    Encoder::callback,
    this
  );

  callback_ex(
    pi_,
    pin_b_,
    EITHER_EDGE,
    Encoder::callback,
    this
  );
}


void Encoder::callback(
  int /*pi*/,
  unsigned /*gpio*/,
  unsigned /*level*/,
  uint32_t /*tick*/,
  void * user)
{
  Encoder * enc =
    static_cast<Encoder *>(user);

  int a = gpio_read(enc->pi_, enc->pin_a_);
  int b = gpio_read(enc->pi_, enc->pin_b_);

  if (a == enc->last_a_ &&
      b == enc->last_b_)
    return;

  if (a == b)
    enc->ticks_++;
  else
    enc->ticks_--;

  enc->last_a_ = a;
  enc->last_b_ = b;
}


int32_t Encoder::get_ticks() const
{
  return ticks_.load();
}


void Encoder::reset()
{
  ticks_.store(0);
}

}// namespace cloud_robot