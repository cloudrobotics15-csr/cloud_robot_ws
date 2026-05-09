#include "cloud_robot/encoder.hpp"
#include <pigpiod_if2.h>

namespace cloud_robot
{

Encoder::Encoder(int pi, int pin_a, int pin_b)
: pi_(pi), pin_a_(pin_a), pin_b_(pin_b), ticks_(0)
{
    set_mode(pi_, pin_a_, PI_INPUT);
    set_mode(pi_, pin_b_, PI_INPUT);

    set_pull_up_down(pi_, pin_a_, PI_PUD_UP);
    set_pull_up_down(pi_, pin_b_, PI_PUD_UP);

    last_a_ = gpio_read(pi_, pin_a_);
}

int32_t Encoder::get_ticks() const
{
    return ticks_.load(std::memory_order_relaxed);
}

void Encoder::update()
{
    int a = gpio_read(pi_, pin_a_);

    // solo si A cambia → contamos (x2)
    if (a != last_a_)
    {
        int b = gpio_read(pi_, pin_b_);

        // dirección
        if (b != a)
            ticks_++;
        else
            ticks_--;
    }

    last_a_ = a;
}

void Encoder::reset()
{
    ticks_.store(0);
}

} // namespace cloud_robot
