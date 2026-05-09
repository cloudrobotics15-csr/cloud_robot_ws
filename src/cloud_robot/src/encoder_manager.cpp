#include "cloud_robot/encoder_manager.hpp"

#include <chrono>

namespace cloud_robot
{

EncoderManager::EncoderManager()
: running_(false)
{
}

EncoderManager::~EncoderManager()
{
    stop(); //evitar thread zombie
}

void EncoderManager::add_encoder(Encoder* enc)
{
    encoders_.push_back(enc);
}

void EncoderManager::start()
{
    if (running_) return;

    running_ = true;
    thread_ = std::thread(&EncoderManager::loop, this);
}

void EncoderManager::stop()
{
    running_ = false;

    if (thread_.joinable())
        thread_.join();
}

void EncoderManager::loop()
{
    using namespace std::chrono;

    const auto period = microseconds(50); 

    while (running_)
    {
        auto start = steady_clock::now();

        for (auto& enc : encoders_)
        {
            enc->update();
        }

        auto elapsed = steady_clock::now() - start;

        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

} // namespace cloud_robot
