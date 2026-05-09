#pragma once

#include "cloud_robot/encoder.hpp"

#include <vector>
#include <thread>
#include <atomic>

namespace cloud_robot
{

class EncoderManager
{
public:
    EncoderManager();
    ~EncoderManager();

    void add_encoder(Encoder* enc);

    void start();
    void stop();

private:
    void loop();

private:
    std::vector<Encoder*> encoders_;

    std::atomic<bool> running_;
    std::thread thread_;
};

} // namespace cloud_robot
