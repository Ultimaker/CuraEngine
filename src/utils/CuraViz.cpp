// Copyright (c) 2026 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/CuraViz.h"

#include <spdlog/spdlog.h>
#ifdef ENABLE_CURAVIZ

#include "cura_viz/point2ll.pb.h"
#include "cura_viz/step.pb.h"

namespace cura
{

CuraViz* CuraViz::instance_ = nullptr;
std::mutex CuraViz::mutex_;

CuraViz::CuraViz()
    : socket_(io_context_)
{
    try
    {
        constexpr uint16_t port = 49673;
        socket_.connect(asio::ip::tcp::endpoint(asio::ip::address::from_string("127.0.0.1"), port));
        spdlog::info("Connected to CuraViz");
    }
    catch (asio::system_error error)
    {
        socket_.close();
        spdlog::warn("CuraViz could not connect to vizualiser: {}", error.what());
    }
}

void CuraViz::send(const cura_viz::Step& step)
{
    if (! socket_.is_open())
    {
        return;
    }

    std::string serialized;
    step.SerializeToString(&serialized);

    uint32_t size = serialized.size();

    const std::lock_guard lock(mutex_);
    asio::write(socket_, asio::buffer(&size, sizeof(size)));
    asio::write(socket_, asio::buffer(serialized));
}

CuraViz* CuraViz::getInstance()
{
    const std::lock_guard lock(mutex_);

    if (instance_ == nullptr)
    {
        instance_ = new CuraViz();
    }

    return instance_;
}

void CuraViz::send(const Point2LL& point, const std::string& name, const std::string& step_name)
{
    cura_viz::Step step;
    step.set_name(step_name);

    cura_viz::GeometricElement* geometric_element = step.add_elements();
    geometric_element->set_name(name);

    cura_viz::GeometricData* geometric_data = geometric_element->mutable_data();

    cura_viz::Point2LL* point_message = geometric_data->mutable_point2ll();
    point_message->set_x(point.X);
    point_message->set_y(point.Y);

    getInstance()->send(step);
}

} // namespace cura

#endif