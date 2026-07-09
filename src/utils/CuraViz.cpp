// Copyright (c) 2026 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/CuraViz.h"

#include <spdlog/spdlog.h>
#ifdef ENABLE_CURAVIZ

#include <cura_viz/message.pb.h>
#include <cura_viz/point2ll.pb.h>
#include <cura_viz/polyline2ll.pb.h>
#include <cura_viz/printer.pb.h>
#include <cura_viz/step.pb.h>

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/map.hpp>

#include "Application.h"
#include "Slice.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "settings/Settings.h"

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

    if (! socket_.is_open())
    {
        return;
    }

    const Settings& global_settings = Application::getInstance().current_slice_->scene.settings;
    Point2LL machine_max(global_settings.get<coord_t>("machine_width"), global_settings.get<coord_t>("machine_depth"));
    Point2LL machine_min(0, 0);
    if (global_settings.get<bool>("machine_center_is_zero"))
    {
        machine_max = machine_max / 2;
        machine_min -= machine_max;
    }

    cura_viz::Message message;
    cura_viz::Printer* printer_def = message.mutable_printer_def();
    printer_def->mutable_bed_min()->set_x(machine_min.X);
    printer_def->mutable_bed_min()->set_y(machine_min.Y);
    printer_def->mutable_bed_max()->set_x(machine_max.X);
    printer_def->mutable_bed_max()->set_y(machine_max.Y);

    send(message, false);
}

void CuraViz::send(const cura_viz::Message& message, const bool should_lock)
{
    if (! socket_.is_open())
    {
        return;
    }

    std::string serialized;
    message.SerializeToString(&serialized);

    uint32_t size = serialized.size();

    std::unique_lock<std::mutex> lock;
    if (should_lock)
    {
        lock = std::unique_lock(mutex_);
    }

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
    cura_viz::Message message;

    cura_viz::Step* step = message.mutable_step();
    step->set_name(step_name);

    cura_viz::GeometricElement* geometric_element = step->add_elements();
    geometric_element->set_name(name);

    cura_viz::GeometricData* geometric_data = geometric_element->mutable_data();

    cura_viz::Point2LL* point_message = geometric_data->mutable_point2ll();
    point_message->set_x(point.X);
    point_message->set_y(point.Y);

    getInstance()->send(message);
}

void CuraViz::send(const Shape& shape, const std::string& name, const std::string& step_name)
{
    cura_viz::Message message;

    cura_viz::Step* step = message.mutable_step();
    step->set_name(step_name);

    cura_viz::GeometricElement* geometric_element = step->add_elements();
    geometric_element->set_name(name);

    cura_viz::GeometricData* geometric_data = geometric_element->mutable_data();

    cura_viz::LinesSet2LL* lines_set_message = geometric_data->mutable_lines_set2ll();

    for (const Polygon& polygon : shape)
    {
        cura_viz::Polyline2LL* polygon_message = lines_set_message->add_lines();
        polygon_message->set_surface(true);

        for (auto iterator = polygon.beginSegments(); iterator != polygon.endSegments(); ++iterator)
        {
            cura_viz::Segment2LL* segment_message = polygon_message->add_segments();
            segment_message->mutable_start()->set_x((*iterator).start.X);
            segment_message->mutable_start()->set_y((*iterator).start.Y);
            segment_message->mutable_end()->set_x((*iterator).end.X);
            segment_message->mutable_end()->set_y((*iterator).end.Y);
        }
    }

    getInstance()->send(message);
}

void CuraViz::send(const std::vector<Shape>& shapes, const std::string& name, const std::string& step_name)
{
    cura_viz::Message message;

    cura_viz::Step* step = message.mutable_step();
    step->set_name(step_name);

    for (const auto& [shape_index, shape] : shapes | ranges::views::enumerate)
    {
        cura_viz::GeometricElement* geometric_element = step->add_elements();
        geometric_element->set_name(fmt::format("{}_{}", name, shape_index));

        cura_viz::GeometricData* geometric_data = geometric_element->mutable_data();

        cura_viz::LinesSet2LL* lines_set_message = geometric_data->mutable_lines_set2ll();

        for (const Polygon& polygon : shape)
        {
            cura_viz::Polyline2LL* polygon_message = lines_set_message->add_lines();
            polygon_message->set_surface(true);

            for (auto iterator = polygon.beginSegments(); iterator != polygon.endSegments(); ++iterator)
            {
                cura_viz::Segment2LL* segment_message = polygon_message->add_segments();
                segment_message->mutable_start()->set_x((*iterator).start.X);
                segment_message->mutable_start()->set_y((*iterator).start.Y);
                segment_message->mutable_end()->set_x((*iterator).end.X);
                segment_message->mutable_end()->set_y((*iterator).end.Y);
            }
        }
    }

    getInstance()->send(message);
}

} // namespace cura

#endif