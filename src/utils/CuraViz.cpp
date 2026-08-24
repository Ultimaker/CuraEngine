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
#include "geometry/MixedLinesSet.h"
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

    const std::shared_ptr<Slice> current_slice = Application::getInstance().current_slice_;
    if (! current_slice)
    {
        spdlog::warn("CuraViz could not send build plate dimensions since slice has not been initialized yet");
        return;
    }

    const Settings& global_settings = current_slice->scene.settings;
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

void CuraViz::setup(const Shape& shape, cura_viz::GeometricElement* element)
{
    cura_viz::LinesSet2LL* lines_set_message = element->mutable_data()->mutable_lines_set2ll();
    for (const Polygon& polygon : shape)
    {
        setup(polygon, lines_set_message->add_lines());
    }
}

void CuraViz::setup(const MixedLinesSet& lines, cura_viz::GeometricElement* element)
{
    cura_viz::LinesSet2LL* lines_set_message = element->mutable_data()->mutable_lines_set2ll();
    for (const PolylinePtr& line : lines)
    {
        setup(*line, lines_set_message->add_lines());
    }
}

void CuraViz::setup(const Point2LL& point, cura_viz::GeometricElement* element)
{
    cura_viz::Point2LL* point_message = element->mutable_data()->mutable_point2ll();
    point_message->set_x(point.X);
    point_message->set_y(point.Y);
}

void CuraViz::setup(const Polyline& polyline, cura_viz::Polyline2LL* polyline_message)
{
    polyline_message->set_surface(dynamic_cast<const Polygon*>(&polyline) != nullptr);

    for (auto iterator = polyline.beginSegments(); iterator != polyline.endSegments(); ++iterator)
    {
        cura_viz::Segment2LL* segment_message = polyline_message->add_segments();
        segment_message->mutable_start()->set_x((*iterator).start.X);
        segment_message->mutable_start()->set_y((*iterator).start.Y);
        segment_message->mutable_end()->set_x((*iterator).end.X);
        segment_message->mutable_end()->set_y((*iterator).end.Y);
    }
}

void CuraViz::send(const Point2LL& point, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    setup(point, message.addGeometricElement(name));
}

void CuraViz::send(const Shape& shape, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    setup(shape, message.addGeometricElement(name));
}

void CuraViz::send(const std::vector<Shape>& shapes, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    for (const auto& [shape_index, shape] : shapes | ranges::views::enumerate)
    {
        setup(shape, message.addGeometricElement(fmt::format("{}_{}", name, shape_index)));
    }
}

void CuraViz::send(const MixedLinesSet& lines_set, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    setup(lines_set, message.addGeometricElement(name));
}

void CuraViz::send(const std::vector<MixedLinesSet>& lines_sets, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    for (const auto& [lines_set_index, lines_set] : lines_sets | ranges::views::enumerate)
    {
        setup(lines_set, message.addGeometricElement(fmt::format("{}_{}", name, lines_set_index)));
    }
}

CuraViz::MessageToSend::MessageToSend(const std::string& step_name)
    : message_(std::make_shared<cura_viz::Message>())
{
    message_->mutable_step()->set_name(step_name);
}

CuraViz::MessageToSend::~MessageToSend()
{
    CuraViz::getInstance()->send(*message_);
}

cura_viz::GeometricElement* CuraViz::MessageToSend::addGeometricElement(const std::string& element_name)
{
    cura_viz::GeometricElement* element = message_->mutable_step()->add_elements();
    element->set_name(element_name);
    return element;
}

} // namespace cura

#endif