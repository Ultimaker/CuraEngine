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

#include <boost/asio/write.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/map.hpp>

#include "Application.h"
#include "Slice.h"
#include "geometry/MixedLinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
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
        socket_.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), port));
        spdlog::info("Connected to CuraViz");
    }
    catch (boost::system::system_error error)
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

    boost::asio::write(socket_, boost::asio::buffer(&size, sizeof(size)));
    boost::asio::write(socket_, boost::asio::buffer(serialized));
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
        setup((*iterator).start, (*iterator).end, polyline_message->add_segments());
    }
}

void CuraViz::setup(const ExtrusionLine& line, cura_viz::Polyline2LL* polyline_message)
{
    polyline_message->set_surface(false);

    if (line.junctions_.empty())
    {
        return;
    }

    Point2LL p0 = line.junctions_[0].p_;
    for (const ExtrusionJunction& junction : line.junctions_ | ranges::views::drop(1))
    {
        const Point2LL& p1 = junction.p_;
        setup(p0, p1, polyline_message->add_segments());
        p0 = p1;
    }
}

void CuraViz::setup(const Point2LL& start, const Point2LL& end, cura_viz::Segment2LL* segment_message)
{
    segment_message->mutable_start()->set_x(start.X);
    segment_message->mutable_start()->set_y(start.Y);
    segment_message->mutable_end()->set_x(end.X);
    segment_message->mutable_end()->set_y(end.Y);
}

void CuraViz::send(const Point2LL& point, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    setup(point, message.addGeometricElement(name));
}

void CuraViz::send(const Point2LL& start, const Point2LL& end, const std::string& name, const std::string& step_name)
{
    const OpenPolyline segment({ start, end });
    send(segment, name, step_name);
}

void CuraViz::send(const Point3LL& start, const Point3LL& end, const std::string& name, const std::string& step_name)
{
    send(start.toPoint2LL(), end.toPoint2LL(), name, step_name);
}

void CuraViz::send(const Polyline& line, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    setup(line, message.addGeometricElement(name)->mutable_data()->mutable_lines_set2ll()->add_lines());
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

void CuraViz::send(const OpenLinesSet& lines, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    for (const auto& [line_index, line] : lines.getLines() | ranges::views::enumerate)
    {
        setup(line, message.addGeometricElement(fmt::format("{}_{}", name, line_index))->mutable_data()->mutable_lines_set2ll()->add_lines());
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

void CuraViz::send(const std::vector<VariableWidthLines>& lines, const std::string& name, const std::string& step_name)
{
    MessageToSend message(step_name);
    for (const auto& [index, lines_set] : lines | ranges::views::enumerate)
    {
        cura_viz::GeometricElement* geometric_element = message.addGeometricElement(fmt::format("{}_{}", name, index));
        for (const ExtrusionLine& line : lines_set)
        {
            setup(line, geometric_element->mutable_data()->mutable_lines_set2ll()->add_lines());
        }
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