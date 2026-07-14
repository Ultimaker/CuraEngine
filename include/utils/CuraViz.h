// Copyright (c) 2026 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef CURAVIZ_H
#define CURAVIZ_H
#ifdef ENABLE_CURAVIZ

#include <asio.hpp>

#include "geometry/Point2LL.h"

namespace cura_viz
{
class Message;
class GeometricElement;
class Polyline2LL;
} // namespace cura_viz

namespace cura
{

class MixedLinesSet;
class Shape;
class Polyline;

class CuraViz
{
public:
    static void send(const Point2LL& point, const std::string& name = "", const std::string& step_name = "");

    static void send(const Shape& shape, const std::string& name = "", const std::string& step_name = "");

    static void send(const std::vector<Shape>& shapes, const std::string& name = "", const std::string& step_name = "");

    static void send(const MixedLinesSet& lines_set, const std::string& name = "", const std::string& step_name = "");

    static void send(const std::vector<MixedLinesSet>& lines_sets, const std::string& name = "", const std::string& step_name = "");

private:
    /*! Convenience class that stores the actual message and handles its actual sending and destruction when appropriate */
    class MessageToSend
    {
    public:
        explicit MessageToSend(const std::string& step_name);

        virtual ~MessageToSend();

        cura_viz::GeometricElement* addGeometricElement(const std::string& element_name);

    private:
        std::shared_ptr<cura_viz::Message> message_;
    };

    CuraViz();

    void send(const cura_viz::Message& message, const bool should_lock = true);

    static CuraViz* getInstance();

    static void setup(const Shape& shape, cura_viz::GeometricElement* element);

    static void setup(const MixedLinesSet& lines, cura_viz::GeometricElement* element);

    static void setup(const Point2LL& point, cura_viz::GeometricElement* element);

    static void setup(const Polyline& polyline, cura_viz::Polyline2LL* polyline_message);

private:
    static CuraViz* instance_;
    static std::mutex mutex_;

    asio::io_context io_context_;
    asio::ip::tcp::socket socket_;
};

} // namespace cura
#endif
#endif
