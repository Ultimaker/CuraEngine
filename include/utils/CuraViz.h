// Copyright (c) 2026 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef CURAVIZ_H
#define CURAVIZ_H
#ifdef ENABLE_CURAVIZ

#include <asio.hpp>

#include "geometry/Point2LL.h"

namespace cura_viz
{
class Step;
}

namespace cura
{

class CuraViz
{
public:
    static void send(const Point2LL& point, const std::string& name = "", const std::string& step_name = "");

private:
    CuraViz();

    void send(const cura_viz::Step& step);

    static CuraViz* getInstance();

private:
    static CuraViz* instance_;
    static std::mutex mutex_;

    asio::io_context io_context_;
    asio::ip::tcp::socket socket_;
};

} // namespace cura
#endif
#endif
