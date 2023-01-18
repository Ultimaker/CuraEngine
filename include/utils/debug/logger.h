// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_LOGGER_H
#define CURAENGINE_LOGGER_H

#include <memory>
#include <unordered_map>

#include <boost/serialization/singleton.hpp>

#include "utils/concepts/geometry.h"
#include "utils/debug/factory.h"
#include "utils/debug/visual_logger.h"


namespace cura::debug
{
namespace details
{
class Loggers_
{
public:
    template<typename... Args>
    shared_visual_logger make_logger(std::string_view id, Args&&... args)
    {
        loggers_.insert_or_assign(id, details::make_visual_logger(id, std::forward<Args>(args)...));
        return get(id);
    }

    [[nodiscard]] shared_visual_logger get(std::string_view id)
    {
        return loggers_[id];
    }

    void set(shared_visual_logger&& logger)
    {
        loggers_.insert_or_assign(logger->getId(), logger);
    }

    void set_all(auto&& value)
    {
        auto shared_value = std::make_shared<decltype(value)>(value);
        for (auto& [_, logger] : loggers_)
        {
            logger->setValue(value);
        }
    }

private:
    std::unordered_map<std::string_view, shared_visual_logger> loggers_{};
};
} // namespace details

using Loggers = boost::serialization::singleton<details::Loggers_>;

[[nodiscard]] auto get_logger(std::string_view id)
{
    return Loggers::get_mutable_instance().get(id);
}

template<typename... Args>
auto make_logger(std::string_view id, Args&&... args)
{
    return Loggers::get_mutable_instance().make_logger(id, std::forward<Args>(args)...);
}

auto set_all(auto&& value)
{
    Loggers::get_mutable_instance().set_all(std::forward<decltype(value)>(value));
}

} // namespace cura::debug

#endif // CURAENGINE_LOGGER_H
