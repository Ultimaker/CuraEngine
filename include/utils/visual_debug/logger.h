// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VISUAL_DEBUG_LOGGER_H
#define INCLUDE_UTILS_VISUAL_DEBUG_LOGGER_H

#include <memory>
#include <mutex>

#include <boost/serialization/singleton.hpp>

#include "utils/visual_debug/visual_logger.h"

namespace cura::debug
{
namespace details
{
class LoggersImpl
{
public:
    /**
     * Get the visual Logger
     * @param id
     * @return
     */
    [[nodiscard]] shared_visual_logger_t Get(const std::string& id)
    {
        return loggers_[id];
    };

    /**
     * Create a new visual Logger
     * @tparam Args
     * @param id
     * @param args
     * @return
     */
    template<typename... Args>
    shared_visual_logger_t MakeLogger(const std::string& id, Args&& ... args)
    {
        if ( Enabled())
        {
            if ( !loggers_.contains( id ))
            {
                loggers_.emplace( id, std::make_shared<VisualLogger>( id, loggers_.size(), VisualDebugPath(), std::forward<Args>( args )... ));
                Get( id )->setValue( layer_map_ );
            }
        }
        else
        {
            loggers_.insert_or_assign( id, std::make_shared<VisualLogger>());
        }
        return Get( id );
    };

    /**
     * Get the visual logger or lazily create a new Logger if it didn't exist yet
     * @tparam Args
     * @param id
     * @param args
     * @return
     */
    template<typename... Args>
    shared_visual_logger_t Logger(const std::string& id, Args&& ... args)
    {
        const std::scoped_lock lock { mutex_ };
        if ( loggers_.contains( id ))
        {
            return Get( id );
        }
        return MakeLogger( id, std::forward<Args>( args )... );
    }

    void setAll(layer_map_t&& layer_map)
    {
        const std::scoped_lock lock { mutex_ };
        layer_map_ = std::make_shared<layer_map_t>( std::forward<layer_map_t>( layer_map ));
        for ( auto& [ _, logger ] : loggers_ )
        {
            logger->setValue( layer_map_ );
        }
    }

private:
    [[nodiscard]] bool Enabled();

    [[nodiscard]] std::filesystem::path VisualDebugPath();

    std::mutex mutex_;
    std::string now_ { };
    std::unordered_map<std::string, shared_visual_logger_t> loggers_ { };
    shared_layer_map_t layer_map_ { };
};
} // namespace details

using Loggers = boost::serialization::singleton<details::LoggersImpl>; //<! Visual Logger registry

} // namespace cura::debug


#endif // INCLUDE_UTILS_VISUAL_DEBUG_LOGGER_H
