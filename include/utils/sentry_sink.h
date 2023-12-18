// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_UTILS_SENTRY_SINK_H
#define CURAENGINE_INCLUDE_UTILS_SENTRY_SINK_H

#include <sentry.h>

#include <fmt/format.h>
#include <spdlog/sinks/base_sink.h>

template<typename Mutex>
class SentryBreadcrumbSink : public spdlog::sinks::base_sink<Mutex>
{
protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        sentry_value_t crumb = sentry_value_new_breadcrumb("debug", msg.payload.data());
        sentry_value_set_by_key(crumb, "category", sentry_value_new_string(msg.logger_name.data()));
        sentry_value_set_by_key(crumb, "level", sentry_value_new_string(spdlog::level::to_string_view(msg.level).data()));

        auto duration = msg.time.time_since_epoch();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        sentry_value_set_by_key(crumb, "timestamp", sentry_value_new_int32(timestamp));

        sentry_add_breadcrumb(crumb);
    }
    void flush_() override
    {
        // The sink doesn't buffer or cache anything, so there's nothing to flush
    }
};

// Convenience typedefs
using SentryBreadcrumbSink_mt = SentryBreadcrumbSink<std::mutex>;
using SentryBreadcrumbSink_st = SentryBreadcrumbSink<spdlog::details::null_mutex>;

#endif // CURAENGINE_INCLUDE_UTILS_SENTRY_SINK_H
