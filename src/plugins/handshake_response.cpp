// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/handshake_response.h"

namespace curaengine::plugins
{
handshake_response::native_value_type handshake_response::operator()(const handshake_response::value_type& message, std::string_view peer) const
{
    return { .slot_version_range = message.slot_version_range(),
             .plugin_name = message.plugin_name(),
             .plugin_version = message.plugin_version(),
             .peer = std::string{ peer },
             .broadcast_subscriptions = std::set<int>(message.broadcast_subscriptions().begin(), message.broadcast_subscriptions().end()) };
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
