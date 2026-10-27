// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/handshake_request.h"

namespace cura::plugins
{
handshake_request::value_type handshake_request::operator()(const std::string& name, const std::string& version, const handshake_request::native_value_type& slot_info) const
{
    value_type message{};
    message.set_slot_id(slot_info.slot_id);
    message.set_version(slot_info.version.data());
    message.set_plugin_name(name);
    message.set_plugin_version(version);
    return message;
}
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
