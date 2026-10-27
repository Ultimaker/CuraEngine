// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/postprocess_request.h"

namespace cura::plugins
{
postprocess_request::value_type postprocess_request::operator()(const postprocess_request::native_value_type& gcode) const
{
    value_type message{};
    message.set_gcode_word(gcode);
    return message;
}
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
