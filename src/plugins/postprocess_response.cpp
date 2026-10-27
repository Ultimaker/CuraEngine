// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/postprocess_response.h"

namespace cura::plugins
{
postprocess_response::native_value_type
    postprocess_response::operator()([[maybe_unused]] const postprocess_response::native_value_type& original_value, const postprocess_response::value_type& message) const
{
    return message.gcode_word();
}
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
