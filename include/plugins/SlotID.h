// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "cura/plugins/v0/slot_id.pb.h"

namespace curaengine::plugins::v0
{
using ::cura::plugins::v0::SlotID;
} // namespace curaengine::plugins::v0
#else
#include <cstdint>
#include <limits>

namespace curaengine::plugins::v0
{
// FIXME: use same SlotID as defined in Cura.proto, Maybe revert back to char_range_literal, but that is a lot of work
enum SlotID : int
{
    SETTINGS_BROADCAST = 0,
    SIMPLIFY_MODIFY = 100,
    POSTPROCESS_MODIFY = 101,
    INFILL_MODIFY = 102,
    GCODE_PATHS_MODIFY = 103,
    INFILL_GENERATE = 200,
    DIALECT_GENERATE = 201,
    SlotID_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
    SlotID_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
} // namespace curaengine::plugins::v0
#endif // ENABLE_PLUGINS
