// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <string_view>

#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/SlotID.h"

namespace curaengine::plugins
{
struct slot_metadata
{
    plugins::v0::SlotID slot_id;
    std::string_view version;
    std::string_view engine_uuid;
};
} // namespace curaengine::plugins
