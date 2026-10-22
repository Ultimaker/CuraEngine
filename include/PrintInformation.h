// Copyright (c) 2026 UltiMaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>
#include <vector>

#include "ExtruderPrintInformation.h"
#include "utils/AABB.h"

namespace cura
{

class LayerPlan;

struct PrintInformation
{
    std::vector<std::optional<ExtruderPrintInformation>> extruders_info; // One per extruder, but no value if the extruder is unused
    std::optional<size_t> initial_extruder_nr;
    AABB initial_layer_bb;

    /*!
     * Update the print information with the given processed layer
     * @param layer_plan The layer plan that has been fully processed
     * @note This method should be called once for all processed layer plans, in processing order
     */
    void updateWithLayer(const LayerPlan* layer_plan);
};

} // namespace cura
