// Copyright (c) 2019 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_H
#define BRIDGE_H

#include <optional>
#include <tuple>

#include "utils/Coord_t.h"

namespace cura
{

class Shape;
class SliceMeshStorage;
class SliceDataStorage;
class SupportLayer;
class AngleDegrees;
class LayerPlan;

/*!
 * \brief Computes the angle that lines have to take to bridge a certain shape
 * best.
 *
 * If the area should not be bridged, an angle of -1 is returned.
 * \param mesh The mesh being processed.
 * \param skin_outline The shape to fill with lines.
 * \param storage The slice data storage where to find objects that the bridge
 * could rest on in previous layers.
 * \param layer_nr The layer currently being printed.
 * \param bridge_layer The bridge layer number (1, 2 or 3).
 * \param support_layer Support that the bridge could rest on.
 * \param supported_regions Pre-computed regions that the support layer would
 * support.
 */
std::optional<AngleDegrees> bridgeAngle(
    const SliceMeshStorage& mesh,
    const Shape& skin_outline,
    const SliceDataStorage& storage,
    const unsigned layer_nr,
    const unsigned bridge_layer,
    const SupportLayer* support_layer,
    Shape& supported_regions);

std::tuple<Shape, AngleDegrees> makeBridgeOverInfillPrintable(
    const Shape& infill_contour,
    const Shape& infill_below_skin_area,
    const SliceMeshStorage& mesh,
    const LayerPlan* completed_layer_plan_below,
    const unsigned layer_nr);

} // namespace cura

#endif // BRIDGE_H
