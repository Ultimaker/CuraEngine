//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BRIDGE_H
#define BRIDGE_H

namespace cura
{

class Polygons;
class Settings;
class SliceDataStorage;
class SupportLayer;

/*!
 * \brief Computes the angle that lines have to take to bridge a certain shape
 * best.
 *
 * If the area should not be bridged, an angle of -1 is returned.
 * \param settings The settings container to get settings from.
 * \param skin_outline The shape to fill with lines.
 * \param storage The slice data storage where to find objects that the bridge
 * could rest on in previous layers.
 * \param layer_nr The layer currently being printed.
 * \param bridge_layer The bridge layer number (1, 2 or 3).
 * \param support_layer Support that the bridge could rest on.
 * \param supported_regions Pre-computed regions that the support layer would
 * support.
 */
int bridgeAngle(const Settings& settings, const Polygons& skin_outline, const SliceDataStorage& storage, const unsigned layer_nr, const unsigned bridge_layer, const SupportLayer* support_layer, Polygons& supported_regions);

}//namespace cura

#endif//BRIDGE_H
