// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstddef>
#include <string>

#include "types/Coord_t.h"
#include "types/LayerIndex.h"
#include "types/RaftLayerType.h"

namespace curaengine
{

class SliceDataStorage;

namespace utils::raft
{
/*!
 * \brief Add a raft polygon to the slice data storage.
 * \param storage The storage to store the newly created raft.
 */
void generate(SliceDataStorage& storage);

/*!
 * \brief Get the height difference between the raft and the bottom of
 * layer 0.
 */
types::coord_t getZdiffBetweenRaftAndLayer0();

/*!
 * \brief Get the amount of layers to fill the airgap and initial layer with
 * helper parts (support, prime tower, etc.).
 *
 * The initial layer gets a separate filler layer because we don't want to
 * apply the layer_0_z_overlap to it.
 */
size_t getFillerLayerCount();

/*!
 * \brief Get the layer height of the filler layers in between the raft and
 * layer 1.
 */
types::coord_t getFillerLayerHeight();

/*!
 * \brief Get the total thickness of the raft (without airgap).
 */
types::coord_t getTotalThickness();

/*!
 * \brief Get the total amount of extra layers below zero because there is a
 * raft.
 *
 * This includes the filler layers which are introduced in the air gap.
 */
size_t getTotalExtraLayers();

/*!
 *  \brief Get the amount of layers for the raft base.
 *  \note This is currently hard-coded to 1 because we have yet no setting for the base
 */
size_t getBaseLayers();

/*! \brief Get the amount of layers for the raft interface. */
size_t getInterfaceLayers();

/*! \brief Get the amount of layers for the raft top. */
size_t getSurfaceLayers();

/*!
 * \brief Get the type of layer at the given layer index.
 * \param layer_index The layer index to get the type of.
 * \return The type of layer at the given layer index.
 */
types::RaftLayerType getLayerType(types::LayerIndex layer_index);

/*!
 * \brief Get the amount of layers to be printed for the given raft section
 * \param extruder_nr_setting_name The name of the setting to be fetched to get the proper extruder number
 * \param target_raft_section The name of the setting to be fetched to get the number of layers
 * \return The number of layers for the given raft section, or 0 if raft is disabled
 */
size_t getLayersAmount(const std::string& extruder_nr_setting_name, const std::string& target_raft_section);
} // namespace utils::raft

} // namespace curaengine
