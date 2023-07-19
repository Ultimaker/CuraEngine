// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef RAFT_H
#define RAFT_H

#include "utils/Coord_t.h"

namespace cura
{

class SliceDataStorage;

class Raft
{
public:
    /*!
     * \brief Add a raft polygon to the slice data storage.
     * \param storage The storage to store the newly created raft.
     */
    static void generate(SliceDataStorage& storage);

    /*!
     * \brief Get the height difference between the raft and the bottom of
     * layer 0.
     */
    static coord_t getZdiffBetweenRaftAndLayer0();

    /*!
     * \brief Get the amount of layers to fill the airgap and initial layer with
     * helper parts (support, prime tower, etc.).
     *
     * The initial layer gets a separate filler layer because we don't want to
     * apply the layer_0_z_overlap to it.
     */
    static size_t getFillerLayerCount();

    /*!
     * \brief Get the layer height of the filler layers in between the raft and
     * layer 1.
     */
    static coord_t getFillerLayerHeight();

    /*!
     * \brief Get the total thickness of the raft (without airgap).
     */
    static coord_t getTotalThickness();

    /*!
     * \brief Get the total amount of extra layers below zero because there is a
     * raft.
     *
     * This includes the filler layers which are introduced in the air gap.
     */
    static size_t getTotalExtraLayers();
};

} // namespace cura

#endif // RAFT_H
