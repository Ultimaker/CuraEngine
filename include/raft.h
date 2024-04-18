// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef RAFT_H
#define RAFT_H

#include "settings/types/LayerIndex.h"
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

    /*!
     *  \brief Get the amount of bottom for the raft interface.
     *  \note This is currently hard-coded to 1 because we have yet no setting for the bottom
     */
    static size_t getBottomLayers();

    /*! \brief Get the amount of layers for the raft interface. */
    static size_t getInterfaceLayers();

    /*! \brief Get the amount of layers for the raft top. */
    static size_t getSurfaceLayers();

    enum LayerType
    {
        RaftBase,
        RaftInterface,
        RaftSurface,
        Airgap,
        Model
    };

    /*!
     * \brief Get the type of layer at the given layer index.
     * \param layer_index The layer index to get the type of.
     * \return The type of layer at the given layer index.
     */
    static LayerType getLayerType(LayerIndex layer_index);

private:
    static size_t getLayersAmount(const std::string& extruder_nr, const std::string& layers_nr);
};

} // namespace cura

#endif // RAFT_H
