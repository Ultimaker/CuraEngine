/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef RAFT_H
#define RAFT_H

#include "sliceDataStorage.h"

namespace cura {

class Raft
{
public:
    static void generate(SliceDataStorage& storage, int distance);

    /*!
     * Get the height difference between the raft and the bottom of layer 1.
     *
     * This is used for the filler layers because they don't use the layer_0_z_overlap
     */
    static int getZdiffBetweenRaftAndLayer1(const SliceDataStorage& storage);

    /*!
     * Get the amount of layers to fill the airgap and initial layer with helper parts (support, prime tower, etc.)
     *
     * The initial layer gets a separate filler layer because we don't want to apply the layer_0_z_overlap to it.
     */
    static int getFillerLayerCount(const SliceDataStorage& storage);

    /*!
     * Get the layer height of the filler layers in between the raft and layer 1
     */
    static int getFillerLayerHeight(const SliceDataStorage& storage);

    /*!
     * Get the total thickness of the raft (without airgap)
     */
    static int getTotalThickness(const SliceDataStorage& storage);

    /*!
     * Get the total amount of extra layers below zero because there is a raft.
     *
     * This includes the filler layers which are introduced in the air gap.
     */
    static int getTotalExtraLayers(const SliceDataStorage& storage);

};

}//namespace cura

#endif//RAFT_H
