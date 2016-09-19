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
     * Get the amount of layers to fill the airgap with helper parts (support, prime tower, etc.)
     */
    static int getFillerLayerCount(const SliceDataStorage& storage);

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
