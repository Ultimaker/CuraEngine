// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SupportStorage.h"

namespace curaengine
{
SupportStorage::SupportStorage()
    : generated(false)
    , layer_nr_max_filled_layer(-1)
    , cross_fill_provider(nullptr)
{
}

SupportStorage::~SupportStorage()
{
    supportLayers.clear();
}

} // namespace curaengine
