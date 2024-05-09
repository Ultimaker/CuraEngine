// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_INTERLEAVED_H
#define PRIME_TOWER_INTERLEAVED_H

#include "PrimeTower/PrimeTower.h"

namespace cura
{

class PrimeTowerInterleaved : public PrimeTower
{
public:
    /*!
     * \brief Creates a prime tower instance that will determine where and how
     * the prime tower gets printed.
     *
     * \param storage A storage where it retrieves the prime tower settings.
     */
    PrimeTowerInterleaved(SliceDataStorage& storage, size_t extruder_count);
};

} // namespace cura

#endif // PRIME_TOWER_INTERLEAVED_H
