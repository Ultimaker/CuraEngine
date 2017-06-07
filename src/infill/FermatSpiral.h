/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#ifndef CURA_INFILL_FREMATSPIRAL_H
#define CURA_INFILL_FREMATSPIRAL_H

#include <cstdint>

#include "../../libs/clipper/clipper.hpp"
#include "../utils/polygon.h"
#include "../sliceDataStorage.h"

/*
 * This Fermat Spiral Infill code is implemented according to the following articles:
 *  - "Connected Fermat Spirals for Layered Fabrication" by H. Zhao et al., SIGGRAPH '16
 */
namespace cura {

/*
 * This class helps to generate a fermat spiral infill for the given polygon objects.
 */
class FermatSpiralInfillGenerator {
public:
    void generateInfill(const Polygons& in_outline, Polygons& result_lines, const SliceMeshStorage* mesh);
};

} // namespace cura

#endif // CURA_FREMATSPIRAL_INFILL_H
