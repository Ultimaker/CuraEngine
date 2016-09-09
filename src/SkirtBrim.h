/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIRT_BRIM_H
#define SKIRT_BRIM_H

#include "sliceDataStorage.h"

namespace cura 
{

/*!
 * Generate skirt or brim (depending on parameters).
 * 
 * When \p distance > 0 and \p count == 1 a skirt is generated, which has
 * slightly different configuration. Otherwise, a brim is generated.
 * 
 * \param storage Storage containing the parts at the first layer.
 * \param distance The distance of the first outset from the parts at the first
 * layer.
 * \param primary_line_count Number of outsets / brim lines of the primary extruder.
 * \param minLength The minimum length the skirt should have (enforced by taking
 * more outsets).
 * \param outside_only Whether to only generate a brim on the outside, rather than also in holes
 */
void generateSkirtBrim(SliceDataStorage& storage, int distance, unsigned int primary_line_count, int minLength, bool outside_only);

}//namespace cura

#endif //SKIRT_BRIM_H
