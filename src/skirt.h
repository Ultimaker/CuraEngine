/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIRT_H
#define SKIRT_H

#include "sliceDataStorage.h"

namespace cura 
{

/*!
 * Generate skirt or brim (depending on parameters); when \p distance > 0 and
 * \p count == 1 the skirt is generated, which has slightly different
 * configuration.
 * 
 * \param storage Storage containing the parts at the first layer.
 * \param distance The distance of the first outset from the parts at the first
 * layer.
 * \param count Number of outsets / brim lines.
 * \param minLength The minimum length the skirt should have (enforced by taking
 * more outsets).
 */
void generateSkirtBrim(SliceDataStorage& storage, int distance, int count, int minLength);

}//namespace cura

#endif//SKIRT_H
