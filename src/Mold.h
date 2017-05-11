/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef MOLD_H
#define MOLD_H

#include "slicer.h"
#include "utils/intpoint.h"

namespace cura
{

class SliceDataStorage;

/*!
 * Class for transforming the outlines of a model into a mold into which the model can be cast.
 */
class Mold
{
public:
    /*!
     * Convert the polygons (and open polylines) of each mold mesh in \p storage
     * in order to make a mold with a cutout in the shape of the outlines.
     * 
     * The open polylines of the model are also used to cut out the shape inside the mold.
     * 
     * First the new outlines of all meshes on a layer are computed and then all mold meshes are cut out.
     * This prevents molds reaching through each others casting cutout space.
     * 
     * \param storage Where to get mesh settings from
     * \param slicer_list The container for the sliced polygons (and open polylines) of all meshes
     * \param layer_height The overall layer height used (used to compute an offset from the mold angle)
     */
    static void process(const SliceDataStorage& storage, std::vector<Slicer*>& slicer_list, coord_t layer_height);
private:
};

}//namespace cura

#endif//MOLD_H
