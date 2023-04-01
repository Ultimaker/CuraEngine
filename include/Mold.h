//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MOLD_H
#define MOLD_H

namespace cura
{

class Slicer;

/*!
 * Class for transforming the outlines of a model into a mold into which the model can be cast.
 */
class Mold
{
public:
    /*!
     * \brief Convert the polygons (and open polylines) of each mold mesh in
     * \p storage in order to make a mold with a cutout in the shape of the
     * outlines.
     *
     * The open polylines of the model are also used to cut out the shape inside
     * the mold.
     *
     * First the new outlines of all meshes on a layer are computed and then all
     * mold meshes are cut out. This prevents molds reaching through each others
     * casting cutout space.
     *
     * \param slicer_list The container for the sliced polygons (and open
     * polylines) of all meshes.
     * \param layer_height The overall layer height used (used to compute an
     * offset from the mold angle).
     */
    static void process(std::vector<Slicer*>& slicer_list);
private:
};

}//namespace cura

#endif//MOLD_H
