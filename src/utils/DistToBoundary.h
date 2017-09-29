/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef UTILS_DIST_TO_BOUNDARY_H
#define UTILS_DIST_TO_BOUNDARY_H

#include "Volumetric.h"

#include "../sliceDataStorage.h"

namespace cura
{
/*!
 * A volumetric specification of distance to the boundary of the mesh as a property which varies throughout the volume of the mesh.
 */
class DistToBoundary : public Volumetric<coord_t>
{
public:
    /*!
     * Virtual destructor
     */
    ~DistToBoundary();

    DistToBoundary(const SliceDataStorage& storage);

    /*!
     * Get the distance of a sphere described by \p radius to the boundary of the model.
     * 
     * A distance of zero is returned if the sphere intersects with the boundary.
     */
    const coord_t& getValue(int layer_nr, Point location) const;
protected:
    const SliceDataStorage& storage;
    const coord_t layer_height; //!< If you don't know what layer_height is then close this file.
};

} // namespace cura


#endif // UTILS_DIST_TO_BOUNDARY_H
