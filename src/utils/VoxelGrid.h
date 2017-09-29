/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef UTILS_VOXEL_GRID_H
#define UTILS_VOXEL_GRID_H

#include <array>

#include "intpoint.h"
#include "Volumetric.h"


namespace cura
{
/*!
 * TODO
 */
template<typename T>
class VoxelGrid : public Volumetric<T>
{
public:
    virtual ~VoxelGrid(); //!< Virtual destructor

    /*!
     * Get the value associated with a particular region in this volumetric specification.
     */
    virtual const T& getValue(int layer_nr, Point location) const = 0;
protected:
    using GridPoint = Point3;
    using grid_coord_t = coord_t;

    std::vector<T> elements;
    GridPoint min;
    GridPoint max;
    GridPoint size; // max - min + 1

    Point3Matrix coord_to_grid;

    coord_t layer_height;

    GridPoint toGridPoint(Point3 in) const;
    Point3 toLowerCorner(GridPoint in) const;

    T& getElem(GridPoint location);
};

template<typename T>
VoxelGrid<T>::~VoxelGrid()
{
}

template<typename T>
typename VoxelGrid<T>::GridPoint VoxelGrid<T>::toGridPoint(Point3 in) const
{
    return coord_to_grid.apply(in);
}

template<typename T>
Point3 VoxelGrid<T>::toLowerCorner(GridPoint in) const
{
    return coord_to_grid.apply(in);
}



template<typename T>
const T& VoxelGrid<T>::getValue(int layer_nr, Point location) const
{
    coord_t z = layer_nr * layer_height; // TODO: get more accurate value
    Point3 loc(location.X, location.Y, z);
    GridPoint grid_loc = toGridPoint(loc);
    return getElem(grid_loc);
}

template<typename T>
T& VoxelGrid<T>::getElem(GridPoint location)
{
    assert(location.x >= min.x && location.x <= max.x);
    assert(location.y >= min.y && location.y <= max.y);
    assert(location.z >= min.z && location.z <= max.z);
    GridPoint zero_based = location - min;
    grid_coord_t vector_elem = zero_based.z + zero_based.y * size.z + zero_based.x * size.z * size.y;
    assert(vector_elem < elements.size());
    return elements[vector_elem];
}


} // namespace cura


#endif // UTILS_VOXEL_GRID_H
