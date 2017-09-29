/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef UTILS_VOLUMETRIC_H
#define UTILS_VOLUMETRIC_H

#include "intpoint.h"

namespace cura
{
/*!
 * A volumetric specification of a property which can vary throughout the volume of a mesh.
 */
template<typename T>
class Volumetric
{
public:
    virtual ~Volumetric() //!< Virtual destructor
    {};

    /*!
     * Get the value associated with a particular region in this volumetric specification.
     */
    virtual const T& getValue(int layer_nr, Point location) const = 0;
};
} // namespace cura


#endif // UTILS_VOLUMETRIC_H
