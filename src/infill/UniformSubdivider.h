/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_UNIFORM_SUBDIVIDER_H
#define INFILL_UNIFORM_SUBDIVIDER_H

#include "../utils/intpoint.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "Subdivider.h"
#include "SierpinskiFillEdge.h"

namespace cura
{

class UniformSubdivider : public Subdivider
{
public:
    UniformSubdivider()
    {
    };

    virtual ~UniformSubdivider()
    {
    };

    virtual bool operator()(const SierpinskiFillEdge&, const SierpinskiFillEdge&) const
    {
        return true;
    };
};

} // namespace cura


#endif // INFILL_UNIFORM_SUBDIVIDER_H
