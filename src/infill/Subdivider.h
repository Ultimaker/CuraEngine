/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SUBDIVIDER_H
#define INFILL_SUBDIVIDER_H


#include "SierpinskiFillEdge.h"

namespace cura
{

class Subdivider
{
public:
    virtual bool operator()(const SierpinskiFillEdge& e1, const SierpinskiFillEdge& e2) const = 0;
    virtual ~Subdivider()
    {
    };
};

} // namespace cura


#endif // INFILL_SUBDIVIDER_H
