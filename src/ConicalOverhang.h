/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef CONICAL_OVERHANG_H
#define CONICAL_OVERHANG_H

#include "slicer.h"


namespace cura {


/*!
 * A class for changing the geometry of a model such that it is printable without support -
 * Or at least with at least support as possible
 */
class ConicalOverhang
{
public:
    /*!
     * Change the slice data such that the model becomes more printable
     *
     * \param[in,out] slicer The slice data
     * \param angle The maximum angle which can be printed without generating support (or at least generating least support)
     * \param layer_thickness The general layer thickness
     */
    static void apply(Slicer* slicer, double angle, int layer_thickness);
};

}//namespace cura

#endif // CONICAL_OVERHANG_H
