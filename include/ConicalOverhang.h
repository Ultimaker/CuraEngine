//Copyright (c) 2016 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef CONICAL_OVERHANG_H
#define CONICAL_OVERHANG_H

namespace cura {

class Mesh;
class Slicer;

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
     * \param[in,out] slicer The slice data.
     * \param mesh The mesh to get the settings from.
     */
    static void apply(Slicer* slicer, const Mesh& mesh);
};

}//namespace cura

#endif // CONICAL_OVERHANG_H
