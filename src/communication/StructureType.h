//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef STRUCTURETYPE_H
#define STRUCTURETYPE_H

namespace cura
{

/*!
 * Types of structures that CuraEngine generates areas for.
 *
 * The values of this enum should match the StructurePolygon::Type structure in
 * Cura.proto.
 */
enum class StructureType: unsigned char
{
    Wall = 0, //Region covered by walls (both inner and outer)
    Skin = 1, //Region covered by skin.
    Infill = 2, //Region covered by infill.
    Support = 3, //Region covered by support.
    Combing = 4, //Collision area for combing.
    Adhesion = 5 //Region covered by brim/skirt/raft/etc.
};

}

#endif //STRUCTURETYPE_H
