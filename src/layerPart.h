//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LAYERPART_H
#define LAYERPART_H

/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons, and makes groups of polygons,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundary of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/

namespace cura
{

class Settings;
class SliceLayer;
class Slicer;
class SlicerLayer;
class SliceMeshStorage;

/*!
 * \brief Split a layer into parts.
 * \param settings The settings to get the settings from (whether to union or
 * not).
 * \param storageLayer Where to store the parts.
 * \param layer The layer to split.
 */
void createLayerWithParts(const Settings& settings, SliceLayer& storageLayer, SlicerLayer* layer);

/*!
 * \brief Split all layers into parts.
 * \param mesh The mesh of which to split the layers into parts.
 * \param slicer The slicer to get the layers from.
 */
void createLayerParts(SliceMeshStorage& mesh, Slicer* slicer);

}//namespace cura

#endif//LAYERPART_H
