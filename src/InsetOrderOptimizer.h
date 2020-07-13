//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INSET_ORDER_OPTIMIZER_H
#define INSET_ORDER_OPTIMIZER_H

#include "PathOrderOptimizer.h"
#include "sliceDataStorage.h" //For SliceMeshStorage, which is used here at implementation in the header.

namespace cura
{

class FffGcodeWriter;
class LayerPlan;
class WallOverlapComputation;

class InsetOrderOptimizer
{
public:
    /*!
     * Constructor for inset ordering optimizer.
     *
     * This constructor gets basically all of the locals passed when it needs to
     * optimise the order of insets.
     * \param gcode_writer The FffGcodeWriter on whose behalf the inset order is
     * being optimized.
     * \param storage Read slice data from this storage.
     * \param gcode_layer The layer where the resulting insets must be planned.
     * \param mesh The mesh that these insets are part of.
     * \param extruder_nr Which extruder to process. If an inset is not printed
     * with this extruder, it will not be added to the plan.
     * \param mesh_config The path configs for a single mesh, indicating the
     * line widths, flows, speeds, etc to print this mesh with.
     * \param part The part from which to read the previously generated insets.
     * \param layer_nr The current layer number.
     */
    InsetOrderOptimizer(const FffGcodeWriter& gcode_writer, const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, unsigned int layer_nr);

    /*!
     * Adds the insets to the given layer plan.
     *
     * The insets and the layer plan are passed to the constructor of this
     * class, so this optimize function needs no additional information.
     * \return Whether anything was added to the layer plan.
     */
    bool optimize();

private:

    const FffGcodeWriter& gcode_writer;
    const SliceDataStorage& storage;
    LayerPlan& gcode_layer;
    const SliceMeshStorage& mesh;
    const size_t extruder_nr;
    const PathConfigStorage::MeshPathConfigs& mesh_config;
    const SliceLayerPart& part;
    const unsigned int layer_nr;
    const ZSeamConfig z_seam_config;
    bool added_something;
    bool retraction_region_calculated; //Whether the retraction_region field has been calculated or not.
    WallOverlapComputation* wall_overlapper_0;
    WallOverlapComputation* wall_overlapper_x;
    std::vector<std::vector<ConstPolygonPointer>> inset_polys; // vector of vectors holding the inset polygons
    Polygons retraction_region; //After printing an outer wall, move into this region so that retractions do not leave visible blobs. Calculated lazily if needed (see retraction_region_calculated).

    /*!
     * Print the insets in an order based on their inset index.
     *
     * For instance, it will first print all insets with index 0, then all
     * insets with index 1, and so on. Which index to start from depends on the
     * ``outer_inset_first`` setting.
     * Within the set of walls with the same index, the walls are ordered to
     * minimize travel distance.
     * \return Whether this added anything to the layer plan or not.
     */
    bool processInsetsIndexedOrdering();

    /*!
     * Generate the insets for the holes of a given layer part after optimizing the ordering.
     */
    void processHoleInsets();

    /*!
     * Generate the insets for the outer walls of a given layer part after optimizing the ordering.
     * \param include_outer true if the outermost inset is to be output
     * \param include_inners true if the innermost insets are to be output
     */
    void processOuterWallInsets(const bool include_outer, const bool include_inners);

    /*!
     * Generate a travel move from the current position to inside the part.
     * This is used after generating an outer wall so that if a retraction occurs immediately afterwards,
     * the extruder won't be on the outer wall.
     */
    void moveInside();

public:
    /*!
     * Generate the insets for all of the walls of a given layer part after optimizing the ordering.
     * \return Whether this function added anything to the layer plan
     */
    bool processInsetsWithOptimizedOrdering();

    /*!
     * Test whether it looks to be worthwhile to optimize the inset order of a given layer part.
     * \param mesh The mesh to be added to the layer plan.
     * \param part The part for which to create gcode
     * \return true if it is worth optimizing the inset order, false if not
     */
    static bool optimizingInsetsIsWorthwhile(const SliceMeshStorage& mesh, const SliceLayerPart& part);
};

} //namespace cura

#endif // INSET_ORDER_OPTIMIZER_H
