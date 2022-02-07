//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INSET_ORDER_OPTIMIZER_H
#define INSET_ORDER_OPTIMIZER_H

#include "PathOrderOptimizer.h"
#include "sliceDataStorage.h" //For SliceMeshStorage, which is used here at implementation in the header.

namespace cura
{

class FffGcodeWriter;
class LayerPlan;

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
    InsetOrderOptimizer(const FffGcodeWriter& gcode_writer,
                        const SliceDataStorage& storage,
                        LayerPlan& gcode_layer,
                        const Settings& settings,
                        const int extruder_nr,
                        const GCodePathConfig& inset_0_non_bridge_config,
                        const GCodePathConfig& inset_X_non_bridge_config,
                        const GCodePathConfig& inset_0_bridge_config,
                        const GCodePathConfig& inset_X_bridge_config,
                        const bool retract_before_outer_wall,
                        const coord_t wall_0_wipe_dist,
                        const coord_t wall_x_wipe_dist,
                        const size_t wall_0_extruder_nr,
                        const size_t wall_x_extruder_nr,
                        const ZSeamConfig& z_seam_config,
                        const VariableWidthPaths& paths);

    /*!
     * Adds the insets to the given layer plan.
     *
     * The insets and the layer plan are passed to the constructor of this
     * class, so this optimize function needs no additional information.
     * \return Whether anything was added to the layer plan.
     */
    bool addToLayer();

private:

    const FffGcodeWriter& gcode_writer;
    const SliceDataStorage& storage;
    LayerPlan& gcode_layer;
    const Settings& settings;
    const size_t extruder_nr;
    const GCodePathConfig& inset_0_non_bridge_config;
    const GCodePathConfig& inset_X_non_bridge_config;
    const GCodePathConfig& inset_0_bridge_config;
    const GCodePathConfig& inset_X_bridge_config;
    const bool retract_before_outer_wall;
    const coord_t wall_0_wipe_dist;
    const coord_t wall_x_wipe_dist;
    const size_t wall_0_extruder_nr;
    const size_t wall_x_extruder_nr;
    const ZSeamConfig& z_seam_config;
    const VariableWidthPaths& paths;
    const unsigned int layer_nr;
    bool added_something;
    bool retraction_region_calculated; //Whether the retraction_region field has been calculated or not.
    std::vector<std::vector<ConstPolygonPointer>> inset_polys; // vector of vectors holding the inset polygons
    Polygons retraction_region; //After printing an outer wall, move into this region so that retractions do not leave visible blobs. Calculated lazily if needed (see retraction_region_calculated).

    /*!
     * Retrieves the region-id of the outer region (belongs to the outer outline, not to a hole).
     */
    static size_t getOuterRegionId(const VariableWidthPaths& toolpaths, size_t& out_max_region_id);

public:
    /*!
     * Converts the VariableWidthPath to a bin of walls, consisting of a vector of paths, consisting of a vector of
     * lines
     * \param toolpaths The toolpaths to convert
     * \param pack_by_inset Pack regions by inset, otherwise, pack insets by region. Useful for outer/inner first situations.
     * \param p_bins_with_index_zero_insets When optimizing, not all inset zero indices are in the zeroth bin. (Can be set to nullptr, which won't negate optimize.)
     * \return A bin of walls, consisting of a vector of paths consisting of vector of lines
     */
    static BinJunctions variableWidthPathToBinJunctions
    (
            const VariableWidthPaths& toolpaths,
            const bool pack_regions_by_inset = true,
            const bool center_last = false,
            std::set<size_t>* p_bins_with_index_zero_insets = nullptr
    );
};

} //namespace cura

#endif // INSET_ORDER_OPTIMIZER_H
