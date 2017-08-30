//Copyright (C) 2013 Ultimaker
//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SUPPORT_H
#define SUPPORT_H

#include "sliceDataStorage.h"
#include "MeshGroup.h"
#include "commandSocket.h"
#include "slicer.h"

namespace cura {

class AreaSupport {
public:
    /*!
     * Move support mesh outlines from slicer data into the support storage
     * 
     * \param[out] storage Where to store the support areas
     * \param mesh Where to get the settings from what kind of support mesh it is.
     * \param slicer Where to get the outlines from
     * \return Whether the mesh is used up in support and no normal mesh processing is needed
     */
    static bool handleSupportModifierMesh(SliceDataStorage& storage, const SettingsBaseVirtual& mesh, const Slicer* slicer);

    /*!
     * Generate the support areas and support skin areas for all models.
     * \param storage data storage containing the input layer outline data and containing the output support storage per layer
     * \param layer_count total number of layers
     */
    static void generateSupportAreas(SliceDataStorage& storage, unsigned int layer_count);

    /*!
     * Generates all gradual support infill features.
     * It does the following:
     *  - initialize insets and infill areas for all support infill parts
     *  - generated support infill areas with different density levels
     *  - combine multiple support infill areas layers into single layers
     *
     * \param storage data storage containing the input layer outline data and containing the output support storage per layer
     */
    static void generateSupportInfillFeatures(SliceDataStorage& storage);

    /*!
     * Generate the insets of the given support infill outline.
     *
     * \param[out] insets The insets result to output.
     * \param outline The given support infill outline.
     * \param inset_count The number of perimeters to surround the support infill outline.
     * \param wall_line_width_x The wall line width in microns on the X axis.
     */
    static void generateOutlineInsets(std::vector<Polygons>& insets, Polygons& outline, int inset_count, int wall_line_width_x);

private:
    /*!
     * Splits the global support areas into separete SupportInfillParts.
     * This is required before generating the gradual support infill.
     * \param storage data storage containing the input layer outline data and containing the output support storage per layer
     * \param global_support_areas_per_layer the global support areas per layer
     * \param total_layer_count total number of layers
     */
    static void splitGlobalSupportAreasIntoSupportInfillParts(SliceDataStorage& storage, const std::vector<Polygons>& global_support_areas_per_layer, unsigned int total_layer_count);

    /*!
     * Generate insets and infill areas for all support infill parts.
     * \param storage data storage containing the input layer outline data and containing the output support storage per layer
     */
    static void prepareInsetsAndInfillAreasForForSupportInfillParts(SliceDataStorage& storage);

    /*!
     * Generate gradual support on the already generated support areas. This must be called after generateSupportAreas().
     * This uses the same technic as the gradual infill.
     *
     * This densities of the infill areas are determined by comparing the **outlines** of each support infill part.
     * Take the following as an example:
     *
     *       comparing infill        comparing with outline (this is our approach)
     *           ^^^^^^               ^^^^^^
     *           ####||^^             ####||^^
     *           ######||^^           ######||^^
     *           ++++####||           ++++++##||
     *           ++++++####           ++++++++##
     *
     * LEGEND:
     *  ^ support roof
     *   | support wall
     *   # dense support
     *   + less dense support
     *
     * In this example, comparing with outlines makes sure that the walls will always be printed upon the most dense area and at the same time
     * we don't generate unnecessary dense infill areas. This saves print time and material and also insures that the walls can be safely printed.
     *
     * \param storage data storage containing the input layer outline data and containing the output support storage per layer
     */
    static void generateGradualSupport(SliceDataStorage& storage);

    /*!
     * \brief Combines the support infill of multiple layers.
     * 
     * The support infill layers are combined while the thickness of each layer is
     * multiplied such that the infill should fill up again to the full height of
     * all combined layers.
     * 
     * \param storage data storage containing the input layer outline data and containing the output support storage per layer
     */
    static void combineSupportInfillLayers(SliceDataStorage& storage);

    /*!
     * Generate support polygons over all layers for one object.
     * 
     * This function also handles small overhang areas (creates towers with larger diameter than just the overhang area) and single walls which could otherwise fall over.
     * 
     * The anti_overhang areas are taken into account.
     * 
     * \warning This function should be called only once for handling support meshes with drop down and
     * once for all support meshes without drop down.
     * The \p mesh_idx should then correspond to an empty \ref SliceMeshStorage of one support mesh with the given value of support_mesh_drop_down.
     * 
     * \param storage data storage containing the input layer outline data
     * \param infill_settings The settings base to get the settings from which are based on the infill of the support
     * \param roof_settings The settings base to get the settings from which are based on the top interface of the support
     * \param bottom_settings The settings base to get the settings from which are based on the bottom interface of the support
     * \param mesh_idx The index of the object for which to generate support areas
     * \param layer_count total number of layers
     */
    static void generateSupportAreasForMesh(SliceDataStorage& storage, const SettingsBaseVirtual& infill_settings, const SettingsBaseVirtual& roof_settings, const SettingsBaseVirtual& bottom_settings, unsigned int mesh_idx, unsigned int layer_count, std::vector<Polygons>& supportAreas);

    /*!
     * Generate support bottom areas for a given mesh.
     *
     * The bottom areas are separated from the normal support areas in the slice
     * data storage, and stored separately in a different field of the slice
     * data storage.
     *
     * \param storage Where to find the previously generated support areas and
     * where to output the new support bottom areas.
     * \param mesh The mesh to generate support for.
     * \param global_support_areas_per_layer the global support areas on each layer.
     */
    static void generateSupportBottom(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer);

    /*!
     * Generate support roof areas for a given mesh.
     *
     * The roof areas are separated from the normal support areas in the slice
     * data storage, and stored separately in a different field of the slice
     * data storage.
     *
     * \param storage Where to find the previously generated support areas and
     * where to output the new support roof areas.
     * \param mesh The mesh to generate support roof for.
     * \param global_support_areas_per_layer the global support areas on each layer.
     */
    static void generateSupportRoof(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer);

    /*!
     * \brief Generate a single layer of support interface.
     *
     * The interface polygons are going to get filled with the actual support
     * interface polygons, and this is subtracted from the support areas.
     *
     * \param support_areas The areas where support infill is going to be
     * printed.
     * \param mesh_outlines The outlines of the mesh above or below the layer
     * we're generating interface for. These layers determine what areas are
     * going to be filled with the interface.
     * \param safety_offset An offset applied to the result to make sure
     * everything can be printed.
     * \param[out] interface_polygons The resulting interface layer. Do not use `interface` in windows!
     */
    static void generateSupportInterfaceLayer(Polygons& support_areas, const Polygons mesh_outlines, const coord_t safety_offset, Polygons& interface_polygons);

    /*!
     * Join current support layer with the support of the layer above, (make support conical) and perform smoothing etc operations.
     * 
     * \param supportLayer_up The support areas the layer above
     * \param supportLayer_this The overhang areas of the current layer at hand
     * \param supportJoinDistance The distance to be filled between two support areas
     * \param smoothing_distance Maximal distance in the X/Y directions of a line segment which is to be smoothed out. 
     * \param min_smoothing_area  minimal area for which to perform smoothing
     * \param conical_support Whether the support should be conical instead of cylindrical
     * \param conical_support_offset The offset determining the angle of the conical support
     * \param conical_smallest_breadth The breadth of the smallest support area which is not to be redoces to a smaller size due to conical support.
     * 
     * \return The joined support areas for this layer.
     */
    static Polygons join(const Polygons& supportLayer_up, Polygons& supportLayer_this, int64_t supportJoinDistance, int64_t smoothing_distance, int min_smoothing_area, bool conical_support, int64_t conical_support_offset, int64_t conical_smallest_breadth);

    /*!
     * Move the support up from model (cut away polygons to ensure bottom z distance)
     * and apply stair step transformation.
     * 
     * If the bottom stairs defined only by the step height are too wide,
     * the top half of the step will be as wide as the stair step width
     * and the bottom half will follow the model.
     * 
     * \param storage Where to get model outlines from
     * \param[in,out] stair_removal The polygons to be removed for stair stepping on the current layer (input) and for the next layer (output). Only changed every [step_height] layers.
     * \param[in,out] support_areas The support areas before and after this function
     * \param layer_idx The layer number of the support layer we are processing
     * \param bottom_empty_layer_count The number of empty layers between the bottom of support and the top of the model on which support rests
     * \param bottom_stair_step_layer_count The max height (in nr of layers) of the support bottom stairs
     * \param support_bottom_stair_step_width The max width of the support bottom stairs
     */
    static void moveUpFromModel(const SliceDataStorage& storage, Polygons& stair_removal, Polygons& support_areas, const int layer_idx, const int bottom_empty_layer_count, const unsigned int bottom_stair_step_layer_count, const coord_t support_bottom_stair_step_width);

    /*!
     * Joins the layerpart outlines of all meshes and collects the overhang points (small areas).
     * \param storage input layer outline information
     * \param overhang_points stores overhang_points of each layer
     * \param layer_count total number of layers
     * \param supportMinAreaSqrt diameter of the minimal area which can be supported without a specialized strut
     */
    static void detectOverhangPoints(
        const SliceDataStorage& storage,
        const SliceMeshStorage& mesh,
        std::vector<std::vector<Polygons>>& overhang_points,
        int layer_count,
        int supportMinAreaSqrt
    );
    
    /*!
     * Compute the basic overhang and full overhang of a layer. 
     * The basic overhang consists of the parts of this layer which are too far away from the layer below to be supported.
     * The full overhang consists of the basic overhang extended toward the border of the layer below.
     * 
     *             layer 2
     * layer 1 ______________|
     * _______|         ^^^^^ basic overhang
     *         ^^^^^^^^^^^^^^ full overhang
     * 
     * \param storage The slice data storage
     * \param mesh The mesh for which to compute the basic overhangs
     * \param layer_idx The layer for which to compute the overhang
     * \param max_dist_from_lower_layer The outward distance from the layer below which can be supported by it
     * \return a pair of basic overhang and full overhang
     */
    static std::pair<Polygons, Polygons> computeBasicAndFullOverhang(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const unsigned int layer_idx, const int64_t max_dist_from_lower_layer);
    
    /*!
     * Adds tower pieces to the current support layer.
     * From below the roof, the towers are added to the normal support layer and handled as normal support area.
     * \param supportLayer_this The support areas in the layer for which we are creating towers/struts
     * \param towerRoofs The parts of roofs which need to expand downward until they have the required diameter
     * \param overhang_points stores overhang_points of each layer
     * \param layer_idx The index of the layer at which to handle towers
     * \param towerRoofExpansionDistance The offset distance which determines the angle of the tower roof tops
     * \param supportTowerDiameter The diameter of the eventual tower, below the roof
     * \param supportMinAreaSqrt diameter of the minimal area which can be supported without a specialized strut
     * \param layer_count total number of layers
     * \param z_layer_distance_tower The number of layers between an overhang point and the top of a support tower
     */
    static void handleTowers(
        Polygons& supportLayer_this,
        std::vector<Polygons>& towerRoofs,
        std::vector<std::vector<Polygons>>& overhang_points,
        int layer_idx,
        int towerRoofExpansionDistance,
        int supportTowerDiameter,
        int supportMinAreaSqrt,
        int layer_count,
        int z_layer_distance_tower
    );
    
    /*!
     * Adds struts (towers against a wall) to the current layer.
     * \param supportLayer_this The areas of the layer for which to handle the wall struts.
     * \param supportMinAreaSqrt The minimal diameter of a wall which doesn't need a strut for reinforcement
     * \param suportTowerDiameter The diameter of the strut
     */
    static void handleWallStruts(
        Polygons& supportLayer_this,
        int supportMinAreaSqrt,
        int supportTowerDiameter
    );

    /*!
     * Clean up the SupportInfillParts.
     * Remove parts which have nothing to be printed.
     * 
     * Remove parts which are too small for the first wall.
     * For parts without walls: remove if combined into upper layers.
     * 
     */
    static void cleanup(SliceDataStorage& storage);
};


}//namespace cura

#endif//SUPPORT_H
