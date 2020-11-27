//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SUPPORT_H
#define SUPPORT_H

namespace cura
{

struct LayerIndex;
class Settings;
class SliceDataStorage;
class SliceMeshStorage;
class Slicer;

class AreaSupport
{
public:
    /*!
     * \brief Move support mesh outlines from slicer data into the support
     * storage.
     * 
     * \param[out] storage Where to store the support areas.
     * \param mesh_settings Where to get the settings from what kind of support
     * mesh it is.
     * \param slicer Where to get the outlines from.
     * \return Whether the mesh is used up in support and no normal mesh
     * processing is needed.
     */
    static bool handleSupportModifierMesh(SliceDataStorage& storage, const Settings& mesh_settings, const Slicer* slicer);

    /*!
     * \brief Generate the overhang areas for all models.
     * \param storage Data storage containing the input layer data and
     * containing the output support storage per layer.
     */
    static void generateOverhangAreas(SliceDataStorage& storage);

    /*!
     * Generate the support areas and support skin areas for all models.
     * \param storage Data storage containing the input layer outline data and
     * containing the output support storage per layer.
     */
    static void generateSupportAreas(SliceDataStorage& storage);

    /*!
     * \brief Computes the base tree for cross infill of support.
     * \param storage[in,out] Data storage containing the input support outlines
     * and where to store the output tree.
     */
    static void precomputeCrossInfillTree(SliceDataStorage& storage);

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
     * \param max_resolution the intended minimum segment length
     * \param max_deviation the max deviation between the output poly and the input when trying to enforce the \p max_resolution
     */
    static void generateOutlineInsets(std::vector<Polygons>& insets, Polygons& outline, const unsigned int inset_count, const coord_t wall_line_width_x, const coord_t max_resolution, const coord_t max_deviation);

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
     * \brief Generate the overhang areas and points for a specific mesh.
     *
     * This function also handles small overhang areas and single walls which
     * would otherwise fall over. The anti_overhang areas are also taken into
     * account.
     * \param storage Data storage containing the input layer outlines.
     * \param mesh The object for which to generate overhang areas.
     */
    static void generateOverhangAreasForMesh(SliceDataStorage& storage, SliceMeshStorage& mesh);

    /*!
     * \brief Generate support polygons over all layers for one object.
     *
     * This function also handles small overhang areas (creates towers with
     * larger diameter than just the overhang area) and single walls which could
     * otherwise fall over.
     *
     * The anti_overhang areas are taken into account.
     *
     * \warning This function should be called only once for handling support
     * meshes with drop down and once for all support meshes without drop down.
     * The \p mesh_idx should then correspond to an empty \ref SliceMeshStorage
     * of one support mesh with the given value of support_mesh_drop_down.
     * 
     * \param storage Data storage containing the input layer outline data.
     * \param infill_settings The settings which are based on the infill of the
     * support.
     * \param roof_settings The settings which are based on the top interface of
     * the support.
     * \param bottom_settings The settings base to get the bottom interface of
     * the support.
     * \param mesh_idx The index of the object for which to generate support
     * areas.
     * \param layer_count Total number of layers.
     */
    static void generateSupportAreasForMesh(SliceDataStorage& storage, const Settings& infill_settings, const Settings& roof_settings, const Settings& bottom_settings, const size_t mesh_idx, const size_t layer_count, std::vector<Polygons>& support_areas);

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
     * \param outline_offset An offset applied to the result outlines.
     * \param minimum_interface_area Minimum area size for resulting interface polygons.
     * \param[out] interface_polygons The resulting interface layer. Do not use `interface` in windows!
     */
    static void generateSupportInterfaceLayer(Polygons& support_areas, const Polygons mesh_outlines, const coord_t safety_offset, const coord_t outline_offset, const double minimum_interface_area, Polygons& interface_polygons);

    /*!
     * \brief Join current support layer with the support of the layer above,
     * (make support conical) and perform smoothing etc. operations.
     * \param storage Where to store the resulting support.
     * \param supportLayer_up The support areas the layer above.
     * \param supportLayer_this The overhang areas of the current layer at hand.
     * \param smoothing_distance Maximal distance in the X/Y directions of a
     * line segment which is to be smoothed out.
     * \return The joined support areas for this layer.
     */
    static Polygons join(const SliceDataStorage& storage, const Polygons& supportLayer_up, Polygons& supportLayer_this, const coord_t smoothing_distance);

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
    static void moveUpFromModel(const SliceDataStorage& storage, Polygons& stair_removal, Polygons& sloped_areas, Polygons& support_areas, const size_t layer_idx, const size_t bottom_empty_layer_count, const size_t bottom_stair_step_layer_count, const coord_t support_bottom_stair_step_width);

    /*!
     * Joins the layer part outlines of all meshes and collects the overhang
     * points (small areas).
     * \param storage Input layer outline information.
     * \param mesh Output mesh to store the resulting overhang points in.
     */
    static void detectOverhangPoints(const SliceDataStorage& storage, SliceMeshStorage& mesh);
    
    /*!
     * \brief Compute the basic overhang and full overhang of a layer.
     *
     * The basic overhang consists of the parts of this layer which are too far
     * away from the layer below to be supported. The full overhang consists of
     * the basic overhang extended toward the border of the layer below.
     * 
     *             layer 2
     * layer 1 ______________|
     * _______|         ^^^^^ basic overhang
     *         ^^^^^^^^^^^^^^ full overhang
     * 
     * \param storage The slice data storage.
     * \param mesh The mesh for which to compute the basic overhangs.
     * \param layer_idx The layer for which to compute the overhang.
     * \return A pair of basic overhang and full overhang.
     */
    static std::pair<Polygons, Polygons> computeBasicAndFullOverhang(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const unsigned int layer_idx);
    
    /*!
     * \brief Adds tower pieces to the current support layer.
     *
     * From below the roof, the towers are added to the normal support layer and
     * handled as normal support area.
     * \param settings The settings to use for towers.
     * \param supportLayer_this The support areas in the layer for which we are
     * creating towers/struts
     * \param towerRoofs The parts of roofs which need to expand downward until
     * they have the required diameter
     * \param overhang_points stores overhang_points of each layer
     * \param layer_idx The index of the layer at which to handle towers
     * \param layer_count total number of layers
     */
    static void handleTowers(
        const Settings& settings,
        Polygons& supportLayer_this,
        std::vector<Polygons>& towerRoofs,
        std::vector<std::vector<Polygons>>& overhang_points,
        LayerIndex layer_idx,
        size_t layer_count
    );
    
    /*!
     * \brief Adds struts (towers against a wall) to the current layer.
     * \param settings The settings to use to create the wall struts.
     * \param supportLayer_this The areas of the layer for which to handle the
     * wall struts.
     */
    static void handleWallStruts(const Settings& settings, Polygons& supportLayer_this);

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
