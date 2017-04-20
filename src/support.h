/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
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
private:
    /*!
     * Generate support polygons over all layers for one object.
     *
     * This function also handles small overhang areas (creates towers with larger diameter than just the overhang area) and single walls which could otherwise fall over.
     *
     * The anti_overhang areas are taken into account.
     *
     * \warning This function should be called only once for handling support meshes.
     * The \p mesh_idx should then correspond to an empty \ref SliceMeshStorage
     *
     * \param storage data storage containing the input layer outline data
     * \param infill_settings The settings base to get the settings from which are based on the infill of the support
     * \param interface_settings The settings base to get the settings from which are based on the interface of the support
     * \param mesh_idx The index of the object for which to generate support areas
     * \param layer_count total number of layers
     */
    static void generateSupportAreas(SliceDataStorage& storage, const SettingsBaseVirtual& infill_settings, const SettingsBaseVirtual& interface_settings, unsigned int mesh_idx, unsigned int layer_count, std::vector<Polygons>& supportAreas);



    /*!
     * Generate support skin areas and non-skin areas for a given mesh.
     *
     * \param storage Output storage: support area + support skin area output
     * \param mesh The mesh to generate support skins for.
     * \param layer_count The number of layers in this mesh group.
     */
    static void generateSupportInterface(SliceDataStorage& storage, const SliceMeshStorage& mesh, const unsigned int layer_count);

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
    static Polygons join(Polygons& supportLayer_up, Polygons& supportLayer_this, int64_t supportJoinDistance, int64_t smoothing_distance, int min_smoothing_area, bool conical_support, int64_t conical_support_offset, int64_t conical_smallest_breadth);

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
};


}//namespace cura

#endif//SUPPORT_H
