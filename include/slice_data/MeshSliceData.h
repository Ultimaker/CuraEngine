// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICEDATA_SLICEMESHSTORAGE_H
#define SLICEDATA_SLICEMESHSTORAGE_H

#include "slice_data/SliceLayer.h"
#include "utils/AABB3D.h"
#include "WipeScriptConfig.h"
#include "settings/types/LayerIndex.h"

namespace cura
{

class SubDivCube;
class Settings;
class SierpinskiFillProvider;
class LightningGenerator;
class Mesh;

class MeshSliceData
{
public:
    Settings& settings;
    std::vector<SliceLayer> layers;
    std::string mesh_name;

    LayerIndex layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content (modified while infill meshes are processed)

    std::vector<AngleDegrees> infill_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> roofing_angles; //!< a list of angle values which is cycled through to determine the roofing angle of each layer
    std::vector<AngleDegrees> flooring_angles; //!< a list of angle values which is cycled through to determine the flooring angle of each layer
    std::vector<AngleDegrees> skin_angles; //!< a list of angle values which is cycled through to determine the skin angle of each layer
    std::vector<Shape> overhang_areas; //!< For each layer the areas that are classified as overhang on this mesh.
    std::vector<Shape> full_overhang_areas; //!< For each layer the full overhang without the tangent of the overhang angle removed, such that the overhang area adjoins the
                                            //!< areas of the next layers.
    std::vector<std::vector<Shape>> overhang_points; //!< For each layer a list of points where point-overhang is detected. This is overhang that hasn't got any surface area,
                                                     //!< such as a corner pointing downwards.
    AABB3D bounding_box; //!< the mesh's bounding box

    std::shared_ptr<SubDivCube> base_subdiv_cube;
    std::shared_ptr<SierpinskiFillProvider> cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern

    std::shared_ptr<LightningGenerator> lightning_generator; //!< Pre-computed structure for Lightning type infill

    RetractionAndWipeConfig retraction_wipe_config; //!< Per-Object retraction and wipe settings.

    /*!
     * \brief Creates a storage space for slice results of a mesh.
     * \param mesh The mesh that the storage space belongs to.
     * \param slice_layer_count How many layers are needed to store the slice
     * results of the mesh. This needs to be at least as high as the highest
     * layer that contains a part of the mesh.
     */
    MeshSliceData(Mesh* mesh, const size_t slice_layer_count);

    /*!
     * \param extruder_nr The extruder for which to check
     * \return whether a particular extruder is used by this mesh
     */
    bool getExtruderIsUsed(const size_t extruder_nr) const;

    /*!
     * \param extruder_nr The extruder for which to check
     * \param layer_nr the layer for which to check
     * \return whether a particular extruder is used by this mesh on a particular layer
     */
    bool getExtruderIsUsed(const size_t extruder_nr, const LayerIndex& layer_nr) const;

    /*!
     * Gets whether this is a printable mesh (not an infill mesh, slicing mesh,
     * etc.)
     * \return True if it's a mesh that gets printed.
     */
    bool isPrinted() const;

    /*!
     * \return the mesh's user specified z seam hint
     */
    Point2LL getZSeamHint() const;
};

} // namespace cura

#endif
