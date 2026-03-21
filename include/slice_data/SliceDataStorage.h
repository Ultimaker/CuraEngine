// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICEDATA_SLICEDATASTORAGE_H
#define SLICEDATA_SLICEDATASTORAGE_H

#include "geometry/MixedLinesSet.h"
#include "settings/Settings.h"
#include "settings/types/LayerIndex.h"
#include "utils/AABB3D.h"
#include "utils/NoCopy.h"
#include "slice_data/SupportStorage.h"
#include "WipeScriptConfig.h"

namespace cura
{

class PrimeTower;

class SliceDataStorage : public NoCopy
{
public:
    const Settings& settings_; // The settings for the mesh group being processed by this storage
    size_t print_layer_count; //!< The total number of layers (except the raft and filler layers)

    Point3LL model_size, model_min, model_max;
    AABB3D machine_size; //!< The bounding box with the width, height and depth of the printer.
    std::vector<std::shared_ptr<SliceMeshStorage>> meshes;

    std::vector<RetractionAndWipeConfig> retraction_wipe_config_per_extruder; //!< Config for retractions, extruder switch retractions, and wipes, per extruder.

    SupportStorage support;

    std::vector<MixedLinesSet> skirt_brim[MAX_EXTRUDERS]; //!< Skirt/brim polygons per extruder, ordered from inner to outer polygons.
    ClosedLinesSet support_brim; //!< brim lines for support, going from the edge of the support inward. \note Not ordered by inset.

    // Storage for the outline of the raft-parts. Will be filled with lines when the GCode is generated.
    Shape raft_base_outline;
    Shape raft_interface_outline;
    Shape raft_surface_outline;

    int max_print_height_second_to_last_extruder; //!< Used in multi-extrusion: the layer number beyond which all models are printed with the same extruder
    std::vector<int> max_print_height_per_extruder; //!< For each extruder the highest layer number at which it is used.
    std::vector<size_t> max_print_height_order; //!< Ordered indices into max_print_height_per_extruder: back() will return the extruder number with the highest print height.

    std::vector<int> spiralize_seam_vertex_indices; //!< the index of the seam vertex for each layer
    std::vector<Shape*> spiralize_wall_outlines; //!< the wall outline polygons for each layer

    //!< Pointer to primer tower handler object (a null pointer indicates that there is no prime tower)
    PrimeTower* prime_tower_{ nullptr };

    std::vector<Shape> ooze_shield; // oozeShield per layer
    Shape draft_protection_shield; //!< The polygons for a heightened skirt which protects from warping by gusts of wind and acts as a heated chamber.

    /*!
     * \brief Creates a new slice data storage that stores the slice data of the
     * current mesh group.
     */
    SliceDataStorage(const Settings &settings);

    ~SliceDataStorage();

    /*!
     * Get all outlines within a given layer.
     *
     * \param layer_nr The index of the layer for which to get the outlines
     * (negative layer numbers indicate the raft).
     * \param include_support Whether to include support in the outline.
     * \param include_prime_tower Whether to include the prime tower in the outline.
     * \param include_models Whether to include the models in the outline
     * \param external_polys_only Whether to disregard all hole polygons.
     * \param extruder_nr (optional) only give back outlines for this extruder (where the walls are printed with this extruder)
     */
    Shape getLayerOutlines(
        const LayerIndex layer_nr,
        const bool include_support,
        const bool include_prime_tower,
        const bool external_polys_only = false,
        const int extruder_nr = -1,
        const bool include_models = true) const;

    /*!
     * Get the axis-aligned bounding-box of the complete model (all meshes).
     */
    AABB3D getModelBoundingBox() const;

    /*!
     * Get the extruders used.
     *
     * \return A vector of booleans indicating whether the extruder with the
     * corresponding index is used in the mesh group.
     */
    std::vector<bool> getExtrudersUsed() const;

    /*!
     * Get the extruders used on a particular layer.
     *
     * \param layer_nr the layer for which to check
     * \return a vector of bools indicating whether the extruder with corresponding index is used in this layer.
     */
    std::vector<bool> getExtrudersUsed(LayerIndex layer_nr) const;

    /*!
     * Gets whether prime blob is enabled for the given extruder number.
     *
     * \param extruder_nr the extruder number to check.
     * \return a bool indicating whether prime blob is enabled for the given extruder number.
     */
    bool getExtruderPrimeBlobEnabled(const size_t extruder_nr) const;

    /*!
     * Gets the border of the usable print area for this machine.
     *
     * \param extruder_nr The extruder for which to return the allowed areas. -1 if the areas allowed for all extruders should be returned.
     * \return the Shape representing the usable area of the print bed.
     */
    Shape getMachineBorder(int extruder_nr = -1) const;

    /*!
     * @return The raw outer build plate shape without any disallowed area
     */
    Shape getRawMachineBorder() const;

    void initializePrimeTower();

private:
    /*!
     * Construct the retraction_wipe_config_per_extruder
     */
    std::vector<RetractionAndWipeConfig> initializeRetractionAndWipeConfigs();
};

} // namespace cura

#endif
