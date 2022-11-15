//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREEMODELVOLUMES_H
#define TREEMODELVOLUMES_H

#include <unordered_map>

#include "settings/EnumSettings.h" //To store whether X/Y or Z distance gets priority.
#include "settings/types/LayerIndex.h" //Part of the RadiusLayerPair.
#include "utils/polygon.h" //For polygon parameters.

namespace cura
{

class SliceDataStorage;
class LayerIndex;
class Settings;

/*!
 * \brief Lazily generates tree guidance volumes.
 *
 * \warning This class is not currently thread-safe and should not be accessed in OpenMP blocks
 */
class TreeModelVolumes
{
public:
    TreeModelVolumes() = default;
    /*!
     * \brief Construct the TreeModelVolumes object
     *
     * \param storage The slice data storage object to extract the model
     * contours from.
     * \param settings The settings object to get relevant settings from.
     * \param xy_distance The required clearance between the model and the
     * tree branches.
     * \param max_move The maximum allowable movement between nodes on
     * adjacent layers
     * \param radius_sample_resolution Sample size used to round requested node radii.
     */
    TreeModelVolumes(const SliceDataStorage& storage, const Settings& settings);

    TreeModelVolumes(TreeModelVolumes&& original) = default;
    TreeModelVolumes& operator=(TreeModelVolumes&& original) = default;

    TreeModelVolumes(const TreeModelVolumes& original) = delete;
    TreeModelVolumes& operator=(const TreeModelVolumes& original) = delete;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model.
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& getCollision(coord_t radius, LayerIndex layer_idx) const;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches
     * in order to reach the build plate.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model or be unable to reach the build platform.
     *
     * The input collision areas are inset by the maximum move distance and
     * propagated upwards.
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& getAvoidance(coord_t radius, LayerIndex layer_idx) const;

    /*!
     * \brief Generates the area of a given layer that must be avoided if the
     * branches wish to go towards the model
     *
     * The area represents the areas that do not collide with the model but
     * are unable to reach the build platform
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& getInternalModel(coord_t radius, LayerIndex layer_idx) const;

private:
    /*!
     * \brief Convenience typedef for the keys to the caches
     */
    using RadiusLayerPair = std::pair<coord_t, LayerIndex>;

    /*!
     * \brief Round \p radius upwards to a multiple of radius_sample_resolution_
     *
     * \param radius The radius of the node of interest
     */
    coord_t ceilRadius(coord_t radius) const;

    /*!
     * \brief Calculate the collision areas at the radius and layer indicated
     * by \p key.
     *
     * \param key The radius and layer of the node of interest
     */
    const Polygons& calculateCollision(const RadiusLayerPair& key) const;

    /*!
     * \brief Calculate the avoidance areas at the radius and layer indicated
     * by \p key.
     *
     * \param key The radius and layer of the node of interest
     */
    const Polygons& calculateAvoidance(const RadiusLayerPair& key) const;

    /*!
     * \brief Calculate the internal model areas at the radius and layer
     * indicated by \p key.
     *
     * \param key The radius and layer of the node of interest
     */
    const Polygons& calculateInternalModel(const RadiusLayerPair& key) const;

    /*!
     * \brief Calculate the collision area around the printable area of the machine.
     *
     * \param a Polygons object representing the non-printable areas on and around the build platform
     */
    static Polygons calculateMachineBorderCollision(const Polygons&& machine_border);

    /*!
     * \brief Polygons representing the limits of the printable area of the
     * machine
     */
    Polygons machine_border_;

    /*!
     * \brief The required clearance between the model and the tree branches
     */
    coord_t xy_distance_;

    /*!
     * The minimum X/Y distance between the model and the tree branches.
     *
     * Used only if the Z distance overrides the X/Y distance and in places that
     * are near the surface where the Z distance applies.
     */
    coord_t xy_distance_overhang;

    /*!
     * The number of layers of spacing to hold as Z distance.
     *
     * This determines where the overhang X/Y distance is used, if the Z
     * distance overrides the X/Y distance.
     */
    int z_distance_layers;

    /*!
     * The priority of X/Y distance over Z distance.
     */
    SupportDistPriority distance_priority;

    /*!
     * \brief The maximum distance that the centrepoint of a tree branch may
     * move in consequtive layers
     */
    coord_t max_move_;

    /*!
     * \brief Sample resolution for radius values.
     *
     * The radius will be rounded (upwards) to multiples of this value before
     * calculations are done when collision, avoidance and internal model
     * Polygons are requested.
     */
    coord_t radius_sample_resolution_;

    /*!
     * \brief Storage for layer outlines of the meshes.
     */
    std::vector<Polygons> layer_outlines_;

    /*!
     * \brief Caches for the collision, avoidance and internal model polygons
     * at given radius and layer indices.
     *
     * These are mutable to allow modification from const function. This is
     * generally considered OK as the functions are still logically const
     * (ie there is no difference in behaviour for the user betweeen
     * calculating the values each time vs caching the results).
     */
    mutable std::unordered_map<RadiusLayerPair, Polygons> collision_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> internal_model_cache_;
};

}

#endif //TREEMODELVOLUMES_H
