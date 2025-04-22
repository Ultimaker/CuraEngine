// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef TREEMODELVOLUMES_H
#define TREEMODELVOLUMES_H

#include <future>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include "TreeSupportEnums.h"
#include "TreeSupportEnums.h"
#include "geometry/Shape.h"
#include "settings/EnumSettings.h" //To store whether X/Y or Z distance gets priority.
#include "settings/types/LayerIndex.h" //Part of the RadiusLayerPair.
#include "utils/PairHash.h"
#include "utils/Simplify.h"

namespace cura
{
constexpr coord_t EPSILON = 5;
constexpr coord_t FUDGE_LENGTH = 50;

class SliceDataStorage;
class SliceMeshStorage;
class LayerIndex;
class Settings;

/*!
 * \brief Lazily generates tree guidance volumes.
 *
 */
class TreeModelVolumes
{
public:
    TreeModelVolumes() = default;
    TreeModelVolumes(
        const SliceDataStorage& storage,
        coord_t max_move,
        coord_t max_move_slow,
        coord_t min_offset_per_step,
        const coord_t min_radius,
        size_t current_mesh_idx,
        double progress_multiplier,
        double progress_offset,
        const std::vector<Shape>& additional_excluded_areas = std::vector<Shape>());
    TreeModelVolumes(TreeModelVolumes&&) = default;
    TreeModelVolumes& operator=(TreeModelVolumes&&) = default;

    TreeModelVolumes(const TreeModelVolumes&) = delete;
    TreeModelVolumes& operator=(const TreeModelVolumes&) = delete;


    /*!
     * \brief Precalculate avoidances and collisions up to this layer.
     *
     * This uses knowledge about branch angle to only calculate avoidances and collisions that could actually be needed.
     * Not calling this will cause the class to lazily calculate avoidances and collisions as needed, which will be a lot slower on systems with more then one or two cores!
     *
     */
    void precalculate(LayerIndex max_layer);

    /*!
     * \brief Provides the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model.
     *
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param min_xy_dist Is the minimum xy distance used.
     * \return Shape object
     */
    const Shape& getCollision(coord_t radius, LayerIndex layer_idx, bool min_xy_dist = false);

    /*!
     * \brief Provides the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer. Holes are removed.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model or be inside a hole.
     * A Hole is defined as an area, in which a branch with increase_until_radius radius would collide with the wall.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param min_xy_dist Is the minimum xy distance used.
     * \return Shape object
     */
    const Shape& getCollisionHolefree(coord_t radius, LayerIndex layer_idx, bool min_xy_dist = false);


    /*!
     * \brief Provides the area where there is model/build-plate below (vs support blocker)
     *
     * The result is a 2D area that represents where if support were to be placed in and just dropped down it would not rest on support blocker.
     * \param layer_idx The layer of interest
     * \return Shape object
     */
    const Shape& getAccumulatedPlaceable0(LayerIndex layer_idx);

    /*!
     * \brief Provides the areas that have to be avoided by the tree's branches
     * in order to reach the build plate.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model or be unable to reach the build platform.
     *
     * The input collision areas are inset by the maximum move distance and
     * propagated upwards.
     *
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param slow Is the propagation with the maximum move distance slow required.
     * \param to_model Does the avoidance allow good connections with the model.
     * \param min_xy_dist is the minimum xy distance used.
     * \return Shape object
     */
    const Shape& getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model = false, bool min_xy_dist = false);

    /*!
     * \brief Provides the area represents all areas on the model where the branch does completely fit on the given layer.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \return Shape object
     */
    const Shape& getPlaceableAreas(coord_t radius, LayerIndex layer_idx);

    /*!
     * \brief Provides the area that represents the walls, as in the printed area, of the model. This is an abstract representation not equal with the outline. See
     * calculateWallRestrictions for better description. \param radius The radius of the node of interest. \param layer_idx The layer of interest. \param min_xy_dist is the minimum
     * xy distance used. \return Shape object
     */
    const Shape& getWallRestriction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist);


    /*!
     * \brief Round \p radius upwards to either a multiple of radius_sample_resolution_ or a exponentially increasing value
     *
     *	It also adds the difference between the minimum xy distance and the regular one.
     *
     * \param radius The radius of the node of interest
     * \param min_xy_dist is the minimum xy distance used.
     * \return The rounded radius
     */
    coord_t ceilRadius(coord_t radius, bool min_xy_dist) const;

    /*!
     * \brief Round \p radius upwards to the maximum that would still round up to the same value as the provided one.
     *
     * \param radius The radius of the element of interest
     * \param min_xy_dist is the minimum xy distance used.
     * \return The maximum radius, resulting in the same rounding.
     */
    coord_t getRadiusNextCeil(coord_t radius, bool min_xy_dist) const;

    /*!
     * \brief Get the first layer index that contains hints about to areas which should be avoided.
     */
    LayerIndex getFirstAntiPreferredLayerIdx();
    //todo[TR:CodeQuality] doku & refactor above or below to be identical (either give first layer with or last layer without)
    LayerIndex getMaxLayerIdxWithoutBlocker();

    /*!
     * \brief Provide hints which areas should be avoided in the future.
     * \param area The area that should be avoided in the future.
     * \param layer_idx The layer said area is on.
     */
    void addAreaToAntiPreferred(const Shape area, LayerIndex layer_idx);

    /*!
     * \brief Precalculate avoidances for the anti-preferred area hints
     *
     * Needs to be called so that anti preferred avoidances are available.
     *
     */
    void precalculateAntiPreferred();

    /*!
     * \brief Get areas that were additionally set to be avoided
     * \param layer_idx The layer said area is on.
     * \param radius The radius of the node of interest.
     * \returns The area that should be avoided
     */
    const Shape& getAntiPreferredAreas(LayerIndex layer_idx, coord_t radius);

    /*!
     * \brief Get avoidance areas for areas that were additionally set to be avoided
     * \param layer_idx The layer said area is on.
     * \param radius The radius of the node of interest.
     * \returns The area that should be avoided
     */
    const Shape& getAntiPreferredAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model, bool min_xy_dist);

    /*!
     * \brief Get areas that were are classified as support blocker
     * \param layer_idx The layer said area is on.
     * \returns The area should not contain support
     */
    const Shape& getSupportBlocker(LayerIndex layer_idx);


private:
    /*!
     * \brief Convenience typedef for the keys to the caches
     */
    using RadiusLayerPair = std::pair<coord_t, LayerIndex>;

    /*!
     * \brief Round \p radius upwards to either a multiple of radius_sample_resolution_ or a exponentially increasing value
     *
     * \param radius The radius of the node of interest
     */
    coord_t ceilRadius(coord_t radius) const;

    /*!
     * \brief Extracts the relevant outline from a mesh
     * \param[in] mesh The mesh which outline will be extracted
     * \param layer_idx The layer which should be extracted from the mesh
     * \return Shape object representing the outline
     */
    Shape extractOutlineFromMesh(const SliceMeshStorage& mesh, LayerIndex layer_idx) const;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollision(const std::deque<RadiusLayerPair>& keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model. Result is saved in the cache.
     * \param key RadiusLayerPairs the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculateCollision(RadiusLayerPair key)
    {
        calculateCollision(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
    }

    /*!
     * \brief Calculates where there is model/build-plate below (vs support blocker).
     *
     * The result is a 2D area that represents where if support were to be placed in and just dropped down it would not rest on support blocker. Result is saved in the cache.
     * \param max_layer The layer up to which the area will be calculated.
     */
    void calculateAccumulatedPlaceable0(const LayerIndex max_layer);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer and not rest on support blocker.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model and not rest on support blocker. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollisionAvoidance(const std::deque<RadiusLayerPair>& keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer and not rest on support blocker.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model and not rest on support blocker. Result is saved in the cache.
     * \param key RadiusLayerPairs the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculateCollisionAvoidance(RadiusLayerPair key)
    {
        calculateCollisionAvoidance(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
    }

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer. Holes are removed.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model or be inside a hole. Result is saved in the cache.
     * A Hole is defined as an area, in which a branch with increase_until_radius radius would collide with the wall.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollisionHolefree(const std::deque<RadiusLayerPair>& keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer. Holes are removed.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model or be inside a hole. Result is saved in the cache.
     * A Hole is defined as an area, in which a branch with increase_until_radius radius would collide with the wall.
     * \param key RadiusLayerPairs the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculateCollisionHolefree(RadiusLayerPair key)
    {
        calculateCollisionHolefree(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
    }

    Shape safeOffset(const Shape& me, coord_t distance, ClipperLib::JoinType jt, coord_t max_safe_step_distance, const Shape& collision) const;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateAvoidance(const std::deque<RadiusLayerPair>& keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model. Result is saved in the cache.
     * \param key RadiusLayerPair of the requested areas. It will be calculated up to the provided layer.
     */
    void calculateAvoidance(RadiusLayerPair key)
    {
        calculateAvoidance(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
    }

    /*!
     * \brief Creates the areas where a branch of a given radius can be place on the model.
     * Result is saved in the cache.
     * \param key RadiusLayerPair of the requested areas. It will be calculated up to the provided layer.
     */
    void calculatePlaceables(RadiusLayerPair key)
    {
        calculatePlaceables(std::deque<RadiusLayerPair>{ key });
    }

    /*!
     * \brief Creates the areas where a branch of a given radius can be placed on the model.
     * Result is saved in the cache.
     * \param keys RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
     *
     */
    void calculatePlaceables(const std::deque<RadiusLayerPair>& keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model without being able to place a branch with given radius on a
     * single layer.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model in a not wanted way. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     *
     */
    void calculateAvoidanceToModel(const std::deque<RadiusLayerPair>& keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model without being able to place a branch with given radius on a
     * single layer.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model in a not wanted way. Result is saved in the cache.
     * \param key RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculateAvoidanceToModel(RadiusLayerPair key)
    {
        calculateAvoidanceToModel(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
    }

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model with radius 0 from the smallest actual avoidance.
     * This is done as a real 0 radius avoidance would not be able to be printed. These 0 radius avoidances are used for calculating roof and cradle.
     *
     * The result is a 2D area that would cause nodes of radius 0 to
     * collide with the model in a not wanted way. Result is saved in the cache.
     * \param max_layer The result will be calculated up to the this layer.
     */
    void calculateFake0Avoidances(const LayerIndex max_layer);


    /*!
     * \brief Creates the areas that can not be passed when expanding an area downwards. As such these areas are an somewhat abstract representation of a wall (as in a printed
     * object).
     *
     * These areas are at least xy_min_dist wide. When calculating it is always assumed that every wall is printed on top of another (as in has an overlap with the wall a layer
     * below). Result is saved in the corresponding cache.
     *
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     *
     * \return A future that has to be waited on
     */
    void calculateWallRestrictions(const std::deque<RadiusLayerPair>& keys);

    /*!
     * \brief Creates the areas that can not be passed when expanding an area downwards. As such these areas are an somewhat abstract representation of a wall (as in a printed
     * object). These areas are at least xy_min_dist wide. When calculating it is always assumed that every wall is printed on top of another (as in has an overlap with the wall a
     * layer below). Result is saved in the corresponding cache. \param key RadiusLayerPair of the requested area. It well be will be calculated up to the provided layer.
     */
    void calculateWallRestrictions(RadiusLayerPair key)
    {
        calculateWallRestrictions(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
    }

    /*!
     * \brief Checks a cache for a given RadiusLayerPair and returns it if it is found
     * \param key RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
     * \return A wrapped optional reference of the requested area (if it was found, an empty optional if nothing was found)
     */
    template<typename KEY>
    const std::optional<std::reference_wrapper<const Shape>> getArea(const std::unordered_map<KEY, Shape>& cache, const KEY key) const;

    bool checkSettingsEquality(const Settings& me, const Settings& other) const;

    /*!
     * \brief Get the highest already calculated layer in the cache.
     * \param radius The radius for which the highest already calculated layer has to be found.
     * \param map The cache in which the lookup is performed.
     *
     * \return A wrapped optional reference of the requested area (if it was found, an empty optional if nothing was found)
     */
    LayerIndex getMaxCalculatedLayer(coord_t radius, const std::unordered_map<RadiusLayerPair, Shape>& map) const;

    static Shape calculateMachineBorderCollision(const Shape&& machine_border);

    /*!
     * \brief The maximum distance that the center point of a tree branch may move in consecutive layers if it has to avoid the model.
     */
    coord_t max_move_;

    /*!
     * \brief The maximum distance that the centre-point of a tree branch may
     * move in consecutive layers if it does not have to avoid the model
     */
    coord_t max_move_slow_;

    /*!
     * \brief For use in 'safeOffset' functions. How many steps at a minimum, to do the offset in.
     * Too high may produce artifacts, too low will take forever.
     */
    coord_t min_offset_per_step_;

    /*!
     * \brief Whether the precalculate was called, meaning every required value should be cached.
     */
    bool precalculated_ = false;

    /*!
     * \brief Whether the precalculate was called and finished, meaning every required value should be cached.
     */
    bool precalculation_finished_ = false;

    /*!
     * \brief The index to access the outline corresponding with the currently processing mesh
     */
    size_t current_outline_idx_;

    /*!
     * \brief The minimum required clearance between the model and the tree branches
     */
    coord_t current_min_xy_dist_;

    /*!
     * \brief The difference between the minimum required clearance between the model and the tree branches and the regular one.
     */
    coord_t current_min_xy_dist_delta_;

    /*!
     * \brief The top most layer where there is no anti_overhang on any layer below
     */
    LayerIndex max_layer_idx_without_blocker_;

    /*!
     * \brief Does at least one mesh allow support to rest on a model.
     */
    bool support_rests_on_model_;

    /*!
     * \brief The progress of the precalculate function for communicating it to the progress bar.
     */
    coord_t precalculation_progress_ = 0;

    /*!
     * \brief The progress multiplier of all values added progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_multiplier_;

    /*!
     * \brief The progress offset added to all values communicated to the progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_offset_;

    /*!
     * \brief Increase radius in the resulting drawn branches, even if the avoidance does not allow it. Will be cut later to still fit.
     */
    coord_t increase_until_radius_;

    /*!
     * \brief Shape representing the limits of the printable area of the
     * machine
     */
    Shape machine_border_;

    /*!
     * \brief Shape representing the printable area of the machine
     */
    Shape machine_area_;

    /*!
     * \brief Storage for layer outlines and the corresponding settings of the meshes grouped by meshes with identical setting.
     */
    std::vector<std::pair<Settings, std::vector<Shape>>> layer_outlines_;

    /*!
     * \brief Storage for areas that should be avoided, like support blocker or previous generated trees.
     */
    std::vector<Shape> anti_overhang_;

    /*!
     * \brief Radii that can be ignored by ceilRadius as they will never be requested.
     */
    std::unordered_set<coord_t> ignorable_radii_;

    /*!
     * \brief Smallest radius a branch can have. This is the radius of a SupportElement with DTT=0.
     */
    coord_t radius_0_;

    /*!
     * \brief Does the main model require regular avoidance, or only avoidance to model.
     */
    RestPreference support_rest_preference_;

    /*!
     * \brief Largest DTT a cradle supporting tip may have.
     */
    size_t max_cradle_dtt = 0;

    LayerIndex first_anti_preferred_layer_idx_ = 0;

    /*!
     * \brief radii for which avoidance was already precalculated. Used to calculate anti preferred avoidance.
     */
    std::deque<RadiusLayerPair> precalculated_avoidance_radii;

    /*!
     * \brief Caches for the collision, avoidance and areas on the model where support can be placed safely
     * at given radius and layer indices.
     *
     * These are mutable to allow modification from const function. This is
     * generally considered OK as the functions are still logically const
     * (ie there is no difference in behaviour for the user between
     * calculating the values each time vs caching the results).
     */
    mutable std::unordered_map<RadiusLayerPair, Shape> collision_cache_;
    std::unique_ptr<std::mutex> critical_collision_cache_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> collision_cache_holefree_;
    std::unique_ptr<std::mutex> critical_collision_cache_holefree_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<LayerIndex, Shape> accumulated_placeables_cache_radius_0_;
    std::unique_ptr<std::mutex> critical_accumulated_placeables_cache_radius_0_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> avoidance_cache_collision_;
    std::unique_ptr<std::mutex> critical_avoidance_cache_collision_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> avoidance_cache_;
    std::unique_ptr<std::mutex> critical_avoidance_cache_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> avoidance_cache_slow_;
    std::unique_ptr<std::mutex> critical_avoidance_cache_slow_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> avoidance_cache_to_model_;
    std::unique_ptr<std::mutex> critical_avoidance_cache_to_model_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> avoidance_cache_to_model_slow_;
    std::unique_ptr<std::mutex> critical_avoidance_cache_to_model_slow_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> placeable_areas_cache_;
    std::unique_ptr<std::mutex> critical_placeable_areas_cache_ = std::make_unique<std::mutex>();

    /*!
     * \brief Caches to avoid holes smaller than the radius until which the radius is always increased, as they are free of holes. Also called safe avoidances, as they are safe
     * regarding not running into holes.
     */
    mutable std::unordered_map<RadiusLayerPair, Shape> avoidance_cache_hole_;
    std::unique_ptr<std::mutex> critical_avoidance_cache_holefree_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> avoidance_cache_hole_to_model_;
    std::unique_ptr<std::mutex> critical_avoidance_cache_holefree_to_model_ = std::make_unique<std::mutex>();

    /*!
     * \brief Caches to represent walls not allowed to be passed over.
     */
    mutable std::unordered_map<RadiusLayerPair, Shape> wall_restrictions_cache_;
    std::unique_ptr<std::mutex> critical_wall_restrictions_cache_ = std::make_unique<std::mutex>();

    // A different cache for min_xy_dist as the maximal safe distance an influence area can be increased(guaranteed overlap of two walls in consecutive layer) is much smaller when
    // min_xy_dist is used. This causes the area of the wall restriction to be thinner and as such just using the min_xy_dist wall restriction would be slower.
    mutable std::unordered_map<RadiusLayerPair, Shape> wall_restrictions_cache_min_;
    std::unique_ptr<std::mutex> critical_wall_restrictions_cache_min_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> anti_preferred_;
    std::unique_ptr<std::mutex> critical_anti_preferred_ = std::make_unique<std::mutex>();

    mutable std::unordered_map<RadiusLayerPair, Shape> anti_preferred_cache_;
    mutable std::unordered_map<RadiusLayerPair, Shape> anti_preferred_cache_to_model_;
    mutable std::unordered_map<RadiusLayerPair, Shape> anti_preferred_cache_collision;
    std::unique_ptr<std::mutex> critical_anti_preferred_caches = std::make_unique<std::mutex>();

    enum class CalculationType
    {
        AVOIDANCE,
        AVOIDANCE_0,
        AVOIDANCE_TO_MODEL,
        AVOIDANCE_COLLISION,
        COLLISION,
        COLLISION_HOLEFREE,
        PLACEABLE,
        PLACEABLE_ACCUMULATED,
        WALL_RESTRICTION
    };
    std::unique_ptr<std::mutex> critical_calculation_request_ = std::make_unique<std::mutex>();
    std::unordered_map<std::pair<coord_t, CalculationType>, std::shared_ptr<std::mutex>> critical_calculation_map_;

    std::shared_ptr<std::mutex> getLockForCalculation(coord_t radius, CalculationType type)
    {
        std::lock_guard<std::mutex> critical_section(*critical_calculation_request_);
        std::pair<coord_t, CalculationType> key(radius, type);
        if (! critical_calculation_map_.contains(key))
        {
            critical_calculation_map_[key] = std::make_shared<std::mutex>();
        }
        return critical_calculation_map_[key];
    }

    std::unique_ptr<std::mutex> critical_progress_ = std::make_unique<std::mutex>();

    Simplify simplifier_ = Simplify(0, 0, 0); // a simplifier to simplify polygons. Will be properly initialised in the constructor.

    Shape empty_polygon = Shape();
};

} // namespace cura

#endif // TREEMODELVOLUMES_H
