// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"
#include "utils/polygon.h"
#include <forward_list>
#include <unordered_set>


namespace cura
{
class ModelVolumes
{
  public:
    ModelVolumes() = default;
    ModelVolumes(const SliceDataStorage& storage, const coord_t max_move, const coord_t max_move_slow, size_t current_mesh_idx, double progress_multiplier, double progress_offset, const std::vector<Polygons>& additional_excluded_areas = std::vector<Polygons>());
    ModelVolumes(ModelVolumes&&) = default;
    ModelVolumes& operator=(ModelVolumes&&) = default;

    ModelVolumes(const ModelVolumes&) = delete;
    ModelVolumes& operator=(const ModelVolumes&) = delete;

    enum class AvoidanceType
    {
        SLOW,
        FAST_SAFE,
        FAST
    };

    void precalculate(coord_t max_layer);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model.
     *
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param min_xy_dist Is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getCollision(coord_t radius, LayerIndex layer_idx, bool min_xy_dist = false);
    const Polygons& getCollisionHolefree(coord_t radius, LayerIndex layer_idx, bool min_xy_dist = false);


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
     * \param layer_idx The layer of interest
     * \param slow Is the propgation with the maximum move distance slow required.
     * \param to_model Does the avoidance allow good connections with the model.
     * \param min_xy_dist is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getAvoidance(coord_t radius, LayerIndex layer_idx, AvoidanceType type, bool to_model = false, bool min_xy_dist = false);
    /*!
     * \brief The area represents all areas on the model where the branch does completely fit on the given layer.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \return Polygons object
     */
    const Polygons& getPlaceableAreas(coord_t radius, LayerIndex layer_idx);

    /*!
     * \brief The area represents the walls, as in the printed area, of the model. This is an abstract representation not equal with the outline. See calculateWallRestictions for better description.
     * \param radius The radius of the node of interest.
     * \param layer_idx The layer of interest.
     * \param min_xy_dist is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getWallRestiction(coord_t radius, LayerIndex layer_idx, bool min_xy_dist);
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
     *
     * \param radius The radius of the node of interest
     * \param min_xy_dist is the minimum xy distance used.
     * \return The maximum radius, resulting in the same rounding.
     */
    coord_t getRadiusNextCeil(coord_t radius, bool min_xy_dist) const;


  private:
    /*!
     * \brief Convenience typedef for the keys to the caches
     */
    using RadiusLayerPair = std::pair<coord_t, LayerIndex>;

    using AvoidanceToModelPair = std::pair<RadiusLayerPair, bool>;


    /*!
     * \brief Round \p radius upwards to either a multiple of radius_sample_resolution_ or a exponentially increasing value
     *
     * \param radius The radius of the node of interest
     */
    coord_t ceilRadius(coord_t radius) const;

    /*!
     * \brief Extracts the relevant outling from a mesh
     * \param[in] mesh The mesh which outline will be extracted
     * \param layer_idx The layer which should be extracted from the mesh
     * \return Polygons object representing the outline
     */
    Polygons extractOutlineFromMesh(const SliceMeshStorage& mesh, LayerIndex layer_idx) const;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollision(std::deque<RadiusLayerPair> keys);
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
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model on this layer. Holes are removed.
     *
     * The result is a 2D area that would cause nodes of given radius to
     * collide with the model or be inside a hole. Result is saved in the cache.
     * A Hole is defined as an area, in which a branch with increase_until_radius radius would collide with the wall.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateCollisionHolefree(std::deque<RadiusLayerPair> keys);

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

    Polygons safeOffset(const Polygons& me, coord_t distance, ClipperLib::JoinType jt, coord_t max_safe_step_distance, const Polygons& collision) const;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateAvoidance(std::deque<RadiusLayerPair> keys);


    void calculateAvoidance(RadiusLayerPair key)
    {
#pragma omp parallel // required as otherwise the "omp for" inside calculateAvoidance will lock up
        {
            calculateAvoidance(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
        }
    }

    /*!
     * \brief Creates the areas where a branch of a given radius can be place on the model.
     * Result is saved in the cache.
     * \param key RadiusLayerPair of the requested areas. It will be calculated up to the provided layer.
     */
    void calculatePlaceables(RadiusLayerPair key)
    {
#pragma omp parallel
        {
            calculatePlaceables(std::deque<RadiusLayerPair>{ key });
        }
    }

    /*!
     * \brief Creates the areas where a branch of a given radius can be placed on the model.
     * Result is saved in the cache.
     * \param keys RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculatePlaceables(std::deque<RadiusLayerPair> keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model without being able to place a branch with given radius on a single layer.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model in a not wanted way. Result is saved in the cache.
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateAvoidanceToModel(std::deque<RadiusLayerPair> keys);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches to prevent collision with the model without being able to place a branch with given radius on a single layer.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model in a not wanted way. Result is saved in the cache.
     * \param key RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
     */
    void calculateAvoidanceToModel(RadiusLayerPair key)
    {
#pragma omp parallel // required as otherwise the "omp for" inside calculateAvoidance will lock up
        {
            calculateAvoidanceToModel(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
        }
    }
    /*!
     * \brief Creates the areas that can not be passed when expanding an area downwards. As such these areas are an somewhat abstract representation of a wall (as in a printed object).
     *
     * These areas are at least xy_min_dist wide. When calculating it is always assumed that every wall is printed on top of another (as in has an overlap with the wall a layer below). Result is saved in the corresponding cache.
     *
     * \param keys RadiusLayerPairs of all requested areas. Every radius will be calculated up to the provided layer.
     */
    void calculateWallRestictions(std::deque<RadiusLayerPair> keys);

    /*!
     * \brief Creates the areas that can not be passed when expanding an area downwards. As such these areas are an somewhat abstract representation of a wall (as in a printed object).
     * These areas are at least xy_min_dist wide. When calculating it is always assumed that every wall is printed on top of another (as in has an overlap with the wall a layer below). Result is saved in the corresponding cache.
     * \param key RadiusLayerPair of the requested area. It well be will be calculated up to the provided layer.
     */
    void calculateWallRestictions(RadiusLayerPair key)
    {
#pragma omp parallel
        calculateWallRestictions(std::deque<RadiusLayerPair>{ RadiusLayerPair(key) });
    }

    /*!
     * \brief Checks a cache for a given RadiusLayerPair and returns it if it is found
     * \param key RadiusLayerPair of the requested areas. The radius will be calculated up to the provided layer.
     * \return A wrapped optional reference of the requested area (if it was found, an empty optional if nothing was found)
     */
    template <typename KEY>
    const std::optional<std::reference_wrapper<const Polygons>> getArea(const std::unordered_map<KEY, Polygons>& cache, const KEY key) const;
    bool checkSettingsEquality(const Settings& me, const Settings& other) const;
    /*!
     * \brief Get the highest already calculated layer in the cache.
     * \param radius The radius for which the highest already calculated layer has to be found.
     * \param map The cache in which the lookup is performed.
     *
     * \return A wrapped optional reference of the requested area (if it was found, an empty optional if nothing was found)
     */
    LayerIndex getMaxCalculatedLayer(coord_t radius, const std::unordered_map<RadiusLayerPair, Polygons>& map) const;

    Polygons calculateMachineBorderCollision(Polygon machine_border);
    /*!
     * \brief The maximum distance that the center point of a tree branch may move in consecutive layers if it has to avoid the model.
     */
    coord_t max_move_;
    /*!
     * \brief The maximum distance that the centrepoint of a tree branch may
     * move in consequtive layers if it does not have to avoid the model
     */
    coord_t max_move_slow_;
    /*!
     * \brief The smallest maximum resolution for simplify
     */
    coord_t min_maximum_resolution_;
    /*!
     * \brief The smallest maximum deviation for simplify
     */
    coord_t min_maximum_deviation_;
    /*!
     * \brief Whether the precalculate was called, meaning every required value should be cached.
     */
    bool precalculated = false;
    /*!
     * \brief The index to access the outline corresponding with the currently processing mesh
     */
    size_t current_outline_idx;
    /*!
     * \brief The minimum required clearance between the model and the tree branches
     */
    coord_t current_min_xy_dist;
    /*!
     * \brief The difference between the minimum required clearance between the model and the tree branches and the regular one.
     */
    coord_t current_min_xy_dist_delta;
    /*!
     * \brief Does at least one mesh allow support to rest on a model.
     */
    bool support_rests_on_model;
    /*!
     * \brief The progress of the precalculate function for communicating it to the progress bar.
     */
    coord_t precalculation_progress = 0;
    /*!
     * \brief The progress multiplier of all values added progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_multiplier;
    /*!
     * \brief The progress offset added to all values communicated to the progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_offset;
    /*!
     * \brief Increase radius in the resulting drawn branches, even if the avoidance does not allow it. Will be cut later to still fit.
     */
    coord_t increase_until_radius;

    /*!
     * \brief Polygons representing the limits of the printable area of the
     * machine
     */
    Polygons machine_border_;
    /*!
     * \brief Storage for layer outlines and the corresponding settings of the meshes grouped by meshes with identical setting.
     */
    std::vector<std::pair<Settings, std::vector<Polygons>>> layer_outlines_;
    /*!
     * \brief Storage for areas that should be avoided, like support blocker or previous generated trees.
     */
    std::vector<Polygons> anti_overhang_;
    /*!
     * \brief Radiis that can be ignored by ceilRadius as they will never be requested.
     */
    std::unordered_set<coord_t> ignorable_radii_;

    /*!
     * \brief Caches for the collision, avoidance and areas on the model where support can be placed safely
     * at given radius and layer indices.
     *
     * These are mutable to allow modification from const function. This is
     * generally considered OK as the functions are still logically const
     * (ie there is no difference in behaviour for the user between
     * calculating the values each time vs caching the results).
     */
    mutable std::unordered_map<RadiusLayerPair, Polygons> collision_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> collision_cache_holefree_;

    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_slow_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_to_model_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_to_model_slow_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> placeable_areas_cache_;


    /*!
     * \brief Caches to avoid holes smaller than the radius until which the radius is always increased. Also called safe avoidances, as they are save regarding not running into holes.
     */
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_hole_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_hole_to_model;

    /*!
     * \brief Caches to represent walls not allowed to be passed over.
     */
    mutable std::unordered_map<RadiusLayerPair, Polygons> wall_restictions_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> wall_restictions_cache_min_; // A different cache for min_xy_dist as the maximal safe distance an influence area can be increased(guaranteed overlap of two walls in consecutive layer) is much smaller when min_xy_dist is used. This causes the area of the wall restriction to be thinner and as such just using the min_xy_dist wall restriction would be slower.
};

// Some forward definitions.
class SliceDataStorage;
class SliceMeshStorage;
class ModelVolumes;


/*!
 * \brief Generates a tree structure to support your models.
 */

class TreeSupport
{
  public:
    using AvoidanceType = ModelVolumes::AvoidanceType;

    /*!
     * \brief Creates an instance of the tree support generator.
     *
     * \param storage The data storage to get global settings from.
     */
    TreeSupport(const SliceDataStorage& storage);

    /*!
     * \brief Create the areas that need support.
     *
     * These areas are stored inside the given SliceDataStorage object.
     * \param storage The data storage where the mesh data is gotten from and
     * where the resulting support areas are stored.
     */
    void generateSupportAreas(SliceDataStorage& storage);


    //todo Remove! Only relevant for public BETA!
    static bool inline showed_critical=false;
    static bool inline showed_performance=false;
    static void showError(std::string message,bool critical);

    struct TreeSupportSettings; // forward declaration as we need some config values in the merge case

    struct AreaIncreaseSettings
    {
        AreaIncreaseSettings() : type(AvoidanceType::FAST), increase_speed(0), increase_radius(false), no_error(false), use_min_distance(false), move(false)
        {
        }

        AreaIncreaseSettings(AvoidanceType type, coord_t increase_speed, bool increase_radius, bool simplify, bool use_min_distance, bool move) : type(type), increase_speed(increase_speed), increase_radius(increase_radius), no_error(simplify), use_min_distance(use_min_distance), move(move)
        {
        }

        AvoidanceType type;
        coord_t increase_speed;
        bool increase_radius;
        bool no_error;
        bool use_min_distance;
        bool move;
        bool operator==(const AreaIncreaseSettings& other)
        {
            return increase_radius == other.increase_radius && increase_speed == other.increase_speed && type == other.type && no_error == other.no_error && use_min_distance == other.use_min_distance && move == other.move;
        }
    };

    struct SupportElement
    {
        SupportElement(coord_t distance_to_top, size_t target_height, Point target_position, bool to_buildplate, bool to_model_gracious, bool use_min_xy_dist, size_t dont_move_until, bool supports_roof, bool can_use_safe_radius)
            : // SupportElement(target_height,target_position,target_height,target_position,distance_to_top,distance_to_top,to_buildplate)
              target_height(target_height),
              target_position(target_position),
              next_position(target_position),
              next_height(target_height),
              effective_radius_height(distance_to_top),
              to_buildplate(to_buildplate),
              distance_to_top(distance_to_top),
              area(nullptr),
              result_on_layer(target_position),
              increased_to_model_radius(0),
              to_model_gracious(to_model_gracious),
              elephant_foot_increases(0),
              use_min_xy_dist(use_min_xy_dist),
              supports_roof(supports_roof),
              dont_move_until(dont_move_until),
              can_use_safe_radius(can_use_safe_radius),
              last_area_increase(AreaIncreaseSettings(AvoidanceType::FAST, 0, false, false, false, false))
        {
        }


        SupportElement(const SupportElement& elem, Polygons* newArea = nullptr)
            : // copy constructor with possibility to set a new area
              target_height(elem.target_height),
              target_position(elem.target_position),
              next_position(elem.next_position),
              next_height(elem.next_height),
              effective_radius_height(elem.effective_radius_height),
              to_buildplate(elem.to_buildplate),
              distance_to_top(elem.distance_to_top),
              area(newArea != nullptr ? newArea : elem.area),
              result_on_layer(elem.result_on_layer),
              increased_to_model_radius(elem.increased_to_model_radius),
              to_model_gracious(elem.to_model_gracious),
              elephant_foot_increases(elem.elephant_foot_increases),
              use_min_xy_dist(elem.use_min_xy_dist),
              supports_roof(elem.supports_roof),
              dont_move_until(elem.dont_move_until),
              can_use_safe_radius(elem.can_use_safe_radius),
              last_area_increase(elem.last_area_increase)

        {
            parents.insert(parents.begin(), elem.parents.begin(), elem.parents.end());
        }

        /*!
         * \brief Create a new Element for one layer below the element of the pointer supplied.
         */

        SupportElement(SupportElement* element_above)
            : target_height(element_above->target_height),
              target_position(element_above->target_position),
              next_position(element_above->next_position),
              next_height(element_above->next_height),
              effective_radius_height(element_above->effective_radius_height),
              to_buildplate(element_above->to_buildplate),
              distance_to_top(element_above->distance_to_top + 1),
              area(element_above->area),
              result_on_layer(Point(-1, -1)), // set to invalid as we are a new node on a new layer
              increased_to_model_radius(element_above->increased_to_model_radius),
              to_model_gracious(element_above->to_model_gracious),
              elephant_foot_increases(element_above->elephant_foot_increases),
              use_min_xy_dist(element_above->use_min_xy_dist),
              supports_roof(element_above->supports_roof),
              dont_move_until(element_above->dont_move_until),
              can_use_safe_radius(element_above->can_use_safe_radius),
              last_area_increase(element_above->last_area_increase)
        {
            parents = { element_above };
        }

        // ONLY to be called in merge as it assumes a few assurances made by it.
        SupportElement(const SupportElement& first, const SupportElement& second, size_t next_height, Point next_position, coord_t increased_to_model_radius, TreeSupportSettings config) : next_position(next_position), next_height(next_height), area(nullptr), increased_to_model_radius(increased_to_model_radius), use_min_xy_dist(first.use_min_xy_dist || second.use_min_xy_dist), supports_roof(first.supports_roof || second.supports_roof), dont_move_until(std::max(first.dont_move_until, second.dont_move_until)), can_use_safe_radius(first.can_use_safe_radius || second.can_use_safe_radius)

        {
            if (first.target_height > second.target_height)
            {
                target_height = first.target_height;
                target_position = first.target_position;
            }
            else
            {
                target_height = second.target_height;
                target_position = second.target_position;
            }
            effective_radius_height = std::max(first.effective_radius_height, second.effective_radius_height);
            distance_to_top = std::max(first.distance_to_top, second.distance_to_top);

            to_buildplate = first.to_buildplate && second.to_buildplate;
            to_model_gracious = first.to_model_gracious && second.to_model_gracious; // valid as we do not merge non gracious with gracious

            AddParents(first.parents);
            AddParents(second.parents);

            elephant_foot_increases = 0;
            if (config.diameter_scale_bp_radius > 0)
            {
                coord_t foot_increase_radius = std::abs(std::max(config.getCollisionRadius(second), config.getCollisionRadius(first)) - config.getCollisionRadius(*this));
                elephant_foot_increases = foot_increase_radius / (config.branch_radius * (config.diameter_scale_bp_radius - config.diameter_angle_scale_factor)); // elephant_foot_increases has to be recalculated, as when a smaller tree with a larger elephant_foot_increases merge with a larger branch the elephant_foot_increases may have to be lower as otherwise the radius suddenly increases. This results often in a non integer value.
            }


            // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
            last_area_increase = AreaIncreaseSettings(std::min(first.last_area_increase.type, second.last_area_increase.type), std::min(first.last_area_increase.increase_speed, second.last_area_increase.increase_speed), first.last_area_increase.increase_radius || second.last_area_increase.increase_radius, first.last_area_increase.no_error || second.last_area_increase.no_error, first.last_area_increase.use_min_distance && second.last_area_increase.use_min_distance, first.last_area_increase.move || second.last_area_increase.move);
        }

        /*!
         * \brief The layer this support elements wants reach
         */
        LayerIndex target_height;

        /*!
         * \brief The position this support elements wants to support on layer=target_height
         */
        Point target_position;

        /*!
         * \brief The next position this support elements wants to reach. NOTE: This is mainly a suggestion regarding direction inside the influence area.
         */
        Point next_position;


        /*!
         * \brief The next height this support elements wants to reach
         */
        LayerIndex next_height;

        /*!
         * \brief The Effective distance to top of this element regarding radius increases and collision calculations.
         */

        size_t effective_radius_height;

        /*!
         * \brief The element trys to reach the buildplate
         */

        bool to_buildplate;

        /*!
         * \brief All elements in the layer above the current one that are supported by this element
         */
        std::vector<SupportElement*> parents;

        /*!
         * \brief The amount of layers this element is below the topmost layer of this branch.
         */
        size_t distance_to_top;

        /*!
         * \brief The resulting influence area.
         * Will only be set in the results of createLayerPathing, and will be nullptr inside!
         */
        Polygons* area;

        /*!
         * \brief The resulting center point around which a circle will be drawn later.
         * Will be set by setPointsOnAreas
         */
        Point result_on_layer = Point(-1, -1);
        /*!
         * \brief The amount of extra radius we got from merging branches that could have reached the buildplate, but merged with ones that can not.
         */
        coord_t increased_to_model_radius; // how much to model we increased only relevant for merging
        /*!
         * \brief Will the branch be able to rest completly on a flat surface, be it buildplate or model ?
         */
        bool to_model_gracious;

        /*!
         * \brief Counter about the times the elephant foot was increased. Can be fractions for merge reasons.
         */
        double elephant_foot_increases;

        /*!
         * \brief Whether the min_xy_distance can be used to get avoidance or similar. Will only be true if support_xy_overrides_z=Z overrides X/Y.
         */
        bool use_min_xy_dist;

        /*!
         * \brief True if this Element or any parent provides support to a support roof.
         */
        bool supports_roof;

        /*!
         * \brief The element trys not to move until this dtt is reached, is set to 0 if the element had to move.
         */
        size_t dont_move_until;

        /*!
         * \brief An influence area is considered safe when it can use the holefree avoidance <=> It will not have to encounter holes on its way downward.
         */
        bool can_use_safe_radius;

        /*!
         * \brief Settings used to increase the influence area to its current state.
         */
        AreaIncreaseSettings last_area_increase;

        bool operator==(const SupportElement& other) const
        {
            return target_position == other.target_position && target_height == other.target_height;
        }

        bool operator<(const SupportElement& other) const // true if me < other
        {
            return !(*this == other) && !(*this > other);
        }
        bool operator>(const SupportElement& other) const
        {
            // Doesn't really have to make sense, only required for ordering in maps to ensure deterministic behavior.
            if (*this == other)
                return false;
            if (other.target_height != target_height)
            {
                return other.target_height < target_height;
            }

            return other.target_position.X == target_position.X ? other.target_position.Y < target_position.Y : other.target_position.X < target_position.X;
        }

        void AddParents(std::vector<SupportElement*> adding)
        {
            for (SupportElement* ptr : adding)
            {
                parents.emplace_back(ptr);
            }
        }
    };

    /*!
     * \brief This struct contains settings used in the tree support. Thanks to this most functions do not need to know of meshes etc. Also makes the code shorter.
     */
    struct TreeSupportSettings
    {
        TreeSupportSettings() = default; // required for the definition of the config variable in the TreeSupport class.

        TreeSupportSettings(const Settings& mesh_group_settings)
            : angle(mesh_group_settings.get<AngleRadians>("support_tree_angle")),
              angle_slow(mesh_group_settings.get<AngleRadians>("support_tree_angle_slow")),
              support_line_width(mesh_group_settings.get<coord_t>("support_line_width")),
              layer_height(mesh_group_settings.get<coord_t>("layer_height")),
              branch_radius(mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2),
              min_radius(1.25 * support_line_width / 2),
              maximum_move_distance((angle < TAU / 4) ? (coord_t)(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max()),
              maximum_move_distance_slow((angle_slow < TAU / 4) ? (coord_t)(tan(angle_slow) * layer_height) : std::numeric_limits<coord_t>::max()),
              support_bottom_layers(mesh_group_settings.get<bool>("support_bottom_enable") ? round_divide(mesh_group_settings.get<coord_t>("support_bottom_height"), layer_height) : 0),
              tip_layers(std::max((branch_radius - min_radius) / (support_line_width / 3), branch_radius / layer_height)), // ensures lines always stack nicely even if layer height is large
              diameter_angle_scale_factor(sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius),
              max_to_model_radius_increase(mesh_group_settings.get<coord_t>("support_tree_max_diameter_increase_by_merges_when_support_to_model") / 2),
              min_dtt_to_model(round_up_divide(mesh_group_settings.get<coord_t>("support_tree_min_height_to_model"), layer_height)),
              increase_radius_until_radius(mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2),
              increase_radius_until_layer(increase_radius_until_radius <= branch_radius ? tip_layers * (increase_radius_until_radius / branch_radius) : (increase_radius_until_radius - branch_radius) / (branch_radius * diameter_angle_scale_factor)),
              support_rests_on_model(mesh_group_settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE),
              xy_distance(mesh_group_settings.get<coord_t>("support_xy_distance")),
              bp_radius(mesh_group_settings.get<coord_t>("support_tree_bp_diameter") / 2),
              diameter_scale_bp_radius(std::min(sin(0.7) * layer_height / branch_radius, 1.0 / (branch_radius / (support_line_width / 2.0)))), // Either 40° or as much as possible so that 2 lines will overlap by at least 50%, whichever is smaller.
              support_overrides(mesh_group_settings.get<SupportDistPriority>("support_xy_overrides_z")),
              xy_min_distance(support_overrides == SupportDistPriority::Z_OVERRIDES_XY ? mesh_group_settings.get<coord_t>("support_xy_distance_overhang") : xy_distance),
              z_distance_top_layers(round_up_divide(mesh_group_settings.get<coord_t>("support_top_distance"), layer_height)),
              z_distance_bottom_layers(round_up_divide(mesh_group_settings.get<coord_t>("support_bottom_distance"), layer_height)),
              performance_interface_skip_layers(round_up_divide(mesh_group_settings.get<coord_t>("support_interface_skip_height"), layer_height)),
              support_infill_angles(mesh_group_settings.get<std::vector<AngleDegrees>>("support_infill_angles")),
              support_roof_angles(mesh_group_settings.get<std::vector<AngleDegrees>>("support_roof_angles")),
              roof_pattern(mesh_group_settings.get<EFillMethod>("support_roof_pattern")),
              support_pattern(mesh_group_settings.get<EFillMethod>("support_pattern")),
              support_roof_line_width(mesh_group_settings.get<coord_t>("support_roof_line_width")),
              support_line_distance(mesh_group_settings.get<coord_t>("support_line_distance")),
              support_bottom_offset(mesh_group_settings.get<coord_t>("support_bottom_offset")),
              minimum_bottom_area(mesh_group_settings.get<double>("minimum_bottom_area")),
              support_wall_count(mesh_group_settings.get<size_t>("support_wall_count")),
              maximum_deviation(mesh_group_settings.get<coord_t>("meshfix_maximum_deviation")),
              maximum_resolution(mesh_group_settings.get<coord_t>("meshfix_maximum_resolution"))
        {
            layer_start_bp_radius = (bp_radius - branch_radius) / (branch_radius * diameter_scale_bp_radius);

            // for performance reasons it is not possible to set a VERY low xy_distance or xy_min_distance (<0.1mm)
            // as such if such a value is set the support will be outset by .1mm to ensure all points are supported that should be.
            // This is not the best solution, but the only one to ensure areas can not lag though walls at high maximum_move_distance.
            if (xy_distance < 100)
            {
                xy_distance = 100;
            }
            if (xy_min_distance < 100)
            {
                xy_min_distance = 100;
                performance_increased_xy_min = true;
            }

            std::function<void(std::vector<AngleDegrees>&, EFillMethod)> getInterfaceAngles = [&](std::vector<AngleDegrees>& angles, EFillMethod pattern) { // (logic) from getInterfaceAngles in FFFGcodeWriter.
                if (angles.empty())
                {
                    if (pattern == EFillMethod::CONCENTRIC)
                    {
                        angles.push_back(0); // Concentric has no rotation.
                    }
                    else if (pattern == EFillMethod::TRIANGLES)
                    {
                        angles.push_back(90); // Triangular support interface shouldn't alternate every layer.
                    }
                    else
                    {
                        if (TreeSupportSettings::some_model_contains_thick_roof)
                        {
                            // Some roofs are quite thick.
                            // Alternate between the two kinds of diagonal: / and \ .
                            angles.push_back(45);
                            angles.push_back(135);
                        }
                        if (angles.empty())
                        {
                            angles.push_back(90); // Perpendicular to support lines.
                        }
                    }
                }
            };

            getInterfaceAngles(support_infill_angles, support_pattern);
            getInterfaceAngles(support_roof_angles, roof_pattern);
        }

      private:
        double angle;
        double angle_slow;

      public:
        static bool some_model_contains_thick_roof; // static variable, because TreeSupportConfig which needs this, will be used in ModelVolumes as this reduces redundancy.

        /*!
         * \brief Width of a single line of support.
         */
        coord_t support_line_width;
        /*!
         * \brief Height of a single layer
         */
        coord_t layer_height;
        /*!
         * \brief Radius of a branch when it has left the tip.
         */
        coord_t branch_radius;
        /*!
         * \brief smallest allowed radius, required to ensure that even at DTT 0 every circle will still be printed
         */
        coord_t min_radius;
        /*!
         * \brief How far an influence area may move outward every layer at most.
         */
        coord_t maximum_move_distance;
        /*!
         * \brief How far every influence area will move outward every layer if possible.
         */
        coord_t maximum_move_distance_slow;
        /*!
         * \brief Amount of bottom layers. 0 if disabled.
         */
        size_t support_bottom_layers;
        /*!
         * \brief Amount of effectiveDTT increases are required to reach branch radius.
         */
        size_t tip_layers;
        /*!
         * \brief Factor by which to increase the branch radius.
         */
        double diameter_angle_scale_factor;
        /*!
         * \brief How much a branch resting on the model may grow in radius by merging with branches that can reach the buildplate.
         */
        coord_t max_to_model_radius_increase;
        /*!
         * \brief If smaller (in layers) than that, all branches to model will be deleted
         */
        size_t min_dtt_to_model;
        /*!
         * \brief Increase radius in the resulting drawn branches, even if the avoidance does not allow it. Will be cut later to still fit.
         */
        coord_t increase_radius_until_radius;
        /*!
         * \brief Same as increase_radius_until_radius, but contains the DTT at which the radius will be reached.
         */
        size_t increase_radius_until_layer;
        /*!
         * \brief True if the branches may connect to the model.
         */
        bool support_rests_on_model;
        /*!
         * \brief How far should support be from the model.
         */
        coord_t xy_distance;
        /*!
         * \brief Radius a branch should have when reaching the buildplate.
         */
        coord_t bp_radius;
        /*!
         * \brief The layer index at which an increase in radius may be required to reach the bp_radius.
         */
        coord_t layer_start_bp_radius;
        /*!
         * \brief Factor by which to increase the branch radius to reach the required bp_radius at layer 0. Note that this radius increase will not happen in the tip, to ensure the tip is structurally sound.
         */
        double diameter_scale_bp_radius;
        /*!
         * \brief Should Z distance override X/Y distance, or the other way around.
         */
        SupportDistPriority support_overrides;
        /*!
         * \brief minimum xy_distance. Only relevant when Z overrides XY, otherwise equal to xy_distance-
         */
        coord_t xy_min_distance;
        /*!
         * \brief Amount of layers distance required the top of the support to the model
         */
        size_t z_distance_top_layers;
        /*!
         * \brief Amount of layers distance required from the top of the model to the bottom of a support structure.
         */
        size_t z_distance_bottom_layers;
        /*!
         * \brief used for performance optimization at the support floor. Should have no impact on the resulting tree.
         */
        size_t performance_interface_skip_layers;
        /*!
         * \brief Was the xy_min_dist increased by 100 microns ?
         */
        bool performance_increased_xy_min = false; // safeOffsetInc can only work in steps of the size xy_min_distance => has be larger than 0 and should be large enough for performance to not suffer extremely
        /*!
         * \brief User specified angles for the support infill.
         */
        std::vector<AngleDegrees> support_infill_angles;
        /*!
         * \brief User specified angles for the support roof infill.
         */
        std::vector<AngleDegrees> support_roof_angles;
        /*!
         * \brief Pattern used in the support roof. May contain non relevant data if support roof is disabled.
         */
        EFillMethod roof_pattern;
        /*!
         * \brief Pattern used in the support infill.
         */
        EFillMethod support_pattern;
        /*!
         * \brief Line width of the support roof.
         */
        coord_t support_roof_line_width;
        /*!
         * \brief Distance between support infill lines.
         */
        coord_t support_line_distance;
        /*!
         * \brief Offset applied to the support floor area.
         */
        coord_t support_bottom_offset;

        /*!
         * \brief Minimum area for support floor to be added.
         */
        coord_t minimum_bottom_area;
        /*
         * \brief Amount of walls the support area will have.
         */
        size_t support_wall_count;
        /*
         * \brief maximum allowed deviation when simplifying.
         */
        coord_t maximum_deviation;
        /*
         * \brief maximum allowed resolution (length of a line segment) when simplifying. The resolution is higher when this variable is smaller => Minimum size a line segment may have.
         */
        coord_t maximum_resolution;

      public:
        bool operator==(const TreeSupportSettings& other) const
        {
            return branch_radius == other.branch_radius && tip_layers == other.tip_layers && diameter_angle_scale_factor == other.diameter_angle_scale_factor && layer_start_bp_radius == other.layer_start_bp_radius && bp_radius == other.bp_radius && diameter_scale_bp_radius == other.diameter_scale_bp_radius && min_radius == other.min_radius && xy_min_distance == other.xy_min_distance && // as a recalculation of the collision areas is required to set a new min_radius.
                   xy_distance - xy_min_distance == other.xy_distance - other.xy_min_distance && // if the delta of xy_min_distance and xy_distance is different the collision areas have to be recalculated.
                   support_rests_on_model == other.support_rests_on_model && increase_radius_until_layer == other.increase_radius_until_layer && min_dtt_to_model == other.min_dtt_to_model && max_to_model_radius_increase == other.max_to_model_radius_increase && maximum_move_distance == other.maximum_move_distance && maximum_move_distance_slow == other.maximum_move_distance_slow && z_distance_bottom_layers == other.z_distance_bottom_layers && support_line_width == other.support_line_width && support_overrides == other.support_overrides && // requires new avoidance calculation. Could be changed so it does not, but because i expect that this case happens seldom i dont think it is worth it.
                   support_line_distance == other.support_line_distance && support_roof_line_width == other.support_roof_line_width && // can not be set on a per mesh basis currently, so code to enable processing different roof line width in the same iteration seems useless.
                   support_bottom_offset == other.support_bottom_offset && minimum_bottom_area == other.minimum_bottom_area && support_wall_count == other.support_wall_count && support_pattern == other.support_pattern && roof_pattern == other.roof_pattern && // can not be set on a per mesh basis currently, so code to enable processing different roof patterns in the same iteration seems useless.
                   support_roof_angles == other.support_roof_angles && support_infill_angles == other.support_infill_angles && performance_increased_xy_min == other.performance_increased_xy_min && increase_radius_until_radius == other.increase_radius_until_radius && support_bottom_layers == other.support_bottom_layers && layer_height == other.layer_height && z_distance_top_layers == other.z_distance_top_layers && maximum_deviation == other.maximum_deviation && // Infill generation depends on deviation and resolution.
                   maximum_resolution == other.maximum_resolution; // So they should be identical to ensure the tree will correctly support the roof.
        }


        /*!
         * \brief Get the Distance to top regarding the real radius this part will have. This is different from distance_to_top, which is can be used to calculate the top most layer of the branch.
         * \param elem[in] The SupportElement one wants to know the effectiveDTT
         * \return The Effective DTT.
         */
        inline size_t getEffectiveDTT(const TreeSupport::SupportElement& elem) const
        {
            return elem.effective_radius_height < increase_radius_until_layer ? (elem.distance_to_top < increase_radius_until_layer ? elem.distance_to_top : increase_radius_until_layer) : elem.effective_radius_height;
        }

        /*!
         * \brief Get the Radius part will have based on numeric values.
         * \param distance_to_top[in] The effective distance_to_top of the element
         * \param elephant_foot_increases[in] The elephant_foot_increases of the element.
         * \return The radius an element with these attributes would have.
         */
        inline coord_t getRadius(size_t distance_to_top, const double elephant_foot_increases = 0) const
        {
            return (distance_to_top <= tip_layers ? min_radius + (branch_radius - min_radius) * distance_to_top / tip_layers : // tip
                        branch_radius + // base
                            branch_radius * (distance_to_top - tip_layers) * diameter_angle_scale_factor)
                   + // gradual increase
                   branch_radius * elephant_foot_increases * (std::max(diameter_scale_bp_radius - diameter_angle_scale_factor, 0.0));
        }

        /*!
         * \brief Get the Radius, that this element will have.
         * \param elem[in] The Element.
         * \return The radius the element has.
         */
        inline coord_t getRadius(const TreeSupport::SupportElement& elem) const
        {
            return getRadius(getEffectiveDTT(elem), elem.elephant_foot_increases);
        }

        /*!
         * \brief Get the collision Radius of this Element. This can be smaller then the actual radius, as the drawAreas will cut off areas that may collide with the model.
         * \param elem[in] The Element.
         * \return The collision radius the element has.
         */
        inline coord_t getCollisionRadius(const TreeSupport::SupportElement& elem) const
        {
            return getRadius(elem.effective_radius_height, elem.elephant_foot_increases);
        }

        /*!
         * \brief Get the Radius an element should at least have at a given layer.
         * \param layer_idx[in] The layer.
         * \return The radius every element should aim to achieve.
         */
        inline coord_t recommendedMinRadius(LayerIndex layer_idx) const
        {
            double scale = (layer_start_bp_radius - layer_idx) * diameter_scale_bp_radius;
            return scale > 0 ? branch_radius + branch_radius * scale : 0;
        }
    };

  private:
    enum class LineStatus
    {
        INVALID,
        TO_MODEL,
        TO_MODEL_GRACIOUS,
        TO_MODEL_GRACIOUS_SAFE,
        TO_BP,
        TO_BP_SAFE
    };

    using LineInformation = std::vector<std::pair<Point, TreeSupport::LineStatus>>;


    /*!
     * \brief Precalculates all avoidances, that could be required.
     *
     * \param storage[in] Background storage to access meshes.
     * \param currently_processing_meshes[in] Indexes of all meshes that are processed in this iteration
     */
    void precalculate(const SliceDataStorage& storage, std::vector<size_t> currently_processing_meshes);
    /*!
     * \brief Converts a Polygons object representing a line into the internal format.
     *
     * \param polylines[in] The Polyline that will be converted.
     * \param layer_idx[in] The current layer.
     * \return All lines of the \p polylines object, with information for each point regarding in which avoidance it is currently valid in.
     */
    std::vector<LineInformation> convertLinesToInternal(Polygons polylines, LayerIndex layer_idx);
    /*!
     * \brief Converts lines in internal format into a Polygons object representing these lines.
     *
     * \param lines[in] The lines that will be converted.
     * \return All lines of the \p lines object as a Polygons object.
     */
    Polygons convertInternalToLines(std::vector<TreeSupport::LineInformation> lines);
    /*!
     * \brief Returns a function, evaluating if a point has to be added now. Required for a splitLines call in generateInitalAreas.
     *
     * \param current_layer[in] The layer on which the point lies
     * \return A function that can be called to evaluate a point.
     */
    std::function<bool(std::pair<Point, TreeSupport::LineStatus>)> getEvaluatePointForNextLayerFunction(size_t current_layer);
    /*!
     * \brief Evaluates which points of some lines are not valid one layer below and which are. Assumes all points are valid on the current layer. Validity is evaluated using supplied lambda.
     *
     * \param lines[in] The lines that have to be evaluated.
     * \param evaluatePoint[in] The function used to evaluate the points.
     * \return A pair with which points are still valid in the first slot and which are not in the second slot.
     */
    std::pair<std::vector<LineInformation>, std::vector<LineInformation>> splitLines(std::vector<LineInformation> lines, std::function<bool(std::pair<Point, TreeSupport::LineStatus>)> evaluatePoint); // assumes all Points on the current line are valid

    Polygons ensureMaximumDistancePolyline(const Polygons& input, coord_t distance, size_t min_points) const;
    /*!
     * \brief Adds the implicit line from the last vertex of a Polygon to the first one.
     *
     * \param poly[in] The Polygons object, of which its lines should be extended.
     * \return A Polygons object with implicit line from the last vertex of a Polygon to the first one added.
     */
    Polygons toPolylines(const Polygons& poly) const;


    /*!
     * \brief Returns Polylines representing the (infill) lines that will result in slicing the given area
     *
     * \param area[in] The area that has to be filled with infill.
     * \param roof[in] Whether the roofing or regular support settings should be used.
     * \param layer_idx[in] The current layer index.
     * \param support_infill_distance[in] The distance that should be between the infill lines.
     * \param cross_fill_provider[in] A SierpinskiFillProvider required for cross infill.
     *
     * \return A Polygons object that represents the resulting infill lines.
     */
    Polygons generateSupportInfillLines(const Polygons& area, bool roof, LayerIndex layer_idx, coord_t support_infill_distance, SierpinskiFillProvider* cross_fill_provider = nullptr);

    //todo comment
    Polygons safeUnion(const Polygons first, const Polygons second) const;
    Polygons safeUnion(const Polygons first) const;


    /*!
     * \brief Creates a valid CrossInfillProvider
     * Based on AreaSupport::precomputeCrossInfillTree, but calculates for each mesh separately
     * \param mesh[in] The mesh that is currently processed.
     * \param line_distance[in] The distance between the infill lines of the resulting infill
     * \param line_width[in] What is the width of a line used in the infill.
     * \return A valid CrossInfillProvider. Has to be freed manually to avoid a memory leak.
     */
    SierpinskiFillProvider* generateCrossFillProvider(const SliceMeshStorage& mesh, coord_t line_distance, coord_t line_width);


    /*!
     * \brief Creates the inital influence areas (that can later be propergated down) by placing them below the overhang.
     *
     * Generates Points where the Model should be supported and creates the areas where these points have to be placed.
     *
     * \param mesh[in] The mesh that is currently processed.
     * \param move_bounds[out] Storage for the influence areas.
     * \param storage[in] Background storage, required for adding roofs.
     */
    void generateInitalAreas(const SliceMeshStorage& mesh, std::vector<std::set<SupportElement*>>& move_bounds, SliceDataStorage& storage);

    /*!
     * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
     * \param me[in] Polygons object that has to be offset.
     * \param distance[in] The distance by which me should be offset. Expects values >=0.
     * \param collision[in] The area representing obstacles.
     * \param last_safe_step_size[in] The most it is allowed to offset in one step.
     * \param min_amount_offset[in] How many steps have to be done at least. As this uses round offset this increases the amount of vertices, which may be required if Polygons get very small. Required as arcTolerance is not exposed in offset, which should result with a similar result.
     * \return The resulting Polygons object.
     */
    Polygons safeOffsetInc(const Polygons& me, coord_t distance, const Polygons& collision, coord_t safe_step_size, coord_t last_safe_step_size, size_t min_amount_offset) const;


    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This helper merges all elements in input with the elements into reduced_new_layer.
     * Elements in input_aabb are merged together if possible, while elements reduced_new_layer_aabb are not checked against each other.
     *
     * \param reduced_aabb[in,out] The already processed elements.
     * \param input_aabb[in] Not yet processed elements
     * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate. Value is the influence area where the center of a circle of support may be placed.
     * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
     * Value is the influence area where the center of a circle of support may be placed.
     * \param influence_areas[in] The influence areas without avoidance removed.
     * \param insert_bp_areas[out] Elements to be inserted into the main dictionary after the Helper terminates.
     * \param insert_model_areas[out] Elements to be inserted into the secondary dictionary after the Helper terminates.
     * \param insert_influence[out] Elements to be inserted into the dictionary containing the largest possibly valid influence area (ignoring if the area may not be there because of avoidance)
     * \param erase[out] Elements that should be deleted from the above dictionaries.
     * \param layer_idx[in] The Index of the current Layer.
     */
    void mergeHelper(std::map<SupportElement, AABB>& reduced_aabb, std::map<SupportElement, AABB>& input_aabb, const std::unordered_map<SupportElement, Polygons>& to_bp_areas, const std::unordered_map<SupportElement, Polygons>& to_model_areas, const std::map<SupportElement, Polygons>& influence_areas, std::unordered_map<SupportElement, Polygons>& insert_bp_areas, std::unordered_map<SupportElement, Polygons>& insert_model_areas, std::unordered_map<SupportElement, Polygons>& insert_influence, std::vector<SupportElement>& erase, const LayerIndex layer_idx);
    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This manages the helper and uses a divide and conquer approach to parallelize this problem. This parallelization can at most accelerate the merging by a factor of 2.
     *
     * \param to_bp_areas[in] The Elements of the current Layer that will reach the buildplate.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param to_model_areas[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param influence_areas[in] The Elements of the current Layer without avoidances removed. This is the largest possible influence area for this layer.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param layer_idx[in] The current layer.
     */
    void mergeInfluenceAreas(std::unordered_map<SupportElement, Polygons>& to_bp_areas, std::unordered_map<SupportElement, Polygons>& to_model_areas, std::map<SupportElement, Polygons>& influence_areas, LayerIndex layer_idx);

    /*!
     * \brief Checks if an influence area contains a valid subsection and returns the corresponding metadata and the new Influence area.
     *
     * Calculates an influence areas of the layer below, based on the influence area of one element on the current layer.
     * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in we would change our gracious or to_buildplate status the influence areas are instead increased by maximum_move_distance_slow.
     * Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were the case, the radius is not increased instead.
     *
     * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was done to avoid not needed heap allocations.
     *
     * \param settings[in] Which settings have to be used to check validity.
     * \param layer_idx[in] Number of the current layer.
     * \param parent[in] The metadata of the parents influence area.
     * \param relevant_offset[in] The maximal possible influence area. Will be reduced in size if required.
     * \param to_bp_data[out] The part of the Influence area that can reach the buildplate
     * \param to_model_data[out] The part of the Influence area that do not have to reach the buildplate. This has overlap with new_layer_data.
     * \param increased[out]  Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
     * \param overspeed[in] How much should the already offset area be offset again. Usually this is 0.
     * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
     */
    std::optional<TreeSupport::SupportElement> increaseSingleArea(AreaIncreaseSettings settings, LayerIndex layer_idx, SupportElement* parent, const Polygons& relevant_offset, Polygons& to_bp_data, Polygons& to_model_data, Polygons& increased, const coord_t overspeed, const bool mergelayer);
    /*!
     * \brief Increases influence areas as far as required.
     *
     * Calculates influence areas of the layer below, based on the influence areas of the current layer.
     * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in it would change the gracious or to_buildplate status, the influence areas are instead increased by maximum_move_distance.
     * Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were the case, the radius is not increased instead.
     *
     * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was done to avoid not needed heap allocations.
     *
     * \param to_bp_areas[out] Influence areas that can reach the buildplate
     * \param to_model_areas[out] Influence areas that do not have to reach the buildplate. This has overlap with new_layer_data, as areas that can reach the buildplate are also considered valid areas to the model.
     * This redundency is required if a to_buildplate influence area is allowed to merge with a to model influence area.
     * \param influence_areas[out] Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
     * \param bypass_merge_areas[out] Influence areas ready to be added to the layer below that do not need merging.
     * \param last_layer[in] Influence areas of the current layer.
     * \param layer_idx[in] Number of the current layer.
     * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
     */
    void increaseAreas(std::unordered_map<SupportElement, Polygons>& to_bp_areas, std::unordered_map<SupportElement, Polygons>& to_model_areas, std::map<SupportElement, Polygons>& influence_areas, std::vector<SupportElement*>& bypass_merge_areas, const std::vector<SupportElement*>& last_layer, const LayerIndex layer_idx, const bool mergelayer);

    /*!
     * \brief Propergates influence downwards, and merges overlapping ones.
     *
     * \param move_bounds[in,out] All currently existing influence areas
     */
    void createLayerPathing(std::vector<std::set<SupportElement*>>& move_bounds);


    /*!
     * \brief Sets the result_on_layer for all parents based on the SupportElement supplied.
     *
     * \param elem[in] The SupportElements, which parent's position should be determined.
     */
    void setPointsOnAreas(const SupportElement* elem);
    /*!
     * \brief Get the best point to connect to the model and set the result_on_layer of the relevant SupportElement accordingly.
     *
     * \param move_bounds[in,out] All currently existing influence areas
     * \param first_elem[in,out] SupportElement that did not have its result_on_layer set meaning that it does not have a child element.
     * \param layer_idx[in] The current layer.
     * \return Should elem be deleted.
     */
    bool setToModelContact(std::vector<std::set<SupportElement*>>& move_bounds, SupportElement* first_elem, const LayerIndex layer_idx);

    /*!
     * \brief Set the result_on_layer point for all influence areas
     *
     * \param move_bounds[in,out] All currently existing influence areas
     */
    void createNodesFromArea(std::vector<std::set<SupportElement*>>& move_bounds);
    /*!
     * \brief Draws circles around resul_on_layer points of the influence areas
     *
     * \param move_bounds[in] All currently existing influence areas
     * \param storage[in,out] The storage where the support should be stored.
     */
    void drawAreas(std::vector<std::set<SupportElement*>>& move_bounds, SliceDataStorage& storage);

    std::vector<std::pair<TreeSupportSettings, std::vector<size_t>>> grouped_meshes;
    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes
     *
     */
    ModelVolumes volumes_;
    /*!
     * \brief Contains config settings to avoid loading them in every function. This was done to improve readability of the code.
     */
    TreeSupportSettings config;

    /*!
     * \brief The progress multiplier of all values added progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_multiplier = 1;
    /*!
     * \brief The progress offset added to all values communitcated to the progress bar.
     * Required for the progress bar the behave as expected when areas have to be calculated multiple times
     */
    double progress_offset = 0;
};


} // namespace cura

namespace std
{
template <>
struct hash<cura::TreeSupport::SupportElement>
{
    size_t operator()(const cura::TreeSupport::SupportElement& node) const
    {
        return hash<cura::Point>()(node.target_position) + node.target_height;
    }
};
} // namespace std

#endif /* TREESUPPORT_H */
