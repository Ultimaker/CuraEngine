// CuraEngine is released under the terms of the AGPLv3 or higher.
#ifndef TREESUPPORTELEMENT_H
#define TREESUPPORTELEMENT_H

#include <map>
#include <unordered_map>

#include <boost/container_hash/hash.hpp>

#include "TreeModelVolumes.h"
#include "TreeSupportBaseCircle.h"
#include "TreeSupportEnums.h"
#include "geometry/Shape.h"
#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"

namespace cura
{

class OBJ;

struct AreaIncreaseSettings
{
    AreaIncreaseSettings() = default;

    AreaIncreaseSettings(const AvoidanceType type, const coord_t increase_speed, const bool increase_radius, const bool simplify, const bool use_min_distance, const bool move)
        : type_(type)
        , increase_speed_(increase_speed)
        , increase_radius_(increase_radius)
        , no_error_(simplify)
        , use_min_distance_(use_min_distance)
        , move_(move)
    {
    }

    AvoidanceType type_{ AvoidanceType::FAST };
    coord_t increase_speed_{ 0 };
    bool increase_radius_{ false };
    bool no_error_{ false };
    bool use_min_distance_{ false };
    bool move_{ false };

    bool operator==(const AreaIncreaseSettings& other) const = default;
};

struct TreeSupportElement
{
    TreeSupportElement(
        coord_t distance_to_top,
        size_t target_height,
        Point2LL target_position,
        bool to_buildplate,
        bool to_model_gracious,
        bool use_min_xy_dist,
        size_t dont_move_until,
        bool supports_roof,
        bool can_use_safe_radius,
        bool force_tips_to_roof,
        bool skip_ovalisation,
        bool influence_area_limit_active,
        coord_t influence_area_limit_range);

    TreeSupportElement(const TreeSupportElement& elem, Shape* newArea = nullptr);

    /*!
     * \brief Create a new Element for one layer below the element of the pointer supplied.
     */
    TreeSupportElement(TreeSupportElement* element_above);

    // ONLY to be called in merge as it assumes a few assurances made by it.
    TreeSupportElement(
        const TreeSupportElement& first,
        const TreeSupportElement& second,
        size_t next_height,
        Point2LL next_position,
        coord_t increased_to_model_radius,
        const std::function<coord_t(size_t, double)>& getRadius,
        double diameter_scale_bp_radius,
        coord_t branch_radius,
        double diameter_angle_scale_factor);


    bool operator==(const TreeSupportElement& other) const
    {
        return target_position_ == other.target_position_ && target_height_ == other.target_height_;
    }

    bool operator<(const TreeSupportElement& other) const // true if me < other
    {
        return ! (*this == other) && ! (*this > other);
    }

    bool operator>(const TreeSupportElement& other) const
    {
        // Doesn't really have to make sense, only required for ordering in maps to ensure deterministic behavior.
        if (*this == other)
        {
            return false;
        }
        if (other.target_height_ != target_height_)
        {
            return other.target_height_ < target_height_;
        }
        return other.target_position_.X == target_position_.X ? other.target_position_.Y < target_position_.Y : other.target_position_.X < target_position_.X;
    }

    void AddParents(const std::vector<TreeSupportElement*>& adding);

    void RecreateInfluenceLimitArea();

    void setToBuildplateForAllParents(bool new_value);

    inline bool isResultOnLayerSet() const
    {
        return result_on_layer_ != Point2LL(-1, -1);
    }

    /*!
     * Saves the influence areas to a 3D object
     * @param obj The object in which to save the influence area
     * @param z The Z at which the area is placed
     * @param layer_height The layer height at which to extrude the influence area
     */
    void saveToObj(OBJ& obj, const coord_t z, const coord_t layer_height) const;

    /*!
     * \brief The layer this support elements wants reach
     */
    LayerIndex target_height_;

    /*!
     * \brief The position this support elements wants to support on layer=target_height
     */
    Point2LL target_position_;

    /*!
     * \brief The next position this support elements wants to reach. NOTE: This is mainly a suggestion regarding direction inside the influence area.
     */
    Point2LL next_position_;


    /*!
     * \brief The next height this support elements wants to reach
     */
    LayerIndex next_height_;

    /*!
     * \brief The Effective distance to top of this element regarding radius increases and collision calculations.
     */

    size_t effective_radius_height_;

    /*!
     * \brief The element trys to reach the buildplate
     */

    bool to_buildplate_;

    /*!
     * \brief All elements in the layer above the current one that are supported by this element
     */
    std::vector<TreeSupportElement*> parents_;

    /*!
     * \brief The amount of layers this element is below the topmost layer of this branch.
     */
    size_t distance_to_top_;

    /*!
     * \brief The resulting influence area.
     * Will only be set in the results of createLayerPathing, and will be nullptr inside!
     */
    Shape* area_;

    /*!
     * \brief The resulting center point around which a circle will be drawn later.
     * Will be set by setPointsOnAreas
     */
    Point2LL result_on_layer_ = Point2LL(-1, -1);
    /*!
     * \brief The amount of extra radius we got from merging branches that could have reached the buildplate, but merged with ones that can not.
     */
    coord_t increased_to_model_radius_; // how much to model we increased only relevant for merging
    /*!
     * \brief Will the branch be able to rest completely on a flat surface, be it buildplate or model ?
     */
    bool to_model_gracious_;

    /*!
     * \brief Counter about the times the radius was increased to reach the bp_radius. Can be fractions for merge reasons.
     */
    double buildplate_radius_increases_;

    /*!
     * \brief Whether the min_xy_distance can be used to get avoidance or similar. Will only be true if support_xy_overrides_z=Z overrides X/Y.
     */
    bool use_min_xy_dist_;

    /*!
     * \brief True if this Element or any parent provides support to a support roof.
     */
    bool supports_roof_;

    /*!
     * \brief The element trys not to move until this dtt is reached, is set to 0 if the element had to move.
     */
    size_t dont_move_until_;

    /*!
     * \brief An influence area is considered safe when it can use the holefree avoidance <=> It will not have to encounter holes on its way downward.
     */
    bool can_use_safe_radius_;

    /*!
     * \brief Settings used to increase the influence area to its current state.
     */
    AreaIncreaseSettings last_area_increase_;

    /*!
     * \brief Amount of roof layers that were not yet added, because the branch needed to move.
     */
    size_t missing_roof_layers_;

    /*!
     * \brief Skip the ovalisation to parent and children when generating the final circles.
     */
    bool skip_ovalisation_;

    /*!
     * \brief The coordinates of all tips supported by this branch.
     */
    std::vector<Point2LL> all_tips_;

    /*!
     * \brief Whether the range of an influence area is limited
     */
    bool influence_area_limit_active_;

    /*!
     * \brief Maximum distance (x/y) the influence area should be from each tip.
     */
    coord_t influence_area_limit_range_;

    /*!
     * \brief Area that influence area has to be inside to conform to influence_area_limit_range.
     */
    Shape influence_area_limit_area_;

    /*!
     * \brief Additional locations that the tip should reach
     */
    std::vector<Point2LL> additional_ovalization_targets_;
};

} // namespace cura

namespace std
{
template<>
struct hash<cura::TreeSupportElement>
{
    size_t operator()(const cura::TreeSupportElement& node) const
    {
        size_t hash_node = hash<cura::Point2LL>()(node.target_position_);
        boost::hash_combine(hash_node, size_t(node.target_height_));
        return hash_node;
    }
};
} // namespace std
#endif /* TREESUPPORTELEMENT_H */
