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

struct CradlePresenceInformation;
struct AreaIncreaseSettings
{
    AreaIncreaseSettings()
        : type_(AvoidanceType::FAST)
        , increase_speed_(0)
        , increase_radius_(false)
        , no_error_(false)
        , use_min_distance_(false)
        , use_anti_preferred_(false)
        , move_(false)
    {
    }

    AreaIncreaseSettings(AvoidanceType type, coord_t increase_speed, bool increase_radius, bool simplify, bool use_min_distance, bool use_anti_preferred, bool move)
        : type_(type)
        , increase_speed_(increase_speed)
        , increase_radius_(increase_radius)
        , no_error_(simplify)
        , use_min_distance_(use_min_distance)
        , use_anti_preferred_(use_anti_preferred)
        , move_(move)
    {
    }

    AvoidanceType type_;
    coord_t increase_speed_;
    bool increase_radius_;
    bool no_error_;
    bool use_min_distance_;
    bool use_anti_preferred_;
    bool move_;

    bool operator==(const AreaIncreaseSettings& other) const
    {
        return increase_radius_ == other.increase_radius_ && increase_speed_ == other.increase_speed_ && type_ == other.type_ && no_error_ == other.no_error_
            && use_min_distance_ == other.use_min_distance_ && move_ == other.move_;
    }
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
        bool supports_cradle,
        bool can_use_safe_radius,
        bool force_tips_to_roof,
        bool skip_ovalisation,
        bool influence_area_limit_active,
        coord_t influence_area_limit_range,
        double hidden_radius_increase)
        : target_height_(target_height)
        , target_position_(target_position)
        , next_position_(target_position)
        , next_height_(target_height)
        , effective_radius_height_(0)
        , to_buildplate_(to_buildplate)
        , distance_to_top_(distance_to_top)
        , area_(nullptr)
        , result_on_layer_(target_position)
        , increased_to_model_radius_(0)
        , to_model_gracious_(to_model_gracious)
        , buildplate_radius_increases_(0)
        , use_min_xy_dist_(use_min_xy_dist)
        , supports_roof_(supports_roof)
        , supports_cradle_(supports_cradle)
        , dont_move_until_(dont_move_until)
        , can_use_safe_radius_(can_use_safe_radius)
        , can_avoid_anti_preferred_(false)
        , ensure_valid_anti_preferred_(false)
        , last_area_increase_(AreaIncreaseSettings(AvoidanceType::FAST, 0, false, false, false, false, false))
        , missing_roof_layers_(force_tips_to_roof ? dont_move_until : 0)
        , roof_with_enforced_walls(false)
        , skip_ovalisation_(skip_ovalisation)
        , all_tips_({ target_position })
        , influence_area_limit_active_(influence_area_limit_active)
        , influence_area_limit_range_(influence_area_limit_range)
        , hidden_radius_increase_(hidden_radius_increase)
    {
        RecreateInfluenceLimitArea();
    }

    TreeSupportElement(const TreeSupportElement& elem, Shape* new_area)
        : // copy constructor that sets a new area
        TreeSupportElement(elem)
    {
        area_ = new_area;
        additional_ovalization_targets_.clear();
        cradle_line_ = nullptr;
    }


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
        double diameter_angle_scale_factor)
        : next_position_(next_position)
        , next_height_(next_height)
        , area_(nullptr)
        , increased_to_model_radius_(increased_to_model_radius)
        , use_min_xy_dist_(first.use_min_xy_dist_ || second.use_min_xy_dist_)
        , supports_roof_(first.supports_roof_ || second.supports_roof_)
        , supports_cradle_(first.supports_cradle_ || second.supports_cradle_)
        , dont_move_until_(std::max(first.dont_move_until_, second.dont_move_until_))
        , can_use_safe_radius_(first.can_use_safe_radius_ || second.can_use_safe_radius_)
        , can_avoid_anti_preferred_(first.can_avoid_anti_preferred_ || second.can_avoid_anti_preferred_)
        , ensure_valid_anti_preferred_(first.ensure_valid_anti_preferred_ || second.ensure_valid_anti_preferred_)
        , missing_roof_layers_(std::min(first.missing_roof_layers_, second.missing_roof_layers_))
        , roof_with_enforced_walls(first.roof_with_enforced_walls && second.roof_with_enforced_walls)
        , skip_ovalisation_(false)
    {
        if (first.target_height_ > second.target_height_)
        {
            target_height_ = first.target_height_;
            target_position_ = first.target_position_;
        }
        else
        {
            target_height_ = second.target_height_;
            target_position_ = second.target_position_;
        }
        effective_radius_height_ = std::max(first.effective_radius_height_, second.effective_radius_height_);
        distance_to_top_ = std::max(first.distance_to_top_, second.distance_to_top_);

        to_buildplate_ = first.to_buildplate_ && second.to_buildplate_;
        to_model_gracious_ = first.to_model_gracious_ && second.to_model_gracious_; // valid as we do not merge non-gracious with gracious

        AddParents(first.parents_);
        AddParents(second.parents_);

        buildplate_radius_increases_ = 0;
        if (diameter_scale_bp_radius > 0)
        {
            const coord_t foot_increase_radius = std::abs(
                std::max(
                    getRadius(second.effective_radius_height_, second.buildplate_radius_increases_),
                    getRadius(first.effective_radius_height_, first.buildplate_radius_increases_))
                - getRadius(effective_radius_height_, buildplate_radius_increases_));
            // 'buildplate_radius_increases' has to be recalculated, as when a smaller tree with a larger buildplate_radius_increases merge with a larger branch,
            //   the buildplate_radius_increases may have to be lower as otherwise the radius suddenly increases. This results often in a non integer value.
            buildplate_radius_increases_ = foot_increase_radius / (branch_radius * (diameter_scale_bp_radius - diameter_angle_scale_factor));

            const coord_t hidden_increase_radius = std::abs(
                std::max(
                    getRadius(second.effective_radius_height_, second.buildplate_radius_increases_ + second.hidden_radius_increase_),
                    getRadius(first.effective_radius_height_, first.buildplate_radius_increases_ + first.hidden_radius_increase_))
                - getRadius(effective_radius_height_, buildplate_radius_increases_));
            hidden_radius_increase_ = hidden_increase_radius / (branch_radius * (diameter_scale_bp_radius - diameter_angle_scale_factor));
        }

        // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
        last_area_increase_ = AreaIncreaseSettings(
            std::min(first.last_area_increase_.type_, second.last_area_increase_.type_),
            std::min(first.last_area_increase_.increase_speed_, second.last_area_increase_.increase_speed_),
            first.last_area_increase_.increase_radius_ || second.last_area_increase_.increase_radius_,
            first.last_area_increase_.no_error_ || second.last_area_increase_.no_error_,
            first.last_area_increase_.use_min_distance_ && second.last_area_increase_.use_min_distance_,
            first.can_avoid_anti_preferred_ && second.can_avoid_anti_preferred_,
            first.last_area_increase_.move_ || second.last_area_increase_.move_);

        all_tips_ = first.all_tips_;
        all_tips_.insert(all_tips_.end(), second.all_tips_.begin(), second.all_tips_.end());
        influence_area_limit_range_ = std::max(first.influence_area_limit_range_, second.influence_area_limit_range_);
        influence_area_limit_active_ = first.influence_area_limit_active_ || second.influence_area_limit_active_;
        RecreateInfluenceLimitArea();
        if (first.to_buildplate_ != second.to_buildplate_)
        {
            setToBuildplateForAllParents(to_buildplate_);
        }
    }

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
     * \brief True if this Element or any parent provides support to a cradle or cradle line.
     */
    bool supports_cradle_;

    /*!
     * \brief The element trys not to move until this dtt is reached, is set to 0 if the element had to move.
     */
    size_t dont_move_until_;

    /*!
     * \brief An influence area is considered safe when it can use the holefree avoidance <=> It will not have to encounter holes on its way downward.
     */
    bool can_use_safe_radius_;

    /*!
     * \brief An influence area can avoid anti-preferred when the difference with it is non empty.
     */
    bool can_avoid_anti_preferred_;

    /*!
     * \brief If the influence area ensures no collision with anti preferred on this layer.
     */
    bool ensure_valid_anti_preferred_;

    /*!
     * \brief Settings used to increase the influence area to its current state.
     */
    AreaIncreaseSettings last_area_increase_;

    /*!
     * \brief Amount of roof layers that were not yet added, because the branch needed to move.
     */
    size_t missing_roof_layers_;

    /*!
     * \brief True if interface with walls has to be used, even though regular interface does not have walls.
     */
    bool roof_with_enforced_walls;

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

    /*!
     * \brief Pointer to the cradle line it supports if it does support a cradle line.
     */
    std::shared_ptr<CradlePresenceInformation> cradle_line_;

    /*!
     * \brief Counter about the times the radius was increased to reach the correct initial radius. Uses logic intended for buildplate_radius_increases_
     */
    double hidden_radius_increase_;


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

    void AddParents(const std::vector<TreeSupportElement*>& adding)
    {
        for (TreeSupportElement* ptr : adding)
        {
            parents_.emplace_back(ptr);
        }
    }

    void RecreateInfluenceLimitArea()
    {
        if (influence_area_limit_active_)
        {
            influence_area_limit_area_.clear();

            for (Point2LL p : all_tips_)
            {
                Polygon circle;
                for (Point2LL corner : TreeSupportBaseCircle::getBaseCircle())
                {
                    circle.push_back(p + corner * influence_area_limit_range_ / double(TreeSupportBaseCircle::base_radius));
                }
                if (influence_area_limit_area_.empty())
                {
                    influence_area_limit_area_ = circle.offset(0);
                }
                else
                {
                    influence_area_limit_area_ = influence_area_limit_area_.intersection(circle.offset(0));
                }
            }
        }
    }

    void setToBuildplateForAllParents(bool new_value)
    {
        to_buildplate_ = new_value;
        to_model_gracious_ |= new_value;
        std::vector<TreeSupportElement*> grandparents{ parents_ };
        while (! grandparents.empty())
        {
            std::vector<TreeSupportElement*> next_parents;
            for (TreeSupportElement* grandparent : grandparents)
            {
                next_parents.insert(next_parents.end(), grandparent->parents_.begin(), grandparent->parents_.end());
                grandparent->to_buildplate_ = new_value;
                grandparent->to_model_gracious_ |= new_value; // If we set to_buildplate to true, update to_model_gracious
            }
            grandparents = next_parents;
        }
    }

    inline bool isResultOnLayerSet() const
    {
        return result_on_layer_ != Point2LL(-1, -1);
    }
    /*!
     * \brief Create a new Element for one layer below the element.
     */

    TreeSupportElement createNewElement()
    {
        TreeSupportElement result(*this);
        result.parents_ = { this };
        result.distance_to_top_ += 1;
        result.skip_ovalisation_ = false;
        result.result_on_layer_ = Point2LL(-1, -1);
        result.area_ = nullptr;
        result.ensure_valid_anti_preferred_ = false;
        return result;
    }
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
