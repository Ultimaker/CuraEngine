//CuraEngine is released under the terms of the AGPLv3 or higher.
#ifndef TREESUPPORTELEMENT_H
#define TREESUPPORTELEMENT_H

#include "TreeModelVolumes.h"
#include "TreeSupportBaseCircle.h"
#include "TreeSupportEnums.h"
#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"
#include "utils/polygon.h"
#include <boost/container_hash/hash.hpp>

#include <map>
#include <unordered_map>

namespace cura
{

struct AreaIncreaseSettings
{
    AreaIncreaseSettings() :
        type(AvoidanceType::FAST),
        increase_speed(0),
        increase_radius(false),
        no_error(false),
        use_min_distance(false),
        move(false)
    {
    }

    AreaIncreaseSettings
    (
        AvoidanceType type,
        coord_t increase_speed,
        bool increase_radius,
        bool simplify,
        bool use_min_distance,
        bool move
    ) :
        type(type),
        increase_speed(increase_speed),
        increase_radius(increase_radius),
        no_error(simplify),
        use_min_distance(use_min_distance),
        move(move)
    {
    }

    AvoidanceType type;
    coord_t increase_speed;
    bool increase_radius;
    bool no_error;
    bool use_min_distance;
    bool move;

    bool operator==(const AreaIncreaseSettings& other) const
    {
        return
            increase_radius == other.increase_radius &&
            increase_speed == other.increase_speed &&
            type == other.type &&
            no_error == other.no_error &&
            use_min_distance == other.use_min_distance &&
            move == other.move;
    }
};

struct TreeSupportElement
{
    TreeSupportElement
    (
        coord_t distance_to_top,
        size_t target_height,
        Point target_position,
        bool to_buildplate,
        bool to_model_gracious,
        bool use_min_xy_dist,
        size_t dont_move_until,
        bool supports_roof,
        bool can_use_safe_radius,
        bool force_tips_to_roof,
        bool skip_ovalisation,
        bool influence_area_limit_active,
        coord_t influence_area_limit_range
    ) :
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
        buildplate_radius_increases(0),
        use_min_xy_dist(use_min_xy_dist),
        supports_roof(supports_roof),
        dont_move_until(dont_move_until),
        can_use_safe_radius(can_use_safe_radius),
        last_area_increase(AreaIncreaseSettings(AvoidanceType::FAST, 0, false, false, false, false)),
        missing_roof_layers(force_tips_to_roof ? dont_move_until : 0),
        skip_ovalisation(skip_ovalisation),
        all_tips({ target_position }),
        influence_area_limit_active(influence_area_limit_active),
        influence_area_limit_range(influence_area_limit_range
    )
    {
        RecreateInfluenceLimitArea();
    }

    TreeSupportElement(const TreeSupportElement& elem, Polygons* newArea = nullptr) : // copy constructor with possibility to set a new area
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
        buildplate_radius_increases(elem.buildplate_radius_increases),
        use_min_xy_dist(elem.use_min_xy_dist),
        supports_roof(elem.supports_roof),
        dont_move_until(elem.dont_move_until),
        can_use_safe_radius(elem.can_use_safe_radius),
        last_area_increase(elem.last_area_increase),
        missing_roof_layers(elem.missing_roof_layers),
        skip_ovalisation(elem.skip_ovalisation),
        all_tips(elem.all_tips),
        influence_area_limit_area(elem.influence_area_limit_area),
        influence_area_limit_range(elem.influence_area_limit_range),
        influence_area_limit_active(elem.influence_area_limit_active)
    {
        parents.insert(parents.begin(), elem.parents.begin(), elem.parents.end());
    }

    /*!
     * \brief Create a new Element for one layer below the element of the pointer supplied.
     */
    TreeSupportElement(TreeSupportElement* element_above) :
        target_height(element_above->target_height),
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
        buildplate_radius_increases(element_above->buildplate_radius_increases),
        use_min_xy_dist(element_above->use_min_xy_dist),
        supports_roof(element_above->supports_roof),
        dont_move_until(element_above->dont_move_until),
        can_use_safe_radius(element_above->can_use_safe_radius),
        last_area_increase(element_above->last_area_increase),
        missing_roof_layers(element_above->missing_roof_layers),
        skip_ovalisation(false),
        all_tips(element_above->all_tips),
        influence_area_limit_area(element_above->influence_area_limit_area),
        influence_area_limit_range(element_above->influence_area_limit_range),
        influence_area_limit_active(element_above->influence_area_limit_active)
    {
        parents = { element_above };
    }

    // ONLY to be called in merge as it assumes a few assurances made by it.
    TreeSupportElement
    (
        const TreeSupportElement& first,
        const TreeSupportElement& second,
        size_t next_height,
        Point next_position,
        coord_t increased_to_model_radius,
        const std::function<coord_t(size_t, double)>& getRadius,
        double diameter_scale_bp_radius,
        coord_t branch_radius,
        double diameter_angle_scale_factor
    ) :
        next_position(next_position),
        next_height(next_height),
        area(nullptr),
        increased_to_model_radius(increased_to_model_radius),
        use_min_xy_dist(first.use_min_xy_dist || second.use_min_xy_dist),
        supports_roof(first.supports_roof || second.supports_roof),
        dont_move_until(std::max(first.dont_move_until, second.dont_move_until)),
        can_use_safe_radius(first.can_use_safe_radius || second.can_use_safe_radius),
        missing_roof_layers(std::min(first.missing_roof_layers, second.missing_roof_layers)),
        skip_ovalisation(false)
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
        to_model_gracious = first.to_model_gracious && second.to_model_gracious; // valid as we do not merge non-gracious with gracious

        AddParents(first.parents);
        AddParents(second.parents);

        buildplate_radius_increases = 0;
        if (diameter_scale_bp_radius > 0)
        {
            const coord_t foot_increase_radius =
                std::abs
                (
                    std::max
                    (
                        getRadius(second.effective_radius_height, second.buildplate_radius_increases),
                        getRadius(first.effective_radius_height, first.buildplate_radius_increases)) - getRadius(effective_radius_height, buildplate_radius_increases
                    )
                );
            // 'buildplate_radius_increases' has to be recalculated, as when a smaller tree with a larger buildplate_radius_increases merge with a larger branch,
            //   the buildplate_radius_increases may have to be lower as otherwise the radius suddenly increases. This results often in a non integer value.
            buildplate_radius_increases = foot_increase_radius / (branch_radius * (diameter_scale_bp_radius - diameter_angle_scale_factor));
        }

        // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
        last_area_increase =
            AreaIncreaseSettings
            (
                std::min(first.last_area_increase.type, second.last_area_increase.type),
                std::min(first.last_area_increase.increase_speed, second.last_area_increase.increase_speed),
                first.last_area_increase.increase_radius || second.last_area_increase.increase_radius,
                first.last_area_increase.no_error || second.last_area_increase.no_error,
                first.last_area_increase.use_min_distance && second.last_area_increase.use_min_distance,
                first.last_area_increase.move || second.last_area_increase.move
            );

        all_tips = first.all_tips;
        all_tips.insert(all_tips.end(), second.all_tips.begin(), second.all_tips.end());
        influence_area_limit_range = std::max(first.influence_area_limit_range, second.influence_area_limit_range);
        influence_area_limit_active = first.influence_area_limit_active || second.influence_area_limit_active;
        RecreateInfluenceLimitArea();
        if(first.to_buildplate != second.to_buildplate)
        {
            setToBuildplateForAllParents(to_buildplate);
        }
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
    std::vector<TreeSupportElement*> parents;

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
     * \brief Will the branch be able to rest completely on a flat surface, be it buildplate or model ?
     */
    bool to_model_gracious;

    /*!
     * \brief Counter about the times the radius was increased to reach the bp_radius. Can be fractions for merge reasons.
     */
    double buildplate_radius_increases;

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

    /*!
     * \brief Amount of roof layers that were not yet added, because the branch needed to move.
     */
    size_t missing_roof_layers;

    /*!
     * \brief Skip the ovalisation to parent and children when generating the final circles.
     */
    bool skip_ovalisation;

    /*!
     * \brief The coordinates of all tips supported by this branch.
     */
    std::vector<Point> all_tips;

    /*!
     * \brief Whether the range of an influence area is limited
     */
    bool influence_area_limit_active;

    /*!
     * \brief Maximum distance (x/y) the influence area should be from each tip.
     */
    coord_t influence_area_limit_range;

    /*!
     * \brief Area that influence area has to be inside to conform to influence_area_limit_range.
     */
    Polygons influence_area_limit_area;

    /*!
     * \brief Additional locations that the tip should reach
     */
    std::vector<Point> additional_ovalization_targets;


    bool operator==(const TreeSupportElement& other) const
    {
        return target_position == other.target_position && target_height == other.target_height;
    }

    bool operator<(const TreeSupportElement& other) const // true if me < other
    {
        return !(*this == other) && !(*this > other);
    }

    bool operator>(const TreeSupportElement& other) const
    {
        // Doesn't really have to make sense, only required for ordering in maps to ensure deterministic behavior.
        if (*this == other)
        {
            return false;
        }
        if (other.target_height != target_height)
        {
            return other.target_height < target_height;
        }
        return other.target_position.X == target_position.X ? other.target_position.Y < target_position.Y : other.target_position.X < target_position.X;
    }

    void AddParents(const std::vector<TreeSupportElement*>& adding)
    {
        for (TreeSupportElement* ptr : adding)
        {
            parents.emplace_back(ptr);
        }
    }

    void RecreateInfluenceLimitArea()
    {
        if (influence_area_limit_active)
        {
            influence_area_limit_area.clear();

            for (Point p : all_tips)
            {
                Polygon circle;
                for (Point corner : TreeSupportBaseCircle::getBaseCircle())
                {
                    circle.add(p + corner * influence_area_limit_range / double(TreeSupportBaseCircle::base_radius));
                }
                if (influence_area_limit_area.empty())
                {
                    influence_area_limit_area = circle.offset(0);
                }
                else
                {
                    influence_area_limit_area = influence_area_limit_area.intersection(circle.offset(0));
                }
            }
        }
    }
    void setToBuildplateForAllParents(bool new_value)
    {
        to_buildplate = new_value;
        std::vector<TreeSupportElement*> grandparents {parents};
        while (!grandparents.empty()){
            std::vector<TreeSupportElement*> next_parents;
            for (TreeSupportElement* grandparent:grandparents){
                next_parents.insert(next_parents.end(),grandparent->parents.begin(),grandparent->parents.end());
                grandparent->to_buildplate = new_value;
            }
            grandparents = next_parents;
        }
    }
    
    inline bool isResultOnLayerSet() const
    {
        return result_on_layer != Point(-1, -1);
    }

};

} // namespace cura

namespace std
{
template <>
struct hash<cura::TreeSupportElement>
{
    size_t operator()(const cura::TreeSupportElement& node) const
    {
        size_t hash_node = hash<cura::Point>()(node.target_position);
        boost::hash_combine(hash_node, size_t(node.target_height));
        return hash_node;
    }
};
} // namespace std
#endif /* TREESUPPORTELEMENT_H */
