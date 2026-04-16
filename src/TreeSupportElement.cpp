// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "TreeSupportElement.h"

#include "utils/OBJ.h"


namespace cura
{

TreeSupportElement::TreeSupportElement(
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
    coord_t influence_area_limit_range)
    : target_height_(target_height)
    , target_position_(target_position)
    , next_position_(target_position)
    , next_height_(target_height)
    , effective_radius_height_(distance_to_top)
    , to_buildplate_(to_buildplate)
    , distance_to_top_(distance_to_top)
    , area_(nullptr)
    , result_on_layer_(target_position)
    , increased_to_model_radius_(0)
    , to_model_gracious_(to_model_gracious)
    , buildplate_radius_increases_(0)
    , use_min_xy_dist_(use_min_xy_dist)
    , supports_roof_(supports_roof)
    , dont_move_until_(dont_move_until)
    , can_use_safe_radius_(can_use_safe_radius)
    , last_area_increase_(AreaIncreaseSettings(AvoidanceType::FAST, 0, false, false, false, false))
    , missing_roof_layers_(force_tips_to_roof ? dont_move_until : 0)
    , skip_ovalisation_(skip_ovalisation)
    , all_tips_({ target_position })
    , influence_area_limit_active_(influence_area_limit_active)
    , influence_area_limit_range_(influence_area_limit_range)
{
    RecreateInfluenceLimitArea();
}

TreeSupportElement::TreeSupportElement(const TreeSupportElement& elem, Shape* newArea)
    : // copy constructor with possibility to set a new area
    target_height_(elem.target_height_)
    , target_position_(elem.target_position_)
    , next_position_(elem.next_position_)
    , next_height_(elem.next_height_)
    , effective_radius_height_(elem.effective_radius_height_)
    , to_buildplate_(elem.to_buildplate_)
    , distance_to_top_(elem.distance_to_top_)
    , area_(newArea != nullptr ? newArea : elem.area_)
    , result_on_layer_(elem.result_on_layer_)
    , increased_to_model_radius_(elem.increased_to_model_radius_)
    , to_model_gracious_(elem.to_model_gracious_)
    , buildplate_radius_increases_(elem.buildplate_radius_increases_)
    , use_min_xy_dist_(elem.use_min_xy_dist_)
    , supports_roof_(elem.supports_roof_)
    , dont_move_until_(elem.dont_move_until_)
    , can_use_safe_radius_(elem.can_use_safe_radius_)
    , last_area_increase_(elem.last_area_increase_)
    , missing_roof_layers_(elem.missing_roof_layers_)
    , skip_ovalisation_(elem.skip_ovalisation_)
    , all_tips_(elem.all_tips_)
    , influence_area_limit_active_(elem.influence_area_limit_active_)
    , influence_area_limit_range_(elem.influence_area_limit_range_)
    , influence_area_limit_area_(elem.influence_area_limit_area_)
{
    parents_.insert(parents_.begin(), elem.parents_.begin(), elem.parents_.end());
}

TreeSupportElement::TreeSupportElement(TreeSupportElement* element_above)
    : target_height_(element_above->target_height_)
    , target_position_(element_above->target_position_)
    , next_position_(element_above->next_position_)
    , next_height_(element_above->next_height_)
    , effective_radius_height_(element_above->effective_radius_height_)
    , to_buildplate_(element_above->to_buildplate_)
    , distance_to_top_(element_above->distance_to_top_ + 1)
    , area_(element_above->area_)
    , result_on_layer_(Point2LL(-1, -1))
    , // set to invalid as we are a new node on a new layer
    increased_to_model_radius_(element_above->increased_to_model_radius_)
    , to_model_gracious_(element_above->to_model_gracious_)
    , buildplate_radius_increases_(element_above->buildplate_radius_increases_)
    , use_min_xy_dist_(element_above->use_min_xy_dist_)
    , supports_roof_(element_above->supports_roof_)
    , dont_move_until_(element_above->dont_move_until_)
    , can_use_safe_radius_(element_above->can_use_safe_radius_)
    , last_area_increase_(element_above->last_area_increase_)
    , missing_roof_layers_(element_above->missing_roof_layers_)
    , skip_ovalisation_(false)
    , all_tips_(element_above->all_tips_)
    , influence_area_limit_active_(element_above->influence_area_limit_active_)
    , influence_area_limit_range_(element_above->influence_area_limit_range_)
    , influence_area_limit_area_(element_above->influence_area_limit_area_)
{
    parents_ = { element_above };
}

TreeSupportElement::TreeSupportElement(
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
    , dont_move_until_(std::max(first.dont_move_until_, second.dont_move_until_))
    , can_use_safe_radius_(first.can_use_safe_radius_ || second.can_use_safe_radius_)
    , missing_roof_layers_(std::min(first.missing_roof_layers_, second.missing_roof_layers_))
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
            std::max(getRadius(second.effective_radius_height_, second.buildplate_radius_increases_), getRadius(first.effective_radius_height_, first.buildplate_radius_increases_))
            - getRadius(effective_radius_height_, buildplate_radius_increases_));
        // 'buildplate_radius_increases' has to be recalculated, as when a smaller tree with a larger buildplate_radius_increases merge with a larger branch,
        //   the buildplate_radius_increases may have to be lower as otherwise the radius suddenly increases. This results often in a non integer value.
        buildplate_radius_increases_ = foot_increase_radius / (branch_radius * (diameter_scale_bp_radius - diameter_angle_scale_factor));
    }

    // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
    last_area_increase_ = AreaIncreaseSettings(
        std::min(first.last_area_increase_.type_, second.last_area_increase_.type_),
        std::min(first.last_area_increase_.increase_speed_, second.last_area_increase_.increase_speed_),
        first.last_area_increase_.increase_radius_ || second.last_area_increase_.increase_radius_,
        first.last_area_increase_.no_error_ || second.last_area_increase_.no_error_,
        first.last_area_increase_.use_min_distance_ && second.last_area_increase_.use_min_distance_,
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

void TreeSupportElement::AddParents(const std::vector<TreeSupportElement*>& adding)
{
    for (TreeSupportElement* ptr : adding)
    {
        parents_.emplace_back(ptr);
    }
}

void TreeSupportElement::RecreateInfluenceLimitArea()
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

void TreeSupportElement::setToBuildplateForAllParents(bool new_value)
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

void TreeSupportElement::saveToObj(OBJ& obj, const coord_t z, const coord_t layer_height) const
{
    if (area_)
    {
        obj.write(*area_, z, layer_height, to_model_gracious_ ? SVG::Color::BLUE : SVG::Color::RED);
    }
}

} // namespace cura
