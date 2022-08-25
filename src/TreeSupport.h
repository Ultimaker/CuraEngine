// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include "TreeModelVolumes.h"
#include "boost/functional/hash.hpp" // For combining hashes
#include "polyclipping/clipper.hpp"
#include "settings/EnumSettings.h"
#include "sliceDataStorage.h"
#include "utils/polygon.h"


#define SUPPORT_TREE_CIRCLE_RESOLUTION 25 // The number of vertices in each circle.

// The various stages of the process can be weighted differently in the progress bar.
// These weights are obtained experimentally using a small sample size. Sensible weights can differ drastically based on the assumed default settings and model.
#define TREE_PROGRESS_TOTAL 10000
#define TREE_PROGRESS_PRECALC_COLL TREE_PROGRESS_TOTAL * 0.1
#define TREE_PROGRESS_PRECALC_AVO TREE_PROGRESS_TOTAL * 0.4
#define TREE_PROGRESS_GENERATE_NODES TREE_PROGRESS_TOTAL * 0.1
#define TREE_PROGRESS_AREA_CALC TREE_PROGRESS_TOTAL * 0.3
#define TREE_PROGRESS_DRAW_AREAS TREE_PROGRESS_TOTAL * 0.1

#define TREE_PROGRESS_GENERATE_BRANCH_AREAS TREE_PROGRESS_DRAW_AREAS / 3
#define TREE_PROGRESS_SMOOTH_BRANCH_AREAS TREE_PROGRESS_DRAW_AREAS / 3
#define TREE_PROGRESS_FINALIZE_BRANCH_AREAS TREE_PROGRESS_DRAW_AREAS / 3

#define SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL false
#define SUPPORT_TREE_AVOID_SUPPORT_BLOCKER true
#define SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION true
#define SUPPORT_TREE_EXPONENTIAL_THRESHOLD 1000
#define SUPPORT_TREE_EXPONENTIAL_FACTOR 1.5
#define SUPPORT_TREE_PRE_EXPONENTIAL_STEPS 1
#define SUPPORT_TREE_COLLISION_RESOLUTION 500 // Only has an effect if SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION is false

#define SUPPORT_TREE_MAX_DEVIATION 0

namespace cura
{


/*!
 * \brief Generates a tree structure to support your models.
 */

class TreeSupport
{
  public:
    using AvoidanceType = TreeModelVolumes::AvoidanceType;
    enum class InterfacePreference
    {
        INTERFACE_AREA_OVERWRITES_SUPPORT,
        SUPPORT_AREA_OVERWRITES_INTERFACE,
        INTERFACE_LINES_OVERWRITE_SUPPORT,
        SUPPORT_LINES_OVERWRITE_INTERFACE,
        NOTHING
    };

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
        bool operator==(const AreaIncreaseSettings& other) const
        {
            return increase_radius == other.increase_radius && increase_speed == other.increase_speed && type == other.type && no_error == other.no_error && use_min_distance == other.use_min_distance && move == other.move;
        }
    };

    struct SupportElement
    {
        SupportElement(coord_t distance_to_top, size_t target_height, Point target_position, bool to_buildplate, bool to_model_gracious, bool use_min_xy_dist, size_t dont_move_until, bool supports_roof, bool can_use_safe_radius, bool force_tips_to_roof, bool skip_ovalisation,bool influence_area_limit_active,coord_t influence_area_limit_range) : target_height(target_height), target_position(target_position), next_position(target_position), next_height(target_height), effective_radius_height(distance_to_top), to_buildplate(to_buildplate), distance_to_top(distance_to_top), area(nullptr), result_on_layer(target_position), increased_to_model_radius(0), to_model_gracious(to_model_gracious), buildplate_radius_increases(0), use_min_xy_dist(use_min_xy_dist), supports_roof(supports_roof), dont_move_until(dont_move_until), can_use_safe_radius(can_use_safe_radius), last_area_increase(AreaIncreaseSettings(AvoidanceType::FAST, 0, false, false, false, false)), missing_roof_layers(force_tips_to_roof ? dont_move_until : 0), skip_ovalisation(skip_ovalisation), all_tips({target_position}),influence_area_limit_active(influence_area_limit_active),influence_area_limit_range(influence_area_limit_range)
        {
            RecreateInfluenceLimitArea();
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
        SupportElement(const SupportElement& first, const SupportElement& second, size_t next_height, Point next_position, coord_t increased_to_model_radius, const TreeSupportSettings& config) : next_position(next_position), next_height(next_height), area(nullptr), increased_to_model_radius(increased_to_model_radius), use_min_xy_dist(first.use_min_xy_dist || second.use_min_xy_dist), supports_roof(first.supports_roof || second.supports_roof), dont_move_until(std::max(first.dont_move_until, second.dont_move_until)), can_use_safe_radius(first.can_use_safe_radius || second.can_use_safe_radius), missing_roof_layers(std::min(first.missing_roof_layers, second.missing_roof_layers)), skip_ovalisation(false)

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
            if (config.diameter_scale_bp_radius > 0)
            {
                coord_t foot_increase_radius = std::abs(std::max(config.getCollisionRadius(second), config.getCollisionRadius(first)) - config.getCollisionRadius(*this));
                buildplate_radius_increases = foot_increase_radius / (config.branch_radius * (config.diameter_scale_bp_radius - config.diameter_angle_scale_factor)); // buildplate_radius_increases has to be recalculated, as when a smaller tree with a larger buildplate_radius_increases merge with a larger branch the buildplate_radius_increases may have to be lower as otherwise the radius suddenly increases. This results often in a non integer value.
            }

            // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
            last_area_increase = AreaIncreaseSettings(std::min(first.last_area_increase.type, second.last_area_increase.type), std::min(first.last_area_increase.increase_speed, second.last_area_increase.increase_speed), first.last_area_increase.increase_radius || second.last_area_increase.increase_radius, first.last_area_increase.no_error || second.last_area_increase.no_error, first.last_area_increase.use_min_distance && second.last_area_increase.use_min_distance, first.last_area_increase.move || second.last_area_increase.move);

            all_tips=first.all_tips;
            all_tips.insert(all_tips.end(),second.all_tips.begin(),second.all_tips.end());
            influence_area_limit_range=std::max(first.influence_area_limit_range,second.influence_area_limit_range);
            influence_area_limit_active=first.influence_area_limit_active||second.influence_area_limit_active;
            RecreateInfluenceLimitArea();
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

        void AddParents(const std::vector<SupportElement*>& adding)
        {
            for (SupportElement* ptr : adding)
            {
                parents.emplace_back(ptr);
            }
        }

      void RecreateInfluenceLimitArea(){

          if(influence_area_limit_active)
          {
              influence_area_limit_area.clear();
              Polygon base_circle;
              const int base_radius = 50;
              for (unsigned int i = 0; i < SUPPORT_TREE_CIRCLE_RESOLUTION; i++)
              {
                  const AngleRadians angle = static_cast<double>(i) / SUPPORT_TREE_CIRCLE_RESOLUTION * TAU;
                  base_circle.emplace_back(coord_t(cos(angle) * base_radius), coord_t(sin(angle) * base_radius)); //todo put base circle at a central location
              }

              for(Point p:all_tips){
                Polygon circle;
                for (Point corner : base_circle)
                {
                  circle.add(p + corner*influence_area_limit_range/double(base_radius));
                }
                if(influence_area_limit_area.empty()){
                    influence_area_limit_area=circle.offset(0);
                }
                else{
                    influence_area_limit_area=influence_area_limit_area.intersection(circle.offset(0));
                }
              }
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
              min_radius(mesh_group_settings.get<coord_t>("support_tree_tip_diameter") / 2), // The actual radius is 50 microns larger as the resulting branches will be increased by 50 microns to avoid rounding errors effectively increasing the xydistance
              maximum_move_distance((angle < TAU / 4) ? (coord_t)(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max()),
              maximum_move_distance_slow((angle_slow < TAU / 4) ? (coord_t)(tan(angle_slow) * layer_height) : std::numeric_limits<coord_t>::max()),
              support_bottom_layers(mesh_group_settings.get<bool>("support_bottom_enable") ? round_divide(mesh_group_settings.get<coord_t>("support_bottom_height"), layer_height) : 0),
              tip_layers(std::max((branch_radius - min_radius) / (support_line_width / 3), branch_radius / layer_height)), // Ensure lines always stack nicely even if layer height is large
              diameter_angle_scale_factor(sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius),
              max_to_model_radius_increase(mesh_group_settings.get<coord_t>("support_tree_max_diameter_increase_by_merges_when_support_to_model") / 2),
              min_dtt_to_model(round_up_divide(mesh_group_settings.get<coord_t>("support_tree_min_height_to_model"), layer_height)),
              increase_radius_until_radius(mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2),
              increase_radius_until_dtt(increase_radius_until_radius <= branch_radius ? tip_layers * (increase_radius_until_radius / branch_radius) : (increase_radius_until_radius - branch_radius) / (branch_radius * diameter_angle_scale_factor)),
              support_rests_on_model(mesh_group_settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE),
              support_has_no_rest_preference(support_rests_on_model && mesh_group_settings.get<bool>("support_tree_no_rest_preference")),
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
              support_wall_count(mesh_group_settings.get<int>("support_wall_count")),
              zig_zaggify_support(mesh_group_settings.get<bool>("zig_zaggify_support")),
              maximum_deviation(mesh_group_settings.get<coord_t>("meshfix_maximum_deviation")),
              maximum_resolution(mesh_group_settings.get<coord_t>("meshfix_maximum_resolution")),
              support_roof_line_distance(mesh_group_settings.get<coord_t>("support_roof_line_distance")), // in the end the actual infill has to be calculated to subtract interface from support areas according to interface_preference.
              skip_some_zags(mesh_group_settings.get<bool>("support_skip_some_zags")),
              zag_skip_count(mesh_group_settings.get<size_t>("support_zag_skip_count")),
              connect_zigzags(mesh_group_settings.get<bool>("support_connect_zigzags")),
              settings(mesh_group_settings),
              min_feature_size(mesh_group_settings.get<coord_t>("min_feature_size"))


        {
            layer_start_bp_radius = (bp_radius - branch_radius) / (branch_radius * diameter_scale_bp_radius);

            // safeOffsetInc can only work in steps of the size xy_min_distance in the worst case => xy_min_distance has to be a bit larger than 0 in this worst case and should be large enough for performance to not suffer extremely
            // When for all meshes the z bottom and top distance is more than one layer though the worst case is xy_min_distance + min_feature_size
            // This is not the best solution, but the only one to ensure areas can not lag though walls at high maximum_move_distance.
            if (has_to_rely_on_min_xy_dist_only)
            {
                xy_min_distance = std::max(coord_t(100), xy_min_distance); // If set to low rounding errors WILL cause errors. Best to keep it above 25.
            }

            xy_distance = std::max(xy_distance, xy_min_distance);

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
            const std::unordered_map<std::string, InterfacePreference> interface_map = { { "support_area_overwrite_interface_area", InterfacePreference::SUPPORT_AREA_OVERWRITES_INTERFACE }, { "interface_area_overwrite_support_area", InterfacePreference::INTERFACE_AREA_OVERWRITES_SUPPORT }, { "support_lines_overwrite_interface_area", InterfacePreference::SUPPORT_LINES_OVERWRITE_INTERFACE }, { "interface_lines_overwrite_support_area", InterfacePreference::INTERFACE_LINES_OVERWRITE_SUPPORT }, { "nothing", InterfacePreference::NOTHING } };
            interface_preference = interface_map.at(mesh_group_settings.get<std::string>("support_interface_priority"));
        }

      private:
        double angle;
        double angle_slow;
        std::vector<coord_t> known_z;

      public:
        // some static variables dependent on other meshes that are not currently processed.
        // Has to be static because TreeSupportConfig will be used in TreeModelVolumes as this reduces redundancy.
        inline static bool some_model_contains_thick_roof = false;
        inline static bool has_to_rely_on_min_xy_dist_only = false;
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
        size_t increase_radius_until_dtt;
        /*!
         * \brief True if the branches may connect to the model.
         */
        bool support_rests_on_model;

        /*!
         * \brief Do we need regular avoidance, or only avoidance to model.
         */
        bool support_has_no_rest_preference;

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
        /*
         * \brief Amount of walls the support area will have.
         */
        int support_wall_count;
        /*
         * \brief Whether support infill lines will be connected. Only required to calculate infill patterns.
         */
        bool zig_zaggify_support;
        /*
         * \brief Maximum allowed deviation when simplifying.
         */
        coord_t maximum_deviation;
        /*
         * \brief Maximum allowed resolution (length of a line segment) when simplifying. The resolution is higher when this variable is smaller => Minimum size a line segment may have.
         */
        coord_t maximum_resolution;
        /*
         * \brief Distance between the lines of the roof.
         */
        coord_t support_roof_line_distance;
        /*
         * \brief Only relevant for zigzag pattern. Only required to calculate infill patterns.
         */
        bool skip_some_zags;
        /*
         * \brief Only relevant for zigzag pattern. Only required to calculate infill patterns.
         */
        size_t zag_skip_count;
        /*
         * \brief Only relevant for zigzag pattern. Only required to calculate infill patterns.
         */
        bool connect_zigzags;
        /*
         * \brief How overlaps of an interface area with a support area should be handled.
         */
        InterfacePreference interface_preference;

        /*
         * \brief The infill class wants a settings object. This one will be the correct one for all settings it uses.
         */
        Settings settings;

        /*
         * \brief Minimum thickness of any model features.
         */
        coord_t min_feature_size;

      public:
        bool operator==(const TreeSupportSettings& other) const
        {
            return branch_radius == other.branch_radius && tip_layers == other.tip_layers && diameter_angle_scale_factor == other.diameter_angle_scale_factor && layer_start_bp_radius == other.layer_start_bp_radius && bp_radius == other.bp_radius && diameter_scale_bp_radius == other.diameter_scale_bp_radius && min_radius == other.min_radius && xy_min_distance == other.xy_min_distance && // as a recalculation of the collision areas is required to set a new min_radius.
                   xy_distance - xy_min_distance == other.xy_distance - other.xy_min_distance && // if the delta of xy_min_distance and xy_distance is different the collision areas have to be recalculated.
                   support_rests_on_model == other.support_rests_on_model && increase_radius_until_dtt == other.increase_radius_until_dtt && min_dtt_to_model == other.min_dtt_to_model && max_to_model_radius_increase == other.max_to_model_radius_increase && maximum_move_distance == other.maximum_move_distance && maximum_move_distance_slow == other.maximum_move_distance_slow && z_distance_bottom_layers == other.z_distance_bottom_layers && support_line_width == other.support_line_width && support_overrides == other.support_overrides && support_line_distance == other.support_line_distance && support_roof_line_width == other.support_roof_line_width && // can not be set on a per-mesh basis currently, so code to enable processing different roof line width in the same iteration seems useless.
                   support_bottom_offset == other.support_bottom_offset && support_wall_count == other.support_wall_count && support_pattern == other.support_pattern && roof_pattern == other.roof_pattern && // can not be set on a per-mesh basis currently, so code to enable processing different roof patterns in the same iteration seems useless.
                   support_roof_angles == other.support_roof_angles && support_infill_angles == other.support_infill_angles && increase_radius_until_radius == other.increase_radius_until_radius && support_bottom_layers == other.support_bottom_layers && layer_height == other.layer_height && z_distance_top_layers == other.z_distance_top_layers && maximum_deviation == other.maximum_deviation && // Infill generation depends on deviation and resolution.
                   maximum_resolution == other.maximum_resolution && support_roof_line_distance == other.support_roof_line_distance && skip_some_zags == other.skip_some_zags && zag_skip_count == other.zag_skip_count && connect_zigzags == other.connect_zigzags && interface_preference == other.interface_preference
                   && min_feature_size == other.min_feature_size // interface_preference should be identical to ensure the tree will correctly interact with the roof.
                   && support_has_no_rest_preference == other.support_has_no_rest_preference
                   // The infill class now wants the settings object and reads a lot of settings, and as the infill class is used to calculate support roof lines for interface-preference. Not all of these may be required to be identical, but as I am not sure, better safe than sorry
                   && (interface_preference == InterfacePreference::INTERFACE_AREA_OVERWRITES_SUPPORT || interface_preference == InterfacePreference::SUPPORT_AREA_OVERWRITES_INTERFACE
                       || (settings.get<bool>("fill_outline_gaps") == other.settings.get<bool>("fill_outline_gaps") && settings.get<coord_t>("min_bead_width") == other.settings.get<coord_t>("min_bead_width") && settings.get<AngleRadians>("wall_transition_angle") == other.settings.get<AngleRadians>("wall_transition_angle") && settings.get<coord_t>("wall_transition_length") == other.settings.get<coord_t>("wall_transition_length") && settings.get<Ratio>("wall_split_middle_threshold") == other.settings.get<Ratio>("wall_split_middle_threshold") && settings.get<Ratio>("wall_add_middle_threshold") == other.settings.get<Ratio>("wall_add_middle_threshold") && settings.get<int>("wall_distribution_count") == other.settings.get<int>("wall_distribution_count") && settings.get<coord_t>("wall_transition_filter_distance") == other.settings.get<coord_t>("wall_transition_filter_distance") && settings.get<coord_t>("wall_transition_filter_deviation") == other.settings.get<coord_t>("wall_transition_filter_deviation") && settings.get<coord_t>("wall_line_width_x") == other.settings.get<coord_t>("wall_line_width_x")
                           && settings.get<int>("meshfix_maximum_extrusion_area_deviation") == other.settings.get<int>("meshfix_maximum_extrusion_area_deviation")));
        }


        /*!
         * \brief Get the Distance to top regarding the real radius this part will have. This is different from distance_to_top, which is can be used to calculate the top most layer of the branch.
         * \param elem[in] The SupportElement one wants to know the effectiveDTT
         * \return The Effective DTT.
         */
        [[nodiscard]] inline size_t getEffectiveDTT(const TreeSupport::SupportElement& elem) const
        {
            return elem.effective_radius_height < increase_radius_until_dtt ? (elem.distance_to_top < increase_radius_until_dtt ? elem.distance_to_top : increase_radius_until_dtt) : elem.effective_radius_height;
        }

        /*!
         * \brief Get the Radius part will have based on numeric values.
         * \param distance_to_top[in] The effective distance_to_top of the element
         * \param buildplate_radius_increases[in] The buildplate_radius_increases of the element.
         * \return The radius an element with these attributes would have.
         */
        [[nodiscard]] inline coord_t getRadius(size_t distance_to_top, const double buildplate_radius_increases = 0) const
        {
            return (distance_to_top <= tip_layers ? min_radius + (branch_radius - min_radius) * distance_to_top / tip_layers : // tip
                           branch_radius + // base
                               branch_radius * (distance_to_top - tip_layers) * diameter_angle_scale_factor)
                   + // gradual increase
                   branch_radius * buildplate_radius_increases * (std::max(diameter_scale_bp_radius - diameter_angle_scale_factor, 0.0));
        }

        /*!
         * \brief Get the Radius, that this element will have.
         * \param elem[in] The Element.
         * \return The radius the element has.
         */
        [[nodiscard]] inline coord_t getRadius(const TreeSupport::SupportElement& elem) const
        {
            return getRadius(getEffectiveDTT(elem), elem.buildplate_radius_increases);
        }

        /*!
         * \brief Get the collision Radius of this Element. This can be smaller then the actual radius, as the drawAreas will cut off areas that may collide with the model.
         * \param elem[in] The Element.
         * \return The collision radius the element has.
         */
        [[nodiscard]] inline coord_t getCollisionRadius(const TreeSupport::SupportElement& elem) const
        {
            return getRadius(elem.effective_radius_height, elem.buildplate_radius_increases);
        }

        /*!
         * \brief Get the Radius an element should at least have at a given layer.
         * \param layer_idx[in] The layer.
         * \return The radius every element should aim to achieve.
         */
        [[nodiscard]] inline coord_t recommendedMinRadius(LayerIndex layer_idx) const
        {
            double scale = (layer_start_bp_radius - layer_idx) * diameter_scale_bp_radius;
            return scale > 0 ? branch_radius + branch_radius * scale : 0;
        }

        /*!
         * \brief Return on which z in microns the layer will be printed. Used only for support infill line generation.
         * \param layer_idx[in] The layer.
         * \return The radius every element should aim to achieve.
         */
        [[nodiscard]] inline coord_t getActualZ(LayerIndex layer_idx)
        {
            return layer_idx < coord_t(known_z.size()) ? known_z[layer_idx] : (layer_idx - known_z.size()) * layer_height + known_z.size() ? known_z.back() : 0;
        }

        /*!
         * \brief Set the z every Layer is printed at. Required for getActualZ to work
         * \param z[in] The z every LayerIndex is printed. Vector is used as a map<LayerIndex,coord_t> with the index of each element being the corresponding LayerIndex
         * \return The radius every element should aim to achieve.
         */
        void setActualZ(std::vector<coord_t>& z)
        {
            known_z = z;
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
     * \brief Returns a function, evaluating if a point has to be added now. Required for a splitLines call in generateInitialAreas.
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

    /*!
     * \brief Ensures that every line segment is about distance in length. The resulting lines may differ from the original but all points are on the original
     *
     * \param input[in] The lines on which evenly spaced points should be placed.
     * \param distance[in] The distance the points should be from each other.
     * \param min_points[in] The amount of points that have to be placed. If not enough can be placed the distance will be reduced to place this many points.
     * \return A Polygons object containing the evenly spaced points. Does not represent an area, more a collection of points on lines.
     */
    Polygons ensureMaximumDistancePolyline(const Polygons& input, coord_t distance, size_t min_points) const;

    /*!
     * \brief Adds the implicit line from the last vertex of a Polygon to the first one.
     *
     * \param poly[in] The Polygons object, of which its lines should be extended.
     * \return A Polygons object with implicit line from the last vertex of a Polygon to the first one added.
     */
    Polygons toPolylines(const Polygons& poly) const;


    /*!
     * \brief Converts toolpaths to a Polygons object, containing the implicit line from first to last vertex
     *
     * \param toolpaths[in] The toolpaths.
     * \return A Polygons object with implicit line from the last vertex of a Polygon to the first one added.
     */
    Polygons toPolylines(const std::vector<VariableWidthLines> toolpaths) const;

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
    Polygons generateSupportInfillLines(const Polygons& area, bool roof, LayerIndex layer_idx, coord_t support_infill_distance, SierpinskiFillProvider* cross_fill_provider = nullptr, bool include_walls = false);

    /*!
     * \brief Unions two Polygons. Ensures that if the input is non empty that the output also will be non empty.
     * \param first[in] The first Polygon.
     * \param second[in] The second Polygon.
     * \return The union of both Polygons
     */
    [[nodiscard]] Polygons safeUnion(const Polygons first, const Polygons second = Polygons()) const;

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
     * \brief Creates the initial influence areas (that can later be propagated down) by placing them below the overhang.
     *
     * Generates Points where the Model should be supported and creates the areas where these points have to be placed.
     *
     * \param mesh[in] The mesh that is currently processed.
     * \param move_bounds[out] Storage for the influence areas.
     * \param storage[in] Background storage, required for adding roofs.
     */
    void generateInitialAreas(const SliceMeshStorage& mesh, std::vector<std::set<SupportElement*>>& move_bounds, SliceDataStorage& storage);

    /*!
     * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
     * \param me[in] Polygons object that has to be offset.
     * \param distance[in] The distance by which me should be offset. Expects values >=0.
     * \param collision[in] The area representing obstacles.
     * \param last_step_offset_without_check[in] The most it is allowed to offset in one step.
     * \param min_amount_offset[in] How many steps have to be done at least. As this uses round offset this increases the amount of vertices, which may be required if Polygons get very small. Required as arcTolerance is not exposed in offset, which should result with a similar result.
     * \return The resulting Polygons object.
     */
    [[nodiscard]] Polygons safeOffsetInc(const Polygons& me, coord_t distance, const Polygons& collision, coord_t safe_step_size, coord_t last_step_offset_without_check, size_t min_amount_offset) const;


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
     * \param relevant_offset[in] The maximal possible influence area. No guarantee regarding validity with current layer collision required, as it is ensured in-function!
     * \param to_bp_data[out] The part of the Influence area that can reach the buildplate.
     * \param to_model_data[out] The part of the Influence area that do not have to reach the buildplate. This has overlap with new_layer_data.
     * \param increased[out]  Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
     * \param overspeed[in] How much should the already offset area be offset again. Usually this is 0.
     * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
     * \return A valid support element for the next layer regarding the calculated influence areas. Empty if no influence are can be created using the supplied influence area and settings.
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
     * This redundancy is required if a to_buildplate influence area is allowed to merge with a to model influence area.
     * \param influence_areas[out] Area than can reach all further up support points. No assurance is made that the buildplate or the model can be reached in accordance to the user-supplied settings.
     * \param bypass_merge_areas[out] Influence areas ready to be added to the layer below that do not need merging.
     * \param last_layer[in] Influence areas of the current layer.
     * \param layer_idx[in] Number of the current layer.
     * \param mergelayer[in] Will the merge method be called on this layer. This information is required as some calculation can be avoided if they are not required for merging.
     */
    void increaseAreas(std::unordered_map<SupportElement, Polygons>& to_bp_areas, std::unordered_map<SupportElement, Polygons>& to_model_areas, std::map<SupportElement, Polygons>& influence_areas, std::vector<SupportElement*>& bypass_merge_areas, const std::vector<SupportElement*>& last_layer, const LayerIndex layer_idx, const bool mergelayer);

    /*!
     * \brief Propagates influence downwards, and merges overlapping ones.
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
     * \brief Draws circles around result_on_layer points of the influence areas
     *
     * \param linear_data[in] All currently existing influence areas with the layer they are on
     * \param layer_tree_polygons[out] Resulting branch areas with the layerindex they appear on. layer_tree_polygons.size() has to be at least linear_data.size() as each Influence area in linear_data will save have at least one (that's why it's a vector<vector>) corresponding branch area in layer_tree_polygons.
     * \param inverse_tree_order[in] A mapping that returns the child of every influence area.
     */
    void generateBranchAreas(std::vector<std::pair<LayerIndex, SupportElement*>>& linear_data, std::vector<std::unordered_map<SupportElement*, Polygons>>& layer_tree_polygons, const std::map<SupportElement*, SupportElement*>& inverse_tree_order);

    /*!
     * \brief Applies some smoothing to the outer wall, intended to smooth out sudden jumps as they can happen when a branch moves though a hole.
     *
     * \param layer_tree_polygons[in,out] Resulting branch areas with the layerindex they appear on.
     */
    void smoothBranchAreas(std::vector<std::unordered_map<SupportElement*, Polygons>>& layer_tree_polygons);

    /*!
     * \brief Drop down areas that do rest non-gracefully on the model to ensure the branch actually rests on something.
     *
     * \param layer_tree_polygons[in] Resulting branch areas with the layerindex they appear on.
     * \param linear_data[in] All currently existing influence areas with the layer they are on
     * \param dropped_down_areas[out] Areas that have to be added to support all non-graceful areas.
     * \param inverse_tree_order[in] A mapping that returns the child of every influence area.
     */
    void dropNonGraciousAreas(std::vector<std::unordered_map<SupportElement*, Polygons>>& layer_tree_polygons, const std::vector<std::pair<LayerIndex, SupportElement*>>& linear_data, std::vector<std::vector<std::pair<LayerIndex, Polygons>>>& dropped_down_areas, const std::map<SupportElement*, SupportElement*>& inverse_tree_order);


    /*!
     * \brief Generates Support Floor, ensures Support Roof can not cut of branches, and saves the branches as support to storage
     *
     * \param support_layer_storage[in] Areas where support should be generated.
     * \param support_roof_storage[in] Areas where support was replaced with roof.
     * \param storage[in,out] The storage where the support should be stored.
     */
    void finalizeInterfaceAndSupportAreas(std::vector<Polygons>& support_layer_storage, std::vector<Polygons>& support_roof_storage, SliceDataStorage& storage);

    /*!
     * \brief Draws circles around result_on_layer points of the influence areas and applies some post processing.
     *
     * \param move_bounds[in] All currently existing influence areas
     * \param storage[in,out] The storage where the support should be stored.
     */
    void drawAreas(std::vector<std::set<SupportElement*>>& move_bounds, SliceDataStorage& storage);

    /*!
     * \brief Settings with the indexes of meshes that use these settings.
     *
     */
    std::vector<std::pair<TreeSupportSettings, std::vector<size_t>>> grouped_meshes;

    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes.
     *
     */
    TreeModelVolumes volumes_;

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
     * \brief The progress offset added to all values communicated to the progress bar.
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
        size_t hash_node = hash<cura::Point>()(node.target_position);
        boost::hash_combine(hash_node, size_t(node.target_height));
        return hash_node;
    }
};
} // namespace std

#endif /* TREESUPPORT_H */
