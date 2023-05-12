//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORTSETTINGS_H
#define TREESUPPORTSETTINGS_H

#include "TreeSupportElement.h"
#include "TreeSupportEnums.h"
#include "settings/EnumSettings.h"
#include "settings/Settings.h"
#include "settings/types/Angle.h"
#include "utils/Coord_t.h"
#include "utils/Simplify.h"
#include <functional>

namespace cura
{

/*!
 * \brief This struct contains settings used in the tree support. Thanks to this most functions do not need to know of meshes etc. Also makes the code shorter.
 */
struct TreeSupportSettings
{
    TreeSupportSettings() = default; // required for the definition of the config variable in the TreeSupport class.

    TreeSupportSettings(const Settings& mesh_group_settings) :
        angle(mesh_group_settings.get<AngleRadians>("support_tree_angle")),
        angle_slow(mesh_group_settings.get<AngleRadians>("support_tree_angle_slow")),
        support_line_width(mesh_group_settings.get<coord_t>("support_line_width")),
        layer_height(mesh_group_settings.get<coord_t>("layer_height")),
        branch_radius(mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2),
        min_radius(mesh_group_settings.get<coord_t>("support_tree_tip_diameter") / 2), // The actual radius is 50 microns larger as the resulting branches will be increased by 50 microns to avoid rounding errors effectively increasing the xydistance
        max_radius(mesh_group_settings.get<coord_t>("support_tree_max_diameter")/2),
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
        support_rest_preference((support_rests_on_model && mesh_group_settings.get<std::string>("support_tree_rest_preference") == "graceful") ? RestPreference::GRACEFUL : RestPreference::BUILDPLATE),
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
        support_roof_wall_count(mesh_group_settings.get<int>("support_roof_wall_count")),
        zig_zaggify_support(mesh_group_settings.get<bool>("zig_zaggify_support")),
        maximum_deviation(mesh_group_settings.get<coord_t>("meshfix_maximum_deviation")),
        maximum_resolution(mesh_group_settings.get<coord_t>("meshfix_maximum_resolution")),
        support_roof_line_distance(mesh_group_settings.get<coord_t>("support_roof_line_distance")), // in the end the actual infill has to be calculated to subtract interface from support areas according to interface_preference.
        skip_some_zags(mesh_group_settings.get<bool>("support_skip_some_zags")),
        zag_skip_count(mesh_group_settings.get<size_t>("support_zag_skip_count")),
        connect_zigzags(mesh_group_settings.get<bool>("support_connect_zigzags")),
        settings(mesh_group_settings),
        min_feature_size(mesh_group_settings.get<coord_t>("min_feature_size")),
        min_wall_line_width(settings.get<coord_t>("min_wall_line_width")),
        fill_outline_gaps(settings.get<bool>("fill_outline_gaps")),
        simplifier(Simplify(mesh_group_settings))
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

        const std::function<void(std::vector<AngleDegrees>&, EFillMethod)> getInterfaceAngles =
            [&](std::vector<AngleDegrees>& angles, EFillMethod pattern) { // (logic) from getInterfaceAngles in FFFGcodeWriter.
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

        if(support_infill_angles.empty())
        {
            support_infill_angles.push_back(0);
        }

        getInterfaceAngles(support_roof_angles, roof_pattern);
        const std::unordered_map<std::string, InterfacePreference> interface_map =
            {
                { "support_area_overwrite_interface_area", InterfacePreference::SUPPORT_AREA_OVERWRITES_INTERFACE },
                { "interface_area_overwrite_support_area", InterfacePreference::INTERFACE_AREA_OVERWRITES_SUPPORT },
                { "support_lines_overwrite_interface_area", InterfacePreference::SUPPORT_LINES_OVERWRITE_INTERFACE },
                { "interface_lines_overwrite_support_area", InterfacePreference::INTERFACE_LINES_OVERWRITE_SUPPORT },
                { "nothing", InterfacePreference::NOTHING }
            };
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
     * \brief The smallest allowed radius, required to ensure that even at DTT 0 every circle will still be printed.
     */
    coord_t min_radius;

    /*!
     * \brief The largest allowed radius.
     */
    coord_t max_radius;

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
     * \brief Whether the branches prefer the buildplate or just flat areas in general.
     */
    RestPreference support_rest_preference;

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

    /*!
     * \brief Amount of walls the support area will have.
     */
    int support_wall_count;

    /*!
     * \brief Amount of walls the support roof area will have.
     */
    int support_roof_wall_count;

    /*!
     * \brief Whether support infill lines will be connected. Only required to calculate infill patterns.
     */
    bool zig_zaggify_support;

    /*!
     * \brief Maximum allowed deviation when simplifying.
     */
    coord_t maximum_deviation;

    /*!
     * \brief Maximum allowed resolution (length of a line segment) when simplifying. The resolution is higher when this variable is smaller => Minimum size a line segment may have.
     */
    coord_t maximum_resolution;

    /*!
     * \brief Distance between the lines of the roof.
     */
    coord_t support_roof_line_distance;

    /*!
     * \brief Only relevant for zigzag pattern. Only required to calculate infill patterns.
     */
    bool skip_some_zags;

    /*!
     * \brief Only relevant for zigzag pattern. Only required to calculate infill patterns.
     */
    size_t zag_skip_count;

    /*!
     * \brief Only relevant for zigzag pattern. Only required to calculate infill patterns.
     */
    bool connect_zigzags;

    /*!
     * \brief How overlaps of an interface area with a support area should be handled.
     */
    InterfacePreference interface_preference;

    /*!
     * \brief The infill class wants a settings object. This one will be the correct one for all settings it uses.
     */
    Settings settings;

    /*!
     * \brief Minimum thickness of any model features.
     */
    coord_t min_feature_size;

    /*!
     * \brief Minimum thickness a wall can have.
     */
    coord_t min_wall_line_width;

    /*!
     * \brief If areas of min_feature_size are enlarged to min_wall_line_width
     */
    bool fill_outline_gaps;

    /*!
     * \brief Simplifier to simplify polygons.
     */
    Simplify simplifier = Simplify(0, 0, 0);

  public:
    bool operator==(const TreeSupportSettings& other) const
    {
        return
            branch_radius == other.branch_radius &&
            tip_layers == other.tip_layers &&
            diameter_angle_scale_factor == other.diameter_angle_scale_factor &&
            layer_start_bp_radius == other.layer_start_bp_radius &&
            bp_radius == other.bp_radius &&
            diameter_scale_bp_radius == other.diameter_scale_bp_radius &&
            min_radius == other.min_radius &&
            xy_min_distance == other.xy_min_distance && // as a recalculation of the collision areas is required to set a new min_radius.
            xy_distance - xy_min_distance == other.xy_distance - other.xy_min_distance && // if the delta of xy_min_distance and xy_distance is different the collision areas have to be recalculated.
            support_rests_on_model == other.support_rests_on_model &&
            increase_radius_until_dtt == other.increase_radius_until_dtt &&
            min_dtt_to_model == other.min_dtt_to_model &&
            max_to_model_radius_increase == other.max_to_model_radius_increase &&
            maximum_move_distance == other.maximum_move_distance &&
            maximum_move_distance_slow == other.maximum_move_distance_slow &&
            z_distance_bottom_layers == other.z_distance_bottom_layers &&
            support_line_width == other.support_line_width &&
            support_overrides == other.support_overrides &&
            support_line_distance == other.support_line_distance &&
            support_roof_line_width == other.support_roof_line_width && // can not be set on a per-mesh basis currently, so code to enable processing different roof line width in the same iteration seems useless.
            support_bottom_offset == other.support_bottom_offset &&
            support_wall_count == other.support_wall_count &&
            support_pattern == other.support_pattern &&
            roof_pattern == other.roof_pattern && // can not be set on a per-mesh basis currently, so code to enable processing different roof patterns in the same iteration seems useless.
            support_roof_angles == other.support_roof_angles &&
            support_infill_angles == other.support_infill_angles &&
            increase_radius_until_radius == other.increase_radius_until_radius &&
            support_bottom_layers == other.support_bottom_layers &&
            layer_height == other.layer_height &&
            z_distance_top_layers == other.z_distance_top_layers &&
            maximum_deviation == other.maximum_deviation && // Infill generation depends on deviation and resolution.
            maximum_resolution == other.maximum_resolution &&
            support_roof_line_distance == other.support_roof_line_distance &&
            skip_some_zags == other.skip_some_zags &&
            zag_skip_count == other.zag_skip_count &&
            connect_zigzags == other.connect_zigzags &&
            interface_preference == other.interface_preference &&
            min_feature_size == other.min_feature_size && // interface_preference should be identical to ensure the tree will correctly interact with the roof.
            support_rest_preference == other.support_rest_preference &&
            max_radius == other.max_radius &&
            min_wall_line_width == other.min_wall_line_width &&
            fill_outline_gaps == other.fill_outline_gaps &&
            // The infill class now wants the settings object and reads a lot of settings, and as the infill class is used to calculate support roof lines for interface-preference. Not all of these may be required to be identical, but as I am not sure, better safe than sorry
            (
                interface_preference == InterfacePreference::INTERFACE_AREA_OVERWRITES_SUPPORT ||
                interface_preference == InterfacePreference::SUPPORT_AREA_OVERWRITES_INTERFACE ||
                (
                    settings.get<bool>("fill_outline_gaps") == other.settings.get<bool>("fill_outline_gaps") &&
                    settings.get<coord_t>("min_bead_width") == other.settings.get<coord_t>("min_bead_width") &&
                    settings.get<AngleRadians>("wall_transition_angle") == other.settings.get<AngleRadians>("wall_transition_angle") &&
                    settings.get<coord_t>("wall_transition_length") == other.settings.get<coord_t>("wall_transition_length") &&
                    settings.get<Ratio>("min_odd_wall_line_width") == other.settings.get<Ratio>("min_odd_wall_line_width") &&
                    settings.get<Ratio>("min_even_wall_line_width") == other.settings.get<Ratio>("min_even_wall_line_width") &&
                    settings.get<int>("wall_distribution_count") == other.settings.get<int>("wall_distribution_count") &&
                    settings.get<coord_t>("wall_transition_filter_distance") == other.settings.get<coord_t>("wall_transition_filter_distance") &&
                    settings.get<coord_t>("wall_transition_filter_deviation") == other.settings.get<coord_t>("wall_transition_filter_deviation") &&
                    settings.get<coord_t>("wall_line_width_x") == other.settings.get<coord_t>("wall_line_width_x") &&
                    settings.get<int>("meshfix_maximum_extrusion_area_deviation") == other.settings.get<int>("meshfix_maximum_extrusion_area_deviation")
                )
            );
    }

    /*!
     * \brief Get the Distance to top regarding the real radius this part will have. This is different from distance_to_top, which is can be used to calculate the top most layer of the branch.
     * \param elem[in] The SupportElement one wants to know the effectiveDTT
     * \return The Effective DTT.
     */
    [[nodiscard]] inline size_t getEffectiveDTT(const TreeSupportElement& elem) const
    {
        return
            elem.effective_radius_height < increase_radius_until_dtt ?
            (elem.distance_to_top < increase_radius_until_dtt ? elem.distance_to_top : increase_radius_until_dtt) :
            elem.effective_radius_height;
    }

    /*!
     * \brief Get the Radius part will have based on numeric values.
     * \param distance_to_top[in] The effective distance_to_top of the element
     * \param buildplate_radius_increases[in] The buildplate_radius_increases of the element.
     * \return The radius an element with these attributes would have.
     */
    [[nodiscard]] inline coord_t getRadius(size_t distance_to_top, const double buildplate_radius_increases = 0) const
    {
        coord_t uncapped_radius =
            (
                distance_to_top <= tip_layers ?
                /* tip  */ min_radius + (branch_radius - min_radius) * distance_to_top / tip_layers :
                /* base */ branch_radius +
                /* gradual increase */ branch_radius * (distance_to_top - tip_layers) * diameter_angle_scale_factor) + 
                    branch_radius * buildplate_radius_increases * (std::max(diameter_scale_bp_radius - diameter_angle_scale_factor, 0.0)
            );
        return std::min(uncapped_radius,max_radius);
    }

    /*!
     * \brief Get the Radius, that this element will have.
     * \param elem[in] The Element.
     * \return The radius the element has.
     */
    [[nodiscard]] inline coord_t getRadius(const TreeSupportElement& elem) const
    {
        return getRadius(getEffectiveDTT(elem), (elem.isResultOnLayerSet() || !support_rests_on_model) && elem.to_buildplate ? elem.buildplate_radius_increases : 0);
    }

    /*!
     * \brief Get the collision Radius of this Element. This can be smaller then the actual radius, as the drawAreas will cut off areas that may collide with the model.
     * \param elem[in] The Element.
     * \return The collision radius the element has.
     */
    [[nodiscard]] inline coord_t getCollisionRadius(const TreeSupportElement& elem) const
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
        const double scale = (layer_start_bp_radius - layer_idx) * diameter_scale_bp_radius;
        return scale > 0 ? std::min(coord_t(branch_radius + branch_radius * scale),max_radius) : 0;
    }

    /*!
     * \brief Return on which z in microns the layer will be printed. Used only for support infill line generation.
     * \param layer_idx[in] The layer.
     * \return The radius every element should aim to achieve.
     */
    [[nodiscard]] inline coord_t getActualZ(LayerIndex layer_idx) const
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
} // namespace cura
#endif // TREESUPPORTSETTINGS_H
