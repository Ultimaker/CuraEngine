
#include "TreeSupportCradle.h"

#include "TreeSupportUtils.h"
#include "utils/MinimumBoundingBox.h"
#include "utils/ThreadPool.h"
#include "utils/linearAlg2D.h"
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/polygonUtils.h" //For moveInside.

namespace cura
{


SupportCradleGeneration::SupportCradleGeneration(const SliceDataStorage& storage, TreeModelVolumes& volumes_s)
    : volumes_(volumes_s)
    , cradle_data_(storage.meshes.size(), std::vector<std::vector<TreeSupportCradle*>>(storage.print_layer_count))
    , floating_parts_cache_(storage.meshes.size())
    , support_free_areas_(storage.print_layer_count)
{

}

size_t SupportCradleGeneration::getDirectionIdx(Point2LL a, Point2LL b) const
{
    size_t size = CradleDeformationHalfCircle().size();
    Point2LL vector_direction = a.X > b.X ? a - b : b - a;
    double best_angle = std::numbers::pi;
    size_t best_idx = 0;
    for (size_t direction_idx = 0; direction_idx < size; direction_idx++)
    {
        Point2LL direction(direction_idx % size / 2, direction_idx - size / 2);
        double angle_vd = angle(vector_direction - normal(direction, vSize(vector_direction)));
        if (angle_vd >= std::numbers::pi)
        {
            angle_vd -= std::numbers::pi;
        }

        if (angle_vd < best_angle)
        {
            best_angle = angle_vd;
            best_idx = direction_idx;
        }
    }
    return best_idx;
}

void SupportCradleGeneration::getLayerDeformation(
    const SliceMeshStorage& mesh,
    MinimumBoundingBox& minimum_box,
    double assumed_part_thickness,
    CradleDeformationHalfCircle& deform_part)
{
    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const double deformation_constant = retrieveSetting<double>(mesh.settings, "support_tree_part_deformation_constant") / 1000.0;

    Point2LL axis_min = minimum_box.extent.X < minimum_box.extent.Y ? minimum_box.axis[0] : minimum_box.axis[1];
    Point2LL axis_max = minimum_box.extent.X < minimum_box.extent.Y ? minimum_box.axis[1] : minimum_box.axis[0];

    for (size_t direction_idx = 0; direction_idx < deform_part.size(); direction_idx++)
    {
        Point2LL direction(direction_idx % deform_part.size() / 2, direction_idx - deform_part.size() / 2);
        double dot_a = dot(axis_min, direction);
        double det_a = cross(axis_min, direction);
        double angle = std::atan2(det_a, dot_a);

        if (angle < 0)
        {
            angle += 2 * std::numbers::pi;
        }
        if (angle >= std::numbers::pi)
        {
            angle -= std::numbers::pi;
        }

        if (angle >= std::numbers::pi / 2.0)
        {
            angle = std::numbers::pi / 2 - (angle - std::numbers::pi / 2.0);
        }

        double assumed_thickness_in_direction = angle / (std::numbers::pi / 2.0) * std::min(assumed_part_thickness, double(std::min(minimum_box.extent.X, minimum_box.extent.Y)))
                                              + (1.0 - angle / (std::numbers::pi / 2.0)) * double(std::max(minimum_box.extent.X, minimum_box.extent.Y));
        double assumed_thickness_opposite_direction
            = angle / (std::numbers::pi / 2.0) * double(std::max(minimum_box.extent.X, minimum_box.extent.Y))
            + (1.0 - angle / (std::numbers::pi / 2.0)) * std::min(assumed_part_thickness, double(std::min(minimum_box.extent.X, minimum_box.extent.Y)));

        deform_part[direction_idx]
            = tan(deformation_constant * layer_height / (assumed_thickness_in_direction * (1 + std::sqrt(std::max(0.0, assumed_thickness_opposite_direction)))));
    }
}

double SupportCradleGeneration::getTotalDeformation(size_t mesh_idx, const SliceMeshStorage& mesh, UnsupportedAreaInformation* element)
{
    if (element->total_deformation_maximum >= 0)
    {
        return element->total_deformation_maximum;
    }
    const size_t cradle_deformation_half_circle_size = CradleDeformationHalfCircle().size();

    LayerIndex layer_idx = element->layer_idx;
    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const double horizontal_movement_weight = retrieveSetting<double>(mesh.settings, "support_tree_side_cradle_xy_factor");
    std::unordered_set<UnsupportedAreaInformation*> elements_on_path_down {element};
    const double layer_per_mm = 1000.0 / double(layer_height);
    std::vector<std::vector<UnsupportedAreaInformation*>> root_areas(layer_idx + 1);

    {
        // Using a unordered_set to ensure no duplicates!
        std::unordered_set<UnsupportedAreaInformation*> all_scan_elements {element};
        LayerIndex scan_layer_idx = element->layer_idx;
        while(!all_scan_elements.empty() && scan_layer_idx > 0)
        {
            std::unordered_set<UnsupportedAreaInformation*> next_scan_elements;

            for(UnsupportedAreaInformation* current_scan_element : all_scan_elements)
            {
                for(UnsupportedAreaInformation* scan_element_below: current_scan_element->areas_below)
                {
                    if(scan_element_below->height > 0)
                    {
                        next_scan_elements.emplace(scan_element_below);
                    }
                    else
                    {
                        root_areas[scan_layer_idx-1].emplace_back(scan_element_below);
                    }
                    elements_on_path_down.emplace(scan_element_below);
                }
            }
            scan_layer_idx--;
            all_scan_elements = next_scan_elements;
        }
    }
    std::unordered_map<UnsupportedAreaInformation*, std::unordered_set<UnsupportedAreaInformation*>> rests_on_root;
    std::unordered_map<UnsupportedAreaInformation*, std::unordered_set<UnsupportedAreaInformation*>> rests_on_self;

    std::unordered_set<UnsupportedAreaInformation*> iterate_elements;

    for (LayerIndex iterate_layer_idx = 0; iterate_layer_idx <= layer_idx; iterate_layer_idx++)
    {
        std::unordered_set<UnsupportedAreaInformation*> next_iterate_elements;

        for (UnsupportedAreaInformation* current_area : root_areas[iterate_layer_idx])
        {
            for (UnsupportedAreaInformation* element_above : current_area->areas_above)
            {
                rests_on_root[element_above].emplace(current_area);
            }
        }
        for (UnsupportedAreaInformation* current_area : iterate_elements)
        {
            for (UnsupportedAreaInformation* element_above : current_area->areas_above)
            {
                if (current_area->areas_above.size() > 1)
                {
                    rests_on_self[element_above].emplace(current_area);
                }
                if (rests_on_self.contains(current_area))
                {
                    rests_on_self[element_above].insert(rests_on_self[current_area].begin(), rests_on_self[current_area].end());
                }
                if (rests_on_root.contains(current_area))
                {
                    rests_on_root[element_above].insert(rests_on_root[current_area].begin(), rests_on_root[current_area].end());
                }
                if (elements_on_path_down.contains(element_above))
                {
                    next_iterate_elements.emplace(element_above);
                }
            }
        }
        iterate_elements = next_iterate_elements;
    }
    iterate_elements.clear();

    std::unordered_map<UnsupportedAreaInformation*, CradleDeformationHalfCircle> known_deformation_map; // todo remove if confirmed no longer needed
    std::unordered_map<UnsupportedAreaInformation*, coord_t> largest_center_distance_map; // todo remove if confirmed no longer needed

    LayerIndex last_z_stable_layer = 0;

    UnsupportedAreaInformation* largest_deformation_in_z_direction_element = element;
    double largest_deformation_in_z_direction_deformation = 0;

    UnsupportedAreaInformation* largest_deformation_in_xy_direction_element = element;
    double largest_deformation_in_xy_direction_deformation = 0;

    for(LayerIndex iterate_layer_idx = 0; iterate_layer_idx <= layer_idx; iterate_layer_idx++)
    {
        std::unordered_set<UnsupportedAreaInformation*> next_iterate_elements;

        iterate_elements.insert(root_areas[iterate_layer_idx].begin(),root_areas[iterate_layer_idx].end());
        for(UnsupportedAreaInformation* current_area : iterate_elements)
        {
            CradleDeformationHalfCircle total_element_deformation = {};
            CradleDeformationHalfCircle element_deformation_layer = current_area->deformation;

            coord_t distance_from_top_below = 0; // todo remove if confirmed no longer needed
            Polygon overhang_path = Polygon();

            for(UnsupportedAreaInformation* element_below: current_area->areas_below)
            {
                CradleDeformationHalfCircle deformation_below = {};
                coord_t distance_below;
                if(largest_center_distance_map.contains(element_below))
                {
                    distance_below = largest_center_distance_map[element_below];
                }
                else
                {
                    distance_below = vSize(element->min_box.center - element_below->min_box.center);
                }

                if(known_deformation_map.contains(element_below))
                {
                    deformation_below = known_deformation_map[element_below];
                }

                if (total_element_deformation == CradleDeformationHalfCircle{})
                {
                    // if multiple connect to one each below should be similar (because of simulated_connection), still do a min in case it is not
                    total_element_deformation = deformation_below;
                    distance_from_top_below = distance_below;
                }
                else
                {
                    //todo Is this working correctly or does this also simulate connections (wrongly) with parts of the model that dont increase stability?
                    std::vector<UnsupportedAreaInformation*> connected_with_current;
                    for (UnsupportedAreaInformation* element_below_check : current_area->areas_below)
                    {
                        if (element_below == element_below_check)
                        {
                            continue;
                        }
                        if (known_deformation_map.contains(element_below_check))
                        {
                            total_element_deformation = elementWiseSelect(
                                known_deformation_map[element_below_check],
                                total_element_deformation,
                                [](double a, double b)
                                {
                                    return std::min(a, b);
                                });
                        }
                        std::set_intersection(
                            rests_on_root[element_below_check].begin(),
                            rests_on_root[element_below_check].end(),
                            rests_on_root[element_below].begin(),
                            rests_on_root[element_below].end(),
                            std::back_inserter(connected_with_current));
                        if (rests_on_self.contains(element_below) && rests_on_self.contains(element_below_check))
                        {
                            std::set_intersection(
                                rests_on_self[element_below_check].begin(),
                                rests_on_self[element_below_check].end(),
                                rests_on_self[element_below].begin(),
                                rests_on_self[element_below].end(),
                                std::back_inserter(connected_with_current));
                        }
                    }
                    Shape simulated_connection;
                    for (UnsupportedAreaInformation* element_connect : connected_with_current)
                    {
                        OpenLinesSet line;

                        line.addSegment(element_below->min_box.center, element_connect->min_box.center);
                        simulated_connection.push_back(line.offset(std::max(
                            std::min(element_below->min_box.extent.X, element_below->min_box.extent.Y),
                            std::min(element_connect->min_box.extent.X, element_connect->min_box.extent.Y))));
                    }
                    if (! simulated_connection.empty())
                    {
                        simulated_connection = simulated_connection.unionPolygons();
                        CradleDeformationHalfCircle simulated_deform_part = {};
                        MinimumBoundingBox simulated_connect_box(simulated_connection.getOutsidePolygons().splitIntoParts()[0]);
                        getLayerDeformation(mesh, simulated_connect_box, layer_height * layer_idx, simulated_deform_part);

                        element_deformation_layer = elementWiseSelect(
                            simulated_deform_part,
                            element_deformation_layer,
                            [](double a, double b)
                            {
                                return std::min(a, b);
                            });
                    }
                    distance_from_top_below = std::min(distance_from_top_below, distance_below);
                }
            }
            for(UnsupportedAreaInformation* element_above: current_area->areas_above)
            {
                if(elements_on_path_down.contains(element_above))
                {
                    next_iterate_elements.emplace(element_above);
                }
            }
            distance_from_top_below = std::max(distance_from_top_below, (vSize(element->min_box.center - current_area->min_box.center)));
            for (size_t direction_idx = 0; direction_idx < total_element_deformation.size(); direction_idx++)
            {
                double deformation_in_xy_direction = element_deformation_layer[direction_idx] * std::pow(double(layer_idx - iterate_layer_idx), 2) / (2 * layer_per_mm);
                if (deformation_in_xy_direction > largest_deformation_in_xy_direction_deformation)
                {
                    largest_deformation_in_xy_direction_deformation = deformation_in_xy_direction;
                    largest_deformation_in_xy_direction_element = current_area;
                }
                total_element_deformation[direction_idx] += (element_deformation_layer[direction_idx] * double(layer_idx - iterate_layer_idx)) / layer_per_mm;
            }

            // Estimate horizontal influence to deformation
            coord_t horizontal_distance = vSize(element->min_box.center - current_area->min_box.center) * horizontal_movement_weight;
            coord_t vertical_distance = (element->layer_idx - current_area->layer_idx) * layer_height;
            coord_t total_distance = std::sqrt(horizontal_distance * horizontal_distance + vertical_distance * vertical_distance);

            double h_distance_per_layer = horizontal_distance == 0 ? 0 : (horizontal_distance / (layer_height * double(layer_idx - iterate_layer_idx)));

            double deformation_in_z_direction = current_area->deformation[getDirectionIdx(element->min_box.center, current_area->min_box.center)]
                                              * std::pow(double(horizontal_distance / layer_height), 2) / (2 * layer_per_mm);
            if (deformation_in_z_direction > largest_deformation_in_z_direction_deformation)
            {
                largest_deformation_in_z_direction_deformation = deformation_in_z_direction;
                largest_deformation_in_z_direction_element = current_area;
            }
            if(current_area->total_deformation_limit > 0)
            {
                for (size_t direction_idx = 0; direction_idx < total_element_deformation.size(); direction_idx++)
                {
                    total_element_deformation[direction_idx] = std::min(total_element_deformation[direction_idx], current_area->total_deformation_limit);
                }
                largest_deformation_in_z_direction_deformation = std::min(largest_deformation_in_z_direction_deformation, current_area->total_deformation_limit);
                largest_deformation_in_xy_direction_deformation = std::min(largest_deformation_in_xy_direction_deformation, current_area->total_deformation_limit);
                distance_from_top_below = 0;
                last_z_stable_layer = current_area->layer_idx;
            }
            known_deformation_map[current_area] = total_element_deformation;
            largest_center_distance_map[current_area] = distance_from_top_below;
        }
        iterate_elements = next_iterate_elements;
    }

    if(! known_deformation_map.contains(element))
    {
        spdlog::error("Could not determine deformation at {}", layer_idx);
    }

    if(! largest_center_distance_map.contains(element))
    {
        spdlog::error("Could not determine largest center distance at {}", layer_idx);
    }

    double result_deformation_xy
        = largest_deformation_in_xy_direction_deformation; // *std::max_element( std::begin( known_deformation_map[element] ), std::end( known_deformation_map[element] ) );
    coord_t horizontal_distance = vSize(element->min_box.center - largest_deformation_in_z_direction_element->min_box.center);

    double result_deformation_z = largest_deformation_in_z_direction_deformation;
    element->total_deformation_maximum = std::pow(sqrt(result_deformation_xy) + sqrt(result_deformation_z), 2);
    printf(
        "at %d layer xy %lf z %lf horizontal distance was %ld result is %lf\n",
        element->layer_idx,
        result_deformation_xy,
        result_deformation_z,
        horizontal_distance * horizontal_movement_weight,
        element->total_deformation_maximum);

    return element->total_deformation_maximum;
}


void SupportCradleGeneration::calculateFloatingParts(const SliceDataStorage& storage, size_t mesh_idx)
{
    const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const coord_t min_wall_line_width = mesh.settings.get<coord_t>("min_wall_line_width");
    const size_t cradle_layers = retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_height") / layer_height;
    const double cradle_area_threshold = 1000 * 1000 * retrieveSetting<double>(mesh.settings, "support_tree_maximum_pointy_area");
    const bool side_cradle_enabled = retrieveSetting<bool>(mesh.settings, "support_tree_side_cradle_enabled");
    const coord_t part_stable_radius = retrieveSetting<coord_t>(mesh.settings, "support_tree_part_deformation_diameter") / 2;
    const double deformation_constant = retrieveSetting<double>(mesh.settings, "support_tree_part_deformation_constant") / 1000.0;
    const coord_t side_cradle_support_threshold = retrieveSetting<coord_t>(mesh.settings, "support_tree_part_side_cradle_support_threshold");

    LayerIndex max_layer = mesh.layers.size();

    LayerIndex start_layer = 1;
    floating_parts_cache_[mesh_idx].resize(max_layer + 1);
    std::mutex critical_floating_parts_cache;
    std::mutex critical_manual_cradle_map;
    std::vector<std::unordered_set<UnsupportedAreaInformation*>> manual_cradle_map(storage.support.supportGenerationModifiers.size());

    for (LayerIndex layer_idx = start_layer; layer_idx < max_layer; layer_idx++)
    {
        // As the collision contains z distance and xy distance it cant be used to clearly identify which parts of the model are connected to the buildplate.
        Shape layer = mesh.layers[layer_idx].getOutlines();
        const std::vector<SingleShape> layer_parts = layer.splitIntoParts();
        cura::parallel_for<size_t>(
            0,
            layer_parts.size(),
            [&](const size_t part_idx)
            {
                const SingleShape& part = layer_parts[part_idx];
                AABB part_aabb(part);
                // Use all models not only this mesh to determine if it rests on something as otherwise cutting meshes lead to issues
                bool has_support_below = ! PolygonUtils::clipPolygonWithAABB(storage.getLayerOutlines(layer_idx - 1, false, false), part_aabb).intersection(part).empty();
                Shape overhang = mesh.overhang_areas[layer_idx].intersection(part);
                coord_t overhang_area = std::max(overhang.area(), std::numbers::pi * min_wall_line_width * min_wall_line_width);
                MinimumBoundingBox minimum_box(part);

                if (! has_support_below || layer_idx == start_layer)
                {
                    UnsupportedAreaInformation* area_info = new UnsupportedAreaInformation(part, layer_idx, 0, overhang_area, CradleDeformationHalfCircle{}, minimum_box);
                    bool add_manual_cradle = false;
                    if (! has_support_below)
                    {
                        for (auto [idx, support_modifier] : storage.support.supportGenerationModifiers | ranges::views::enumerate)
                        {
                            bool possible_manual_cradle = support_modifier.isCradleModifier() && layer_idx < support_modifier.areas_.size()
                                                       && ! PolygonUtils::clipPolygonWithAABB(support_modifier.areas_[layer_idx], part_aabb).intersection(part).empty();

                            if (possible_manual_cradle)
                            {
                                add_manual_cradle = true;
                                std::lock_guard<std::mutex> critical_section_add(critical_manual_cradle_map);
                                manual_cradle_map[idx].emplace(area_info);
                            }
                        }
                    }
                    bool add_automatic_cradle = ! has_support_below && overhang_area < cradle_area_threshold;
                    if (add_manual_cradle)
                    {
                        area_info->support_required = CradlePlacementMethod::MANUAL_POINTY;
                        top_most_cradle_layer_ = layer_idx + cradle_layers + 1;
                    }
                    else if (add_automatic_cradle)
                    {
                        area_info->support_required = CradlePlacementMethod::AUTOMATIC_POINTY;
                        top_most_cradle_layer_ = layer_idx + cradle_layers + 1;
                    }
                    std::lock_guard<std::mutex> critical_section_add(critical_floating_parts_cache);
                    floating_parts_cache_[mesh_idx][layer_idx].emplace_back(area_info);
                    return;
                }

                CradleDeformationHalfCircle deform_part = {};
                double deform_radius = 0;
                if(side_cradle_enabled)
                {
                    deform_radius = std::min(minimum_box.extent.X, minimum_box.extent.Y);
                    bool found_limit_down = false;
                    double scan_height = std::max(minimum_box.extent.X, minimum_box.extent.Y);
                    for (LayerIndex scan_down_layer_idx = layer_idx - 1; scan_down_layer_idx > 0 && (layer_idx - scan_down_layer_idx) * layer_height < 10 * deform_radius;
                         scan_down_layer_idx--)
                    {
                        if (mesh.layers[scan_down_layer_idx].getOutlines().intersection(part).empty())
                        {
                            scan_height = (layer_idx - scan_down_layer_idx) * layer_height;
                            found_limit_down = true;
                            break;
                        }
                    }
                    // assume 90° triangle, get thickness of part. Thats not exactly correct, but close enough for now. The error is at most x2. Error increases as angle decreases
                    // (which means scan height will be larger anyway)
                    double assumed_part_thickness = part_stable_radius;
                    if (found_limit_down)
                    {
                        // triangle sides
                        double a = deform_radius * 2;
                        double b = scan_height;
                        double c = std::sqrt(a * a + b * b);
                        double area = 0.5 * a * b;
                        assumed_part_thickness = std::min(2 * area / c, scan_height) / 2; // /2 because all calculation works on radius
                    }

                    if (std::min(deform_radius, assumed_part_thickness) < part_stable_radius)
                    {
                        getLayerDeformation(mesh, minimum_box, assumed_part_thickness, deform_part);
                    }
                }

                size_t min_resting_on_layers = 0;
                coord_t supported_overhang_area = 0;
                bool add = false;
                std::vector<size_t> idx_of_floating_below;
                std::vector<UnsupportedAreaInformation*> cradles_below;
                for (auto [idx, floating] : floating_parts_cache_[mesh_idx][layer_idx - 1] | ranges::views::enumerate)
                {
                    if (layer_idx > 1 && ! floating->area.intersection(part).empty())
                    {
                        idx_of_floating_below.emplace_back(idx);

                        supported_overhang_area += floating->accumulated_supportable_overhang;
                        min_resting_on_layers = std::max(min_resting_on_layers, floating->height);
                        cradles_below.insert(cradles_below.end(), floating->cradles_below.begin(), floating->cradles_below.end());
                        if (floating->support_required != CradlePlacementMethod::NONE)
                        {
                            cradles_below.emplace_back(floating);
                        }
                        add = true;
                    }
                }
                bool add_manual_cradle = false;
                std::vector<size_t> manual_cradle_causes;
                for (auto [idx, support_modifier] : storage.support.supportGenerationModifiers | ranges::views::enumerate)
                {
                    bool possible_manual_cradle = support_modifier.isCradleModifier() && layer_idx < support_modifier.areas_.size();
                    LayerIndex previous_cradle_layer_idx = -1;
                    if (possible_manual_cradle)
                    {
                        {
                            std::lock_guard<std::mutex> critical_section_add(critical_manual_cradle_map);
                            for (UnsupportedAreaInformation* cradle_below : cradles_below)
                            {
                                previous_cradle_layer_idx = std::max(previous_cradle_layer_idx, cradle_below->layer_idx);
                                if (manual_cradle_map[idx].contains(cradle_below))
                                {
                                    possible_manual_cradle = false;
                                    break;
                                }
                            }
                        }
                    }
                    possible_manual_cradle &= previous_cradle_layer_idx == -1 || previous_cradle_layer_idx + cradle_layers < layer_idx;
                    Shape clipped_modifier = PolygonUtils::clipPolygonWithAABB(support_modifier.areas_[layer_idx], part_aabb);
                    if (possible_manual_cradle && ! clipped_modifier.intersection(part).empty())
                    {
                        std::lock_guard<std::mutex> critical_section_add(critical_manual_cradle_map);
                        for (UnsupportedAreaInformation* cradle_below : cradles_below)
                        {
                            manual_cradle_map[idx].emplace(cradle_below);
                        }
                        add_manual_cradle = true;
                        manual_cradle_causes.emplace_back(idx);
                    }
                }
                if (add)
                {
                    std::lock_guard<std::mutex> critical_section_add(critical_floating_parts_cache);
                    UnsupportedAreaInformation* area_info
                        = new UnsupportedAreaInformation(part, layer_idx, min_resting_on_layers + 1, overhang_area + supported_overhang_area, deform_part, minimum_box);
                    if (add_manual_cradle)
                    {
                        top_most_cradle_layer_ = layer_idx + cradle_layers + 1;
                        area_info->support_required = CradlePlacementMethod::MANUAL_SIDE;
                        std::lock_guard<std::mutex> critical_section_add_to_manual_cradle_map(critical_manual_cradle_map);
                        for (size_t manual_cradle_idx : manual_cradle_causes)
                        {
                            manual_cradle_map[manual_cradle_idx].emplace(area_info);
                        }
                        spdlog::debug("Adding manual side-cradle at layer {}", layer_idx);
                    }
                    for (size_t idx : idx_of_floating_below)
                    {
                        area_info->areas_below.emplace_back(floating_parts_cache_[mesh_idx][layer_idx - 1][idx]);
                        area_info->cradles_below = cradles_below;
                        floating_parts_cache_[mesh_idx][layer_idx - 1][idx]->areas_above.emplace_back(area_info);
                    }
                    floating_parts_cache_[mesh_idx][layer_idx].emplace_back(area_info);

                    if(side_cradle_enabled)
                    {
                        if (layer_idx * layer_height % 50000 == 0 && layer_idx > 20 || layer_idx + 5 * (1000 / layer_height) == mesh.layers.size())
                        {
                            /*
                            mfem::Mesh* mfem_mesh = toMfemMeshRasterized(mesh, mesh_idx, area_info);
                            std::ofstream mesh_ofs(std::to_string(layer_idx) + "gen.mesh");
                            mesh_ofs.precision(8);
                            mfem_mesh->Print(mesh_ofs);
                            runMfemExample(mfem_mesh, std::to_string(layer_idx), part.area()/(1000.0*1000.0*1000.0*1000.0), 0.0005); // Calculation uses meter I think
                             */
                            double estimated_deformation = getTotalDeformation(mesh_idx, mesh, area_info);
                            printf("My estimation was %lf layer is %d\n",estimated_deformation, layer_idx);
                        }
                        /*std::cout << "at " << layer_idx<< ": " << " with part deform " << deform_part << " with constant " << deformation_constant << " deform rad of " << deform_radius << " resulting estimation of " << estimated_deformation
                                  <<std::endl ;*/
                        LayerIndex last_cradle_at_layer_idx = -1;
                        for(auto& cradle : cradles_below)
                        {
                            last_cradle_at_layer_idx = std::max(last_cradle_at_layer_idx, cradle->layer_idx);
                        }
                        if(last_cradle_at_layer_idx != -1 && last_cradle_at_layer_idx < cradle_layers) // Assume any cradle reaches full height. This can be wrong! TODO
                        {
                            area_info->total_deformation_limit = EPSILON; //todo do I want to use min cradle xy distance here?
                        }
                        else
                        {
                            double estimated_deformation = getTotalDeformation(mesh_idx, mesh, area_info);
                            if(estimated_deformation > side_cradle_support_threshold) // todo additional cradle if it rests on cradle earlier
                            {
                                top_most_cradle_layer_ = layer_idx + cradle_layers + 1;
                                area_info->support_required = CradlePlacementMethod::AUTOMATIC_SIDE;
                                area_info->total_deformation_limit = EPSILON; //todo do I want to use min cradle xy distance here?
                                std::cout << "at " << layer_idx<< ": " << " Mark for support "
                                          <<std::endl ;
                            }
                        }
                    }
                }
            });
    }
}

mfem::Mesh* SupportCradleGeneration::toMfemMeshRasterized(const SliceMeshStorage& mesh, size_t mesh_idx, UnsupportedAreaInformation* element)
{
    mfem::Mesh* mfem_mesh = new mfem::Mesh(3,0,0);
    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const coord_t simulation_layer_height = layer_height;
    const size_t layers_per_simulation_step = simulation_layer_height/layer_height;
    const double deformation_constant = retrieveSetting<double>(mesh.settings, "support_tree_part_deformation_constant") / 1000.0;
    coord_t raster_size = 400;
    const coord_t wall_thickness = (mesh.settings.get<size_t>("wall_line_count") - 1) * mesh.settings.get<coord_t>("wall_line_width_x") +
                                   mesh.settings.get<coord_t>("wall_line_width_0");

    const coord_t simulated_infill_line_width_raster_multiplier = 10;

    // idea Every layer one square.
    // splits: if below make current square two squares
    // merges dont matter. we look at it top down!

    struct ComparePoint2LL {
        bool operator()(const Point2LL& a, const Point2LL& b) const {
            return a < b;
        }
    };

    struct ComparePoint3LL {
        bool operator()(const Point3LL& p0, const Point3LL& p1) const {
            if(p0.z_ < p1.z_)
            {
                return true;
            }
            else if(p0.z_ ==  p1.z_)
            {
                return p0.x_ < p1.x_ || (p0.x_ == p1.x_ && p0.y_ < p1.y_);
            }
            return false;
        }
    };


    std::vector<std::map<Point2LL, std::vector<Point3LL>,ComparePoint2LL>> square_element_map(element->layer_idx+1+layers_per_simulation_step);// contains to which vertices the upside of the cube of said element should connect
    std::map<Point3LL, int64_t,ComparePoint3LL> coordinate_id_map;
    int64_t last_added_vertex = -1;

    {
        std::unordered_set<UnsupportedAreaInformation*> all_scan_elements {element};


        LayerIndex scan_layer_idx = element->layer_idx;
        while(!all_scan_elements.empty() && scan_layer_idx > 0)
        {
            std::unordered_set<UnsupportedAreaInformation*> next_scan_elements;

            for(UnsupportedAreaInformation* current_scan_element : all_scan_elements)
            {
                if (current_scan_element->height == 0)
                {
                    continue;
                }

                if(scan_layer_idx != current_scan_element->layer_idx)
                {
                    printf("Mismattching layer idx  %d %D\n",scan_layer_idx, current_scan_element->layer_idx);
                }

                if(scan_layer_idx % layers_per_simulation_step == 0)
                {
                    AABB scan_area(current_scan_element->area);

                    for (coord_t x_scan = scan_area.min_.X; x_scan <= scan_area.max_.X; x_scan += raster_size)
                    {
                        for (coord_t y_scan = scan_area.min_.Y; y_scan <= scan_area.max_.Y; y_scan += raster_size)
                        {
                            // calc dist to inside
                            // if smaller than raster/2 assume filled and add block

                            Point2LL center_of_raster(x_scan + raster_size / 2, y_scan + raster_size / 2);
                            Point2LL center_moved = center_of_raster;
                            PolygonUtils::moveInside(current_scan_element->area, center_moved);
                            bool is_inside= current_scan_element->area.inside(center_of_raster);
                            bool is_in_wall = (is_inside && vSize2(center_moved - center_of_raster) < wall_thickness * wall_thickness);
                            bool is_close_to_wall_outside = false; // was vSize2(center_moved - center_of_raster) <= raster_size/2 * raster_size/2; todo find other way to ensure small objects are not discarded
                            bool is_infill = is_inside && !is_in_wall &&  ((center_of_raster.X / raster_size) % simulated_infill_line_width_raster_multiplier == 0 ||
                                                                           (center_of_raster.Y / raster_size) % simulated_infill_line_width_raster_multiplier == 0 );
                            if (is_in_wall || is_close_to_wall_outside || is_infill)
                            {
                                for (int i = 0; i < 8; i++)
                                {
                                    if (i < 4)
                                    {
                                        Point2LL direction(i == 1 || i == 2 ? 1 : -1, i == 2 || i== 3  ? 1 : -1);
                                        Point2LL x_direction_check(direction.X,0);
                                        Point2LL y_direction_check(0,direction.Y);
                                        Point2LL position = center_of_raster + direction * raster_size / 2;
                                        Point3LL full_position = Point3LL(position.X,position.Y,layer_height * (scan_layer_idx));
                                        square_element_map[scan_layer_idx][center_of_raster].emplace_back(full_position);
                                        coordinate_id_map[full_position] = -1;
                                    }
                                    else
                                    {
                                        Point2LL direction( i == 5 || i == 6 ? 1 : -1, i == 6 || i ==7 ? 1 : -1);
                                        Point2LL x_direction_check(direction.X,0);
                                        Point2LL y_direction_check(0,direction.Y);
                                        Point2LL position = center_of_raster + direction * raster_size / 2;
                                        Point3LL full_position = Point3LL(position.X,position.Y,layer_height * (scan_layer_idx + layers_per_simulation_step));
                                        coordinate_id_map[full_position] = -1;
                                        square_element_map[scan_layer_idx][center_of_raster].emplace_back(full_position);
                                    }
                            }
                        }
                    }
                }
            }
            for(UnsupportedAreaInformation* scan_element_below: current_scan_element->areas_below)
            {
                next_scan_elements.emplace(scan_element_below);
            }
        }
        scan_layer_idx--;
        all_scan_elements = next_scan_elements;
    }
    }

    for(auto& elem: coordinate_id_map)
    {
        last_added_vertex++;
        coordinate_id_map[elem.first] = last_added_vertex;
        mfem_mesh->AddVertex(INT2MM2(elem.first.x_), INT2MM2(elem.first.y_), INT2MM2(elem.first.z_));
        mfem_mesh->SetNode(last_added_vertex, mfem_mesh->GetVertex(last_added_vertex));
    }
    for(LayerIndex scan_layer_idx = 0; scan_layer_idx <= element->layer_idx;  scan_layer_idx++)
    {

        for(auto& elem: square_element_map[scan_layer_idx])
        {
            std::vector<int> cube_vertices;
            for(Point3LL p:elem.second)
            {
                cube_vertices.emplace_back(coordinate_id_map[p]);
            }

            bool has_element_below = scan_layer_idx > layers_per_simulation_step && square_element_map[scan_layer_idx-layers_per_simulation_step].contains(elem.first);
            bool has_element_above = square_element_map[scan_layer_idx+layers_per_simulation_step].contains(elem.first);
            bool has_element_left = square_element_map[scan_layer_idx].count(Point2LL(elem.first.X - raster_size, elem.first.Y));
            bool has_element_right = square_element_map[scan_layer_idx].count(Point2LL(elem.first.X + raster_size, elem.first.Y));
            bool has_element_up = square_element_map[scan_layer_idx].count(Point2LL(elem.first.X, elem.first.Y + raster_size));
            bool has_element_down = square_element_map[scan_layer_idx].count(Point2LL(elem.first.X, elem.first.Y - raster_size));

            mfem::Hexahedron* cube = new mfem::Hexahedron(cube_vertices.data(), has_element_below ? 2 : 1);
            mfem_mesh->AddElement(cube);
            //Add boundaries
            if(!has_element_left)
            {
                mfem_mesh->AddBdrQuad(cube_vertices[3],cube_vertices[0],cube_vertices[4],cube_vertices[7],3);
            }
            if(!has_element_right)
            {
                mfem_mesh->AddBdrQuad(cube_vertices[1],cube_vertices[2],cube_vertices[6],cube_vertices[5],3);
            }
            if(!has_element_down)
            {
                mfem_mesh->AddBdrQuad(cube_vertices[0],cube_vertices[1],cube_vertices[5],cube_vertices[4],3);
            }
            if(!has_element_up)
            {
                mfem_mesh->AddBdrQuad(cube_vertices[2],cube_vertices[3],cube_vertices[7],cube_vertices[6],3);
            }
            if(!has_element_below)
            {
                mfem_mesh->AddBdrQuad(cube_vertices[0], cube_vertices[1], cube_vertices[2], cube_vertices[3], 1);
            }
            if(!has_element_above)
            {
                mfem_mesh->AddBdrQuad(cube_vertices[4], cube_vertices[5], cube_vertices[6], cube_vertices[7], 2);
            }
        }
    }
    mfem_mesh->FinalizeTopology();
    mfem_mesh->Finalize(false,true);

    return mfem_mesh;

}


void SupportCradleGeneration::runMfemExample(mfem::Mesh* mesh, std::string prefix_name, double top_area, double force)
{
    int vert_idx = mesh->GetNV()-1;

    mfem::real_t vertex_before[3];
    mesh->GetNode(vert_idx,vertex_before);

    int dim = mesh->Dimension();
    int order = 1;
    bool static_cond = false;
    bool visualization = 1;
    if (mesh->attributes.Max() < 2 || mesh->bdr_attributes.Max() < 2)
    {
        std::cerr << "\nInput mesh should have at least two materials and "
             << "two boundary attributes! (See schematic in ex2.cpp)\n"
             << std::endl;
        return;
    }

    // 3. Select the order of the finite element discretization space. For NURBS
    //    meshes, we increase the order by degree elevation.
    if (mesh->NURBSext)
    {
        mesh->DegreeElevate(order, order);
    }

    // 4. Refine the mesh to increase the resolution. In this example we do
    //    'ref_levels' of uniform refinement. We choose 'ref_levels' to be the
    //    largest number that gives a final mesh with no more than 5,000
    //    elements.
    /*{
        int ref_levels =
            (int)floor(log(5000./mesh->GetNE())/log(2.)/dim);
        for (int l = 0; l < ref_levels; l++)
        {
            mesh->UniformRefinement();
        }
    }*/

    // 5. Define a finite element space on the mesh. Here we use vector finite
    //    elements, i.e. dim copies of a scalar finite element space. The vector
    //    dimension is specified by the last argument of the FiniteElementSpace
    //    constructor. For NURBS meshes, we use the (degree elevated) NURBS space
    //    associated with the mesh nodes.
    mfem::FiniteElementCollection *fec;
    mfem::FiniteElementSpace *fespace;
    if (mesh->NURBSext)
    {
        fec = NULL;
        fespace = mesh->GetNodes()->FESpace();
    }
    else
    {
        fec = new mfem::H1_FECollection(order, dim);
        fespace = new mfem::FiniteElementSpace(mesh, fec, dim);
    }
    std::cout << "Number of finite element unknowns: " << fespace->GetTrueVSize()
         << std::endl << "Assembling: " << std::flush;

    // 6. Determine the list of true (i.e. conforming) essential boundary dofs.
    //    In this example, the boundary conditions are defined by marking only
    //    boundary attribute 1 from the mesh as essential and converting it to a
    //    list of true dofs.
    mfem::Array<int> ess_tdof_list, ess_bdr(mesh->bdr_attributes.Max());
    ess_bdr = 0;
    ess_bdr[0] = 1;
    fespace->GetEssentialTrueDofs(ess_bdr, ess_tdof_list);

    // 7. Set up the linear form b(.) which corresponds to the right-hand side of
    //    the FEM linear system. In this case, b_i equals the boundary integral
    //    of f*phi_i where f represents a "pull down" force on the Neumann part
    //    of the boundary and phi_i are the basis functions in the finite element
    //    fespace. The force is defined by the VectorArrayCoefficient object f,
    //    which is a vector of Coefficient objects. The fact that f is non-zero
    //    on boundary attribute 2 is indicated by the use of piece-wise constants
    //    coefficient for its last component.
    mfem::VectorArrayCoefficient f(dim);
    for (int i = 1; i < dim; i++)
    {
        f.Set(i, new mfem::ConstantCoefficient(0.0));
    }
    {
        mfem::Vector pull_force(mesh->bdr_attributes.Max());
        pull_force = 0.0;
        pull_force(1) = force/top_area;
        f.Set(0, new mfem::PWConstCoefficient(pull_force));
        f.Set(1, new mfem::PWConstCoefficient(pull_force));

    }

    mfem::LinearForm *b = new mfem::LinearForm(fespace);
    b->AddBoundaryIntegrator(new mfem::VectorBoundaryLFIntegrator(f));
    std::cout << "r.h.s. ... " << std::flush;
    b->Assemble();

    // 8. Define the solution vector x as a finite element grid function
    //    corresponding to fespace. Initialize x with initial guess of zero,
    //    which satisfies the boundary conditions.
    mfem::GridFunction x(fespace);
    x = 0.0;

    // 9. Set up the bilinear form a(.,.) on the finite element space
    //    corresponding to the linear elasticity integrator with piece-wise
    //    constants coefficient lambda and mu.
    mfem::Vector lambda(mesh->attributes.Max());
    lambda = 3.500 * 1000.0 * 1000.0 * 1000.0;
    //lambda(0) = lambda(1)*50;
    mfem::PWConstCoefficient lambda_func(lambda);
    mfem::Vector mu(mesh->attributes.Max());
    mu = 0.800  * 1000.0 * 1000.0 * 1000.0;;
    //mu(0) = mu(1)*50;
    mfem::PWConstCoefficient mu_func(mu);

    mfem::BilinearForm *a = new mfem::BilinearForm(fespace);
    a->AddDomainIntegrator(new mfem::ElasticityIntegrator(lambda_func,mu_func));

    // 10. Assemble the bilinear form and the corresponding linear system,
    //     applying any necessary transformations such as: eliminating boundary
    //     conditions, applying conforming constraints for non-conforming AMR,
    //     static condensation, etc.
    std::cout << "matrix ... " << std::flush;
    if (static_cond) { a->EnableStaticCondensation(); }
    a->Assemble();

    mfem::SparseMatrix A;
    mfem::Vector B, X;
    a->FormLinearSystem(ess_tdof_list, x, *b, A, X, B);
    std::cout << "done." << std::endl;

    std::cout << "Size of linear system: " << A.Height() << std::endl;

#ifndef MFEM_USE_SUITESPARSE
    // 11. Define a simple symmetric Gauss-Seidel preconditioner and use it to
    //     solve the system Ax=b with PCG.
    mfem::GSSmoother M(A);
    PCG(A, M, B, X, false, 100000, 1e-8, 0.0);
#else
    // 11. If MFEM was compiled with SuiteSparse, use UMFPACK to solve the system.
    UMFPackSolver umf_solver;
    umf_solver.Control[UMFPACK_ORDERING] = UMFPACK_ORDERING_METIS;
    umf_solver.SetOperator(A);
    umf_solver.Mult(B, X);
#endif

    // 12. Recover the solution as a finite element grid function.
    a->RecoverFEMSolution(X, *b, x);

    // 13. For non-NURBS meshes, make the mesh curved based on the finite element
    //     space. This means that we define the mesh elements through a fespace
    //     based transformation of the reference element. This allows us to save
    //     the displaced mesh as a curved mesh when using high-order finite
    //     element displacement field. We assume that the initial mesh (read from
    //     the file) is not higher order curved mesh compared to the chosen FE
    //     space.
    if (!mesh->NURBSext)
    {
        mesh->SetNodalFESpace(fespace);
    }

    // 14. Save the displaced mesh and the inverted solution (which gives the
    //     backward displacements to the original grid). This output can be
    //     viewed later using GLVis: "glvis -m displaced.mesh -g sol.gf".
    {
        mfem::GridFunction *nodes = mesh->GetNodes();
        *nodes += x;
        x *= -1;
        std::ofstream mesh_ofs(prefix_name + "displaced.mesh");
        mesh_ofs.precision(8);
        mesh->Print(mesh_ofs);
        std::ofstream sol_ofs(prefix_name+"sol.gf");
        sol_ofs.precision(8);
        x.Save(sol_ofs);
    }

    mfem::real_t vertex_after[3];
    mesh->GetNode(vert_idx,vertex_after);
    Point2LL point_before(vertex_before[0]*1000*1000,vertex_before[1]*1000*1000);
    Point2LL point_after(vertex_after[0]*1000*1000,vertex_after[1]*1000*1000);

    printf("%d was %lf %lf %lf, Now %lf %lf %lf \n", vert_idx,vertex_before[0],vertex_before[1],vertex_before[2],vertex_after[0],vertex_after[1],vertex_after[2]);
    printf("Estimated deformation is %ld \n", vSize(point_before-point_after));

}


std::vector<SupportCradleGeneration::UnsupportedAreaInformation*> SupportCradleGeneration::getFullyUnsupportedArea(size_t mesh_idx, LayerIndex layer_idx)
{
    std::vector<UnsupportedAreaInformation*> result;

    if (layer_idx == 0)
    {
        return result;
    }

    bool has_result = false;
    {
        std::lock_guard<std::mutex> critical_section(*critical_floating_parts_cache_);
        has_result = layer_idx < floating_parts_cache_[mesh_idx].size();
    }

    if (has_result)
    {
        std::lock_guard<std::mutex> critical_section(*critical_floating_parts_cache_);
        for (auto [idx, floating_data] : floating_parts_cache_[mesh_idx][layer_idx] | ranges::views::enumerate)
        {
            if (floating_data->support_required != CradlePlacementMethod::NONE)
            {
                result.emplace_back(floating_data);
            }
        }
    }
    else
    {
        spdlog::error("Requested not calculated unsupported area.", layer_idx);
        return result;
    }
    return result;
}

std::vector<std::vector<TreeSupportCradle*>> SupportCradleGeneration::generateCradleCenters(const SliceMeshStorage& mesh, size_t mesh_idx)
{
    std::shared_ptr<CradleConfig> cradle_config = std::make_shared<CradleConfig>(mesh, mesh.settings.get<bool>("support_roof_enable"));

    const size_t z_distance_top_layers = round_up_divide(mesh.settings.get<coord_t>("support_top_distance"), mesh.settings.get<coord_t>("layer_height"));
    const size_t z_distance_delta_(std::min(z_distance_top_layers + 1, mesh.overhang_areas.size()));
    const bool support_moves = mesh.settings.get<ESupportStructure>("support_structure") != ESupportStructure::NORMAL;
    coord_t maximum_move_distance = 0;
    coord_t maximum_move_distance_slow = 0;
    if(support_moves)
    {
        TreeSupportSettings config(mesh.settings);
        maximum_move_distance = config.maximum_move_distance;
        maximum_move_distance_slow = config.maximum_move_distance_slow;
    }
    std::mutex critical_dedupe;
    std::vector<std::unordered_set<UnsupportedAreaInformation*>> dedupe(mesh.overhang_areas.size());
    std::vector<std::vector<TreeSupportCradle*>> result(mesh.overhang_areas.size());
    cura::parallel_for<coord_t>(
        1,
        mesh.layers.size() - (z_distance_delta_ + 1),
        [&](const LayerIndex layer_idx)
        {
            if (getFullyUnsupportedArea(mesh_idx, layer_idx + z_distance_delta_).empty())
            {
                return;
            }

            for (auto pointy_info : getFullyUnsupportedArea(mesh_idx, layer_idx + z_distance_delta_))
            {
                if(pointy_info->height  == 0)
                {
                    AABB overhang_aabb(mesh.overhang_areas[layer_idx + z_distance_delta_]);
                    if (PolygonUtils::clipPolygonWithAABB(mesh.overhang_areas[layer_idx + z_distance_delta_], overhang_aabb).intersection(pointy_info->area).empty())
                    {
                        // It will be assumed that if it touches this mesh's overhang, it will be part of that mesh.
                        continue;
                    }
                }

                std::vector<Shape> accumulated_model(std::min(cradle_config->cradle_layers_ + cradle_config->cradle_z_distance_layers_ + 1, mesh.overhang_areas.size() - layer_idx), Shape());
                std::vector<UnsupportedAreaInformation*> all_pointy{ pointy_info };

                Point2LL center_prev = Polygon(pointy_info->area.getOutsidePolygons()[0]).centerOfMass();
                std::vector<Point2LL> additional_centers;
                TreeSupportCradle* cradle_main
                    = new TreeSupportCradle(layer_idx, center_prev, cradle_config->cradle_base_roof_, pointy_info->support_required, cradle_config, mesh_idx);
                for (size_t z_distance = 0; z_distance < z_distance_top_layers; z_distance++)
                {
                    accumulated_model[z_distance] = pointy_info->area;
                    cradle_main->centers_.emplace_back(center_prev);
                }
                Shape shadow; // A combination of all outlines of the model that will be supported with a cradle.
                bool aborted = false;
                bool contacted_other_pointy = false;
                std::vector<Shape> unsupported_model(accumulated_model.size());
                for (size_t cradle_up_layer = 0; cradle_up_layer < accumulated_model.size() - z_distance_top_layers; cradle_up_layer++)
                {
                    // shadow model up => not cradle where model
                    // then drop cradle down
                    // cut into parts => get close to original pointy that are far enough from each other.
                    std::vector<UnsupportedAreaInformation*> next_pointy;
                    Shape model_outline;
                    bool blocked_by_dedupe = false;
                    // The cradle base is below the bottommost unsupported and the first cradle layer is around it, so this will be needed only for the second one and up
                    if (cradle_up_layer > 1)
                    {
                        for (UnsupportedAreaInformation* pointy : all_pointy)
                        {
                            for (auto next_pointy_data : pointy->areas_above)
                            {
                                if (next_pointy_data->height
                                    != (cradle_up_layer - 1) + pointy_info->height) // If the area belongs to another pointy overhang stop and let this other overhang handle it
                                {
                                    contacted_other_pointy = true;
                                    continue;
                                }
                                unsupported_model[cradle_up_layer].push_back(next_pointy_data->area);
                                // Ensure each area is only handles once
                                std::lock_guard<std::mutex> critical_section_cradle(critical_dedupe);
                                if (! dedupe[layer_idx + cradle_up_layer].contains(next_pointy_data))
                                {
                                    dedupe[layer_idx + cradle_up_layer].emplace(next_pointy_data);
                                    model_outline.push_back(next_pointy_data->area);
                                    next_pointy.emplace_back(next_pointy_data);
                                }
                                else
                                {
                                    blocked_by_dedupe = true;
                                }

                                std::vector<UnsupportedAreaInformation*> all_pointy_below{ next_pointy_data };
                                for (int64_t cradle_down_layer = cradle_up_layer; cradle_down_layer > 0 && ! all_pointy_below.empty(); cradle_down_layer--)
                                {
                                    std::vector<UnsupportedAreaInformation*> next_all_pointy_below;

                                    for (UnsupportedAreaInformation* pointy_below : all_pointy_below)
                                    {
                                        for (UnsupportedAreaInformation* prev_pointy_data : pointy_below->areas_below)
                                        {
                                            if (prev_pointy_data != pointy || cradle_down_layer != cradle_up_layer)
                                            {
                                                // Only add if area below does not have it's own cradle.
                                                if (prev_pointy_data->height < cradle_config->cradle_layers_min_)
                                                {
                                                    accumulated_model[cradle_down_layer].push_back(prev_pointy_data->area);
                                                    next_all_pointy_below.emplace_back(prev_pointy_data);
                                                }
                                            }
                                        }
                                    }
                                    all_pointy_below = next_all_pointy_below;
                                    accumulated_model[cradle_down_layer] = accumulated_model[cradle_down_layer].unionPolygons();
                                }
                            }
                        }
                        all_pointy = next_pointy;
                    }
                    else
                    {
                        model_outline.push_back(pointy_info->area);
                    }

                    if (model_outline.empty())
                    {
                        if (cradle_up_layer < cradle_config->cradle_layers_min_)
                        {
                            aborted = true;
                            break;
                        }

                        if (! blocked_by_dedupe)
                        {
                            // The model is surrounded with cradle based on the area above (z distance).
                            // When an area that should have a cradle merges with a buildplate supported area above, it will no longer exist for a cradle.
                            // But if the cradle stops there will be z distance layer between the end of the cradle and said merge.
                            // To reduce the impact an area is estimated where the cradle should be for these areas.
                            Shape previous_area = shadow;
                            for (size_t cradle_up_layer_z_distance = cradle_up_layer;
                                 cradle_up_layer_z_distance < std::min(cradle_up_layer + cradle_config->cradle_z_distance_layers_, accumulated_model.size() - z_distance_top_layers);
                                 cradle_up_layer_z_distance++)
                            {
                                accumulated_model[cradle_up_layer_z_distance + z_distance_top_layers] = unsupported_model[cradle_up_layer_z_distance].unionPolygons();
                            }
                        }
                        break;
                    }

                    model_outline = model_outline.unionPolygons();
                    shadow = shadow.offset(-maximum_move_distance).unionPolygons(model_outline);
                    accumulated_model[cradle_up_layer + z_distance_top_layers] = shadow;

                    if (cradle_up_layer > 0)
                    {
                        Point2LL shadow_center = Polygon(shadow.getOutsidePolygons()[0]).centerOfMass();
                        coord_t center_move_distance = support_moves ? std::min(maximum_move_distance_slow, cradle_config->cradle_line_width_ / 3) : cradle_config->cradle_line_width_ / 3;
                        center_move_distance = std::min(center_move_distance, vSize(shadow_center - center_prev));
                        center_prev = center_prev + normal(shadow_center - center_prev, center_move_distance);
                        cradle_main->centers_.emplace_back(center_prev);
                    }
                }

                if (aborted)
                {
                    // If aborted remove all model information for the cradle generation except the pointy overhang, as it may be needed to cut a small hole in the large interface
                    // base. todo reimplement that

                    Shape cradle_0 = accumulated_model[0];
                    accumulated_model.clear();
                    accumulated_model.emplace_back(cradle_0);
                    delete cradle_main;
                }
                else
                {
                    cradle_main->shadow_ = accumulated_model;
                    result[layer_idx].emplace_back(cradle_main);
                }
            }
        });
    return result;
}

void SupportCradleGeneration::generateCradleLines(std::vector<std::vector<TreeSupportCradle*>>& cradle_data_mesh, const SliceMeshStorage& mesh)
{

    const size_t z_distance_top_layers = round_up_divide(mesh.settings.get<coord_t>("support_top_distance"), mesh.settings.get<coord_t>("layer_height"));
    const size_t z_distance_bottom_layers = round_up_divide(mesh.settings.get<coord_t>("support_bottom_distance"), mesh.settings.get<coord_t>("layer_height"));
    const bool support_rests_on_model = mesh.settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
    const coord_t support_line_width = mesh.settings.get<coord_t>("support_line_width");
    const coord_t xy_distance = mesh.settings.get<coord_t>("support_xy_distance");
    const bool xy_overrides = mesh.settings.get<SupportDistPriority>("support_xy_overrides_z") == SupportDistPriority::XY_OVERRIDES_Z;
    const coord_t xy_min_distance = !xy_overrides ? mesh.settings.get<coord_t>("support_xy_distance_overhang") : xy_distance;
    const bool support_moves = mesh.settings.get<ESupportStructure>("support_structure") != ESupportStructure::NORMAL;

    const coord_t minimum_area_to_be_supportable = support_moves ? mesh.settings.get<coord_t>("support_tree_tip_diameter") / 2 : 0;
    cura::parallel_for<coord_t>(
        1,
        cradle_data_mesh.size(),
        [&](const LayerIndex layer_idx)
        {
            for (auto [center_idx, cradle] : cradle_data_mesh[layer_idx] | ranges::views::enumerate)
            {
                const coord_t max_cradle_jump_length_forward = cradle->config_->cradle_length_ / 3;
                constexpr bool ignore_xy_dist_for_jumps = true;
                const coord_t max_cradle_xy_distance = *std::max_element(cradle->config_->cradle_xy_distance_.begin(), cradle->config_->cradle_xy_distance_.end());
                std::vector<bool> removed_directions(cradle->config_->cradle_line_count_);
                const auto& accumulated_model = cradle->shadow_;
                for (auto [idx, model_shadow] : accumulated_model | ranges::views::enumerate)
                {
                    Point2LL center = cradle->getCenter(layer_idx + idx);
                    const coord_t current_cradle_xy_distance = cradle->config_->cradle_xy_distance_[idx];
                    const coord_t previous_cradle_xy_distance = idx > 0 ? cradle->config_->cradle_xy_distance_[idx-1] : current_cradle_xy_distance;
                    const coord_t current_cradle_length = cradle->config_->cradle_length_ + max_cradle_xy_distance - current_cradle_xy_distance;

                    if (cradle->lines_.empty())
                    {
                        cradle->lines_.resize(cradle->config_->cradle_line_count_);
                    }

                    if (idx > cradle->config_->cradle_z_distance_layers_ && ! model_shadow.empty())
                    {
                        Shape relevant_forbidden = volumes_.getAvoidance(
                            0,
                            layer_idx + idx,
                            (only_gracious_ || ! support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                            support_rests_on_model,
                            true);

                        Shape this_part_influence = model_shadow.offset(current_cradle_xy_distance + cradle->config_->cradle_line_width_ / 2);

                        for (size_t layer_offset = 1; layer_offset <= z_distance_bottom_layers && layer_offset <= idx; layer_offset++)
                        {
                            this_part_influence.push_back(accumulated_model[idx - layer_offset].offset(current_cradle_xy_distance + cradle->config_->cradle_line_width_ / 2));
                        }

                        for (coord_t layer_offset = 1; layer_offset <= z_distance_top_layers && layer_offset + idx < accumulated_model.size(); layer_offset++)
                        {
                            const coord_t required_range_x = coord_t(
                                (current_cradle_xy_distance + cradle->config_->cradle_line_width_ / 2)
                                - ((layer_offset - (z_distance_top_layers == 1 ? 0.5 : 0)) * (current_cradle_xy_distance + cradle->config_->cradle_line_width_ / 2)
                                   / z_distance_top_layers));
                            this_part_influence.push_back(accumulated_model[idx + layer_offset].offset(required_range_x));
                        }

                        this_part_influence = this_part_influence.unionPolygons();

                        coord_t cradle_min_xy_distance_delta = std::max(xy_min_distance - current_cradle_xy_distance, coord_t(0));

                        // Somewhere, Somehow there is a small rounding error which causes small slivers of collision of the model to remain.
                        //  To prevent this offset my the delta before removing the influence of the model.
                        relevant_forbidden = relevant_forbidden.offset(-cradle_min_xy_distance_delta)
                                                 .difference(this_part_influence.offset(cradle_min_xy_distance_delta + EPSILON).unionPolygons())
                                                 .offset(cradle_min_xy_distance_delta + cradle->config_->cradle_line_width_ / 2 + FUDGE_LENGTH)
                                                 .unionPolygons(this_part_influence)
                                                 .unionPolygons(volumes_.getSupportBlocker(layer_idx + idx).offset(cradle->config_->cradle_line_width_ / 2));
                        coord_t max_distance2 = 0;
                        for (auto line : model_shadow)
                        {
                            for (Point2LL p : line)
                            {
                                max_distance2 = std::max(max_distance2, vSize2(center - p));
                            }
                        }

                        ClosedPolyline max_outer_points = PolygonUtils::makeCircle(
                            center,
                            sqrt(max_distance2) + current_cradle_length * 2.0,
                            cradle->config_->cradle_line_count_);

                        // create lines that go from the furthest possible location to the center
                        OpenLinesSet lines_to_center;
                        for (Point2LL p : max_outer_points)
                        {
                            Point2LL direction = p - center;
                            lines_to_center.addSegment(p, center + normal(direction, support_line_width));
                        }

                        // Subtract the model shadow up until this layer from the lines.
                        if (idx > 0)
                        {
                            lines_to_center = model_shadow.offset(current_cradle_xy_distance + cradle->config_->cradle_line_width_ / 2).unionPolygons().difference(lines_to_center, false);
                        }

                        // shorten lines to be at most SUPPORT_TREE_CRADLE_WIDTH long, with the location closest to the center not changing
                        OpenLinesSet shortened_lines_to_center;
                        for (auto [line_idx, line] : lines_to_center | ranges::views::enumerate)
                        {
                            bool front_closer = vSize2(line.front() - center) < vSize2(line.back() - center);
                            Point2LL closer = front_closer ? line.front() : line.back();
                            Point2LL further = front_closer ? line.back() : line.front();
                            coord_t cradle_line_length = line.length();
                            if (cradle_line_length < cradle->config_->cradle_length_min_)
                            {
                                continue;
                            }
                            if (line.length() <= current_cradle_length)
                            {
                                shortened_lines_to_center.push_back(line);
                            }
                            else
                            {
                                double scale = (double(current_cradle_length) / double(vSize(further - closer)));
                                Point2LL correct_length = closer + (further - closer) * scale;
                                shortened_lines_to_center.addSegment(correct_length, closer);
                            }
                        }
                        // If a line is drawn, but half of it removed as it would collide with the collision, there may not actually be a print line. The offset should prevent
                        // this.
                        shortened_lines_to_center = relevant_forbidden.difference(shortened_lines_to_center, false);
                        std::vector<OpenPolyline> ordered_lines_to_center(cradle->config_->cradle_line_count_);

                        // Evaluate which lines are still valid after the avoidance was subtracted
                        for (auto [line_idx, line] : shortened_lines_to_center | ranges::views::enumerate)
                        {
                            bool front_closer = vSize2(line.front() - center) < vSize2(line.back() - center);
                            Point2LL closer = front_closer ? line.front() : line.back();
                            Point2LL further = front_closer ? line.back() : line.front();
                            Point2LL current_direction = further - closer;
                            coord_t distance_from_center = vSize(closer - center);
                            size_t angle_idx = cradle->getIndexForLineEnd(further, layer_idx + idx);

                            if (removed_directions[angle_idx])
                            {
                                continue;
                            }

                            bool keep_line = false;
                            bool found_candidate = false;
                            bool too_short = (vSize(closer - further)) < cradle->config_->cradle_length_min_;
                            bool too_long_jump = false;
                            Point2LL closest_on_prev_segment;
                            if(! cradle->lines_[angle_idx].empty())
                            {
                                closest_on_prev_segment = LinearAlg2D::getClosestOnLineSegment(closer, cradle->lines_[angle_idx].back().line_.front(), cradle->lines_[angle_idx].back().line_.back());
                                Point2LL closest_on_prev_line = LinearAlg2D::getClosestOnLine(closer, cradle->lines_[angle_idx].back().line_.front(), cradle->lines_[angle_idx].back().line_.back());
                                coord_t xy_distance_jump = std::max(coord_t(0), previous_cradle_xy_distance - current_cradle_xy_distance);
                                //todo if jump too long check if line could be shortened. Do that above!
                                too_long_jump = vSize(closest_on_prev_segment - closest_on_prev_line)  - (ignore_xy_dist_for_jumps? xy_distance_jump : 0)  >
                                          max_cradle_jump_length_forward; //todo better non arbitrary limit
                            }
                            // a cradle line should also be removed if there will no way to support it
                            if (idx >= cradle->config_->cradle_z_distance_layers_ + 1)
                            {
                                const Shape& actually_forbidden = volumes_.getAvoidance(
                                    minimum_area_to_be_supportable,
                                    layer_idx + idx - (cradle->config_->cradle_z_distance_layers_ + 1),
                                    (only_gracious_ || ! support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                    support_rests_on_model,
                                    true);
                                OpenLinesSet current_shortened;
                                current_shortened.push_back(shortened_lines_to_center[line_idx]);
                                too_short |= actually_forbidden.difference(current_shortened).length() < support_line_width;
                            }
                            if (! too_short && ! too_long_jump)
                            {
                                if (ordered_lines_to_center[angle_idx].empty())
                                {
                                    ordered_lines_to_center[angle_idx].push_back(closer);
                                    ordered_lines_to_center[angle_idx].push_back(further);
                                }
                                else
                                {
                                    // Prefer lines not touching the center. Otherwise, prefer the line closest to the center
                                    if (distance_from_center > FUDGE_LENGTH)
                                    {
                                        if (vSize(ordered_lines_to_center[angle_idx].front() - center) < FUDGE_LENGTH)
                                        {
                                            ordered_lines_to_center[angle_idx].clear();
                                            ordered_lines_to_center[angle_idx].push_back(closer);
                                            ordered_lines_to_center[angle_idx].push_back(further);
                                        }
                                        else if (distance_from_center < vSize(ordered_lines_to_center[angle_idx].front() - center))
                                        {
                                            ordered_lines_to_center[angle_idx].clear();
                                            ordered_lines_to_center[angle_idx].push_back(closer);
                                            ordered_lines_to_center[angle_idx].push_back(further);
                                        }
                                    }
                                }
                            }
                        }

                        for (auto [angle_idx, next_line] : ordered_lines_to_center | ranges::views::enumerate)
                        {
                            if (next_line.empty())
                            {
                                removed_directions[angle_idx] = true;
                                continue;
                            }
                            OpenPolyline line(next_line);
                            // Handle cradle_z_distance_layers by overwriting first element in the vector until valid distance is reached.
                            if (idx <= cradle->config_->cradle_z_distance_layers_ + 1 && ! cradle->lines_[angle_idx].empty())
                            {
                                cradle->lines_[angle_idx][0] = TreeSupportCradleLine(line, layer_idx + idx, cradle->config_->cradle_lines_roof_);
                            }
                            else
                            {
                                cradle->lines_[angle_idx].emplace_back(TreeSupportCradleLine(line, layer_idx + idx, cradle->config_->cradle_lines_roof_));
                            }
                        }
                    }
                }

                // enlarge cradle lines below to minimize overhang of cradle lines.
                for (auto [line_idx, cradle_lines] : cradle->lines_ | ranges::views::enumerate)
                {
                    if (! cradle_lines.empty())
                    {
                        Point2LL line_end = cradle_lines.back().line_.back();
                        for (auto [up_idx, line] : cradle_lines | ranges::views::enumerate | ranges::views::reverse)
                        {
                            Point2LL center = cradle->getCenter(line.layer_idx_);
                            if (vSize2(line_end - center) > vSize2(line.line_.back() - center))
                            {
                                OpenLinesSet line_extension;
                                line_extension.addSegment(line.line_.back(), line_end);
                                coord_t line_length_before = line_extension.length();
                                Shape actually_forbidden = volumes_.getAvoidance(
                                    0,
                                    line.layer_idx_,
                                    (only_gracious_ || ! support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                    support_rests_on_model,
                                    true);
                                line_extension = actually_forbidden.difference(line_extension);

                                if (line_extension.length() + EPSILON < line_length_before)
                                {
                                    for (auto line_part : line_extension)
                                    {
                                        bool front_closer = vSize2(line_part.front() - center) < vSize2(line_part.back() - center);
                                        Point2LL closer = front_closer ? line_part.front() : line_part.back();
                                        Point2LL further = front_closer ? line_part.back() : line_part.front();

                                        if (vSize2(closer - line.line_.back() < EPSILON * EPSILON))
                                        {
                                            line_end = further;
                                        }
                                    }
                                }
                                // As the center can move there is no guarantee that the point of the current line lies on the line below.
                                line_end = LinearAlg2D::getClosestOnLine(line_end, line.line_.front(), line.line_.back());
                                line.line_.back() = line_end;
                            }
                        }
                    }
                }
            }
        });
}


void SupportCradleGeneration::cleanCradleLineOverlaps()
{
    std::vector<std::vector<CradlePresenceInformation>> all_cradles_per_layer(cradle_data_.front().size());
    for(size_t mesh_idx = 0; mesh_idx < cradle_data_.size(); mesh_idx++)
    {
        for (LayerIndex layer_idx = 0; layer_idx < cradle_data_[mesh_idx].size(); layer_idx++)
        {
            for (size_t cradle_idx = 0; cradle_idx < cradle_data_[mesh_idx][layer_idx].size(); cradle_idx++)
            {
                for (size_t cradle_line_idx = 0; cradle_line_idx < cradle_data_[mesh_idx][layer_idx][cradle_idx]->lines_.size(); cradle_line_idx++)
                {
                    for (size_t height = 0; height < cradle_data_[mesh_idx][layer_idx][cradle_idx]->lines_[cradle_line_idx].size(); height++)
                    {
                        LayerIndex cradle_line_layer_idx = cradle_data_[mesh_idx][layer_idx][cradle_idx]->lines_[cradle_line_idx][height].layer_idx_;
                        if(all_cradles_per_layer.size() <= layer_idx + height)
                        {
                            all_cradles_per_layer.resize(cradle_data_[mesh_idx][layer_idx][cradle_idx]->config_->cradle_layers_ + 1);
                        }
                        all_cradles_per_layer[layer_idx + height].emplace_back(cradle_data_[mesh_idx][layer_idx][cradle_idx], cradle_line_layer_idx, cradle_line_idx);
                    }
                }
            }
        }
    }

    cura::parallel_for<coord_t>(
        1,
        all_cradles_per_layer.size(),
        [&](const LayerIndex layer_idx)
        {
            std::vector<CradlePresenceInformation>& all_cradles_on_layer = all_cradles_per_layer[layer_idx];

            std::function<void(size_t, Point2LL)> handleNewEnd = [&](size_t cradle_line_idx, Point2LL new_end)
            {
                TreeSupportCradleLine* cradle_line = all_cradles_on_layer[cradle_line_idx].getCradleLine();
                if (LinearAlg2D::pointIsProjectedBeyondLine(new_end, cradle_line->line_.front(), cradle_line->line_.back()) || vSize(new_end - cradle_line->line_.front()) < all_cradles_on_layer[cradle_line_idx].cradle_->config_->cradle_length_min_)
                {
                    cradle_line->addLineToRemoved(cradle_line->line_);
                    cradle_line->line_.clear();
                }
                else
                {
                    cradle_line->line_.back() = new_end;
                }
            };

            for (size_t cradle_idx = 0; cradle_idx < all_cradles_on_layer.size(); cradle_idx++)
            {
                TreeSupportCradleLine* cradle_line = all_cradles_on_layer[cradle_idx].getCradleLine();

                AABB bounding_box_outer = AABB(cradle_line->line_);
                for (size_t cradle_idx_inner = cradle_idx + 1; cradle_idx_inner < all_cradles_on_layer.size(); cradle_idx_inner++)
                {
                    TreeSupportCradleLine* cradle_line_inner = all_cradles_on_layer[cradle_idx_inner].getCradleLine();

                    const coord_t min_distance_between_lines
                        = std::max(all_cradles_on_layer[cradle_idx].cradle_->config_->cradle_line_width_, all_cradles_on_layer[cradle_idx_inner].cradle_->config_->cradle_line_width_)
                        + std::max(all_cradles_on_layer[cradle_idx].cradle_->config_->cradle_line_distance_, all_cradles_on_layer[cradle_idx_inner].cradle_->config_->cradle_line_distance_);

                    AABB bounding_box_current = bounding_box_outer;
                    bounding_box_current.expand(min_distance_between_lines);

                    if (cradle_line_inner->line_.empty() || cradle_line->line_.empty())
                    {
                        continue;
                    }
                    if (bounding_box_current.hit(AABB(cradle_line_inner->line_)))
                    {
                        OpenPolyline& outer_line = cradle_line->line_;
                        OpenPolyline& inner_line = cradle_line_inner->line_;
                        Point2LL intersect;
                        if (LinearAlg2D::lineLineIntersection(outer_line.front(), outer_line.back(), inner_line.front(), inner_line.back(), intersect)
                            && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, outer_line.front(), outer_line.back())
                            && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, inner_line.front(), inner_line.back()))
                        {
                            coord_t inner_intersect_dist = vSize(inner_line.front() - intersect);
                            coord_t outer_intersect_dist = vSize(outer_line.front() - intersect);

                            if (inner_intersect_dist > outer_intersect_dist)
                            {
                                // this does not ensure that the line ends will not touch. Line ends not touching is handled later
                                Point2LL new_end_inner = intersect + normal((inner_line.front() - intersect), min_distance_between_lines);
                                handleNewEnd(cradle_idx_inner, new_end_inner);
                            }
                            if (outer_intersect_dist > inner_intersect_dist)
                            {
                                Point2LL new_end_outer = intersect + normal((outer_line.front() - intersect), min_distance_between_lines);
                                handleNewEnd(cradle_idx, new_end_outer);
                            }
                        }

                        if (! outer_line.empty() && ! inner_line.empty())
                        {
                            // Touching lines have the same issue Lines touch if the end is to close to another line
                            Point2LL inner_direction = inner_line.back() - inner_line.front();
                            Point2LL outer_direction = outer_line.back() - outer_line.front();
                            double cosine = std::abs((dot(inner_direction, outer_direction)) / double(vSize(outer_direction) * vSize(inner_direction)));
                            // If both lines point in the same/opposite direction check that them being to close is not the end line of one to the start of the other
                            if (cosine < 0.99 || vSize2(outer_line.back() - inner_line.back()) + EPSILON * EPSILON < vSize2(outer_line.front() - inner_line.front()))
                            {
                                coord_t inner_end_to_outer_distance = sqrt(LinearAlg2D::getDist2FromLineSegment(outer_line.front(), inner_line.back(), outer_line.back()));
                                if (inner_end_to_outer_distance < min_distance_between_lines && inner_end_to_outer_distance < vSize(outer_line.front() - inner_line.front()))
                                {
                                    Point2LL new_end_inner
                                        = inner_line.back() + normal(inner_line.front() - inner_line.back(), min_distance_between_lines - inner_end_to_outer_distance);
                                    double error = min_distance_between_lines - sqrt(LinearAlg2D::getDist2FromLineSegment(outer_line.front(), new_end_inner, outer_line.back()));
                                    double error_correction_factor = 1.0 + error / double(min_distance_between_lines - inner_end_to_outer_distance);
                                    new_end_inner
                                        = inner_line.back()
                                        + normal(inner_line.front() - inner_line.back(), (min_distance_between_lines - inner_end_to_outer_distance) * error_correction_factor);
                                    handleNewEnd(cradle_idx_inner, new_end_inner);
                                }
                                else
                                {
                                    coord_t outer_end_to_inner_distance = sqrt(LinearAlg2D::getDist2FromLineSegment(inner_line.front(), outer_line.back(), inner_line.back()));
                                    if (outer_end_to_inner_distance < min_distance_between_lines && outer_end_to_inner_distance < vSize(outer_line.front() - inner_line.front()))
                                    {
                                        Point2LL new_end_outer
                                            = outer_line.back() + normal(outer_line.front() - outer_line.back(), min_distance_between_lines - outer_end_to_inner_distance);
                                        double error = min_distance_between_lines - sqrt(LinearAlg2D::getDistFromLine(new_end_outer, outer_line.front(), outer_line.back()));
                                        double error_correction_factor = 1.0 + error / double(min_distance_between_lines - outer_end_to_inner_distance);
                                        new_end_outer
                                            = outer_line.back()
                                            + normal(outer_line.front() - outer_line.back(), (min_distance_between_lines - outer_end_to_inner_distance) * error_correction_factor);
                                        handleNewEnd(cradle_idx, new_end_outer);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        });
    for(size_t mesh_idx = 0; mesh_idx < cradle_data_.size(); mesh_idx++)
    {
        cura::parallel_for<coord_t>(
            1,
            cradle_data_[mesh_idx].size(),
            [&](const LayerIndex layer_idx)
            {
                for (size_t cradle_idx = 0; cradle_idx < cradle_data_[mesh_idx][layer_idx].size(); cradle_idx++)
                {
                    TreeSupportCradle* cradle = cradle_data_[mesh_idx][layer_idx][cradle_idx];
                    const coord_t max_cradle_xy_distance = *std::max_element(cradle->config_->cradle_xy_distance_.begin(), cradle->config_->cradle_xy_distance_.end());

                    cradle->verifyLines();
                    // As cradle lines (causing lines below to be longer) may have been removed to prevent them intersecting, all cradle lines are now shortened again if required.
                    for (auto [line_idx, cradle_lines] : cradle->lines_ | ranges::views::enumerate)
                    {
                        if (! cradle_lines.empty())
                        {
                            Point2LL line_end = cradle_lines.back().line_.back();
                            Point2LL line_front_uppermost = cradle_lines.back().line_.front();

                            if (vSize2(line_end - cradle_lines.back().line_.front()) > cradle->config_->cradle_length_ * cradle->config_->cradle_length_)
                            {
                                coord_t current_cradle_xy_distance = cradle->config_->cradle_xy_distance_[cradle_lines.back().layer_idx_ - layer_idx];
                                coord_t current_cradle_length = cradle->config_->cradle_length_ + max_cradle_xy_distance - current_cradle_xy_distance;
                                cradle_lines.back().line_.back() = line_front_uppermost + normal(line_end - line_front_uppermost, current_cradle_length);
                                for (auto [up_idx, line] : cradle_lines | ranges::views::enumerate | ranges::views::reverse)
                                {
                                    Point2LL center = cradle->getCenter(line.layer_idx_);
                                    Point2LL line_back_inner = line.line_.back();
                                    Point2LL line_front_inner = line.line_.front();
                                    if (vSize2(line_back_inner - line_front_inner) > cradle->config_->cradle_length_ * cradle->config_->cradle_length_)
                                    {
                                        // As the center can move there is no guarantee that the point of the current line lies on the line below.
                                        Point2LL projected_line_end = LinearAlg2D::getClosestOnLine(line_end, line.line_.front(), line.line_.back());
                                        current_cradle_xy_distance = cradle->config_->cradle_xy_distance_[line.layer_idx_ - layer_idx];
                                        current_cradle_length = cradle->config_->cradle_length_ + max_cradle_xy_distance - current_cradle_xy_distance;
                                        if (vSize2(line_front_inner - projected_line_end) > current_cradle_length * current_cradle_length)
                                        {
                                            line.line_.back() = projected_line_end;
                                        }
                                        else
                                        {
                                            line.line_.back() = line_front_inner + normal(line_back_inner - line_front_inner, current_cradle_length);
                                        }
                                    }
                                    line_end = line.line_.back();
                                }
                            }
                        }
                    }
                }
            });
    }
}


void SupportCradleGeneration::generateCradleLineAreasAndBase(const SliceDataStorage& storage)
{

    std::mutex critical_support_free_areas_and_cradle_areas;

    for(size_t mesh_idx = 0; mesh_idx < cradle_data_.size(); mesh_idx++)
    {

        if(cradle_data_[mesh_idx].empty())
        {
            continue;

        }

        const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
        const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
        const size_t z_distance_top_layers = round_up_divide(mesh.settings.get<coord_t>("support_top_distance"), layer_height);
        const bool support_rests_on_model = mesh.settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
        const coord_t support_line_width = mesh.settings.get<coord_t>("support_line_width");
        const size_t support_roof_layers = mesh.settings.get<bool>("support_roof_enable") ? round_divide(mesh.settings.get<coord_t>("support_roof_height"), layer_height) : 0;
        const bool xy_overrides = mesh.settings.get<SupportDistPriority>("support_xy_overrides_z") == SupportDistPriority::XY_OVERRIDES_Z;
        const coord_t xy_distance = mesh.settings.get<coord_t>("support_xy_distance");
        const coord_t xy_min_distance = !xy_overrides ? mesh.settings.get<coord_t>("support_xy_distance_overhang") : xy_distance;
        const coord_t roof_outset = support_roof_layers > 0 ? mesh.settings.get<coord_t>("support_roof_offset") : 0;
        const bool support_moves = mesh.settings.get<ESupportStructure>("support_structure") != ESupportStructure::NORMAL;

        const coord_t max_roof_movement = support_moves ? TreeSupportSettings(mesh.settings).maximum_move_distance_slow : 0; //todo without TreeSupportSettings
        Simplify simplifyer(mesh.settings);

        cura::parallel_for<coord_t>(
            0,
            cradle_data_[mesh_idx].size(),
            [&](const LayerIndex layer_idx)
            {
                std::vector<TreeSupportCradle*> valid_cradles;
                for (size_t cradle_idx = 0; cradle_idx < cradle_data_[mesh_idx][layer_idx].size(); cradle_idx++)
                {
                    TreeSupportCradle& cradle = *cradle_data_[mesh_idx][layer_idx][cradle_idx];
                    const coord_t min_distance_between_lines = FUDGE_LENGTH + cradle.config_->min_distance_between_lines_areas_;
                    // Some angles needed to move cradle lines outwards to prevent them from toughing.
                    double center_angle = (2.0 * std::numbers::pi) / double(cradle.config_->cradle_line_count_);
                    double outer_angle = (std::numbers::pi - center_angle) / 2;
                    coord_t outer_radius = (double(min_distance_between_lines + support_line_width) / sin(center_angle)) * sin(outer_angle);
                    const coord_t small_hole_size = EPSILON + cradle.config_->min_distance_between_lines_areas_; // based on calculation in WallToolPath

                    for (size_t cradle_height = 0; cradle_height <= cradle.config_->cradle_layers_; cradle_height++)
                    {
                        std::vector<std::pair<Point2LL, Point2LL>> all_tips_center;
                        // generate trapezoid line tip with front width of support line width, back cradle_width.

                        for (size_t line_idx = 0; line_idx < cradle.lines_.size(); line_idx++)
                        {
                            std::optional<TreeSupportCradleLine*> line_opt = cradle.getCradleLineOfIndex(layer_idx + cradle_height, line_idx);
                            if (! line_opt)
                            {
                                continue;
                            }
                            TreeSupportCradleLine* cradle_line = line_opt.value();
                            OpenPolyline line = cradle_line->line_;

                            coord_t current_cradle_line_width = cradle.config_->cradle_line_width_;

                            double assumed_half_center_angle = std::numbers::pi / (1.5 * cradle.config_->cradle_line_count_);
                            coord_t triangle_length
                                = cradle.config_->cradle_line_count_ <= 2 ? 0 : ((current_cradle_line_width - support_line_width) / 2) * tan(std::numbers::pi / 2 - assumed_half_center_angle);

                            const coord_t line_length = line.length();
                            if (triangle_length >= line_length + cradle.config_->cradle_line_width_)
                            {
                                triangle_length = line_length + cradle.config_->cradle_line_width_;
                                current_cradle_line_width = support_line_width + 2 * triangle_length * tan(assumed_half_center_angle);
                            }

                            Point2LL direction = line.back() - line.front();
                            Point2LL center_front = line.front() - normal(direction, cradle.config_->cradle_line_width_ / 2);

                            Point2LL direction_up_center = normal(rotate(direction, std::numbers::pi / 2), support_line_width / 2);
                            Point2LL center_up = center_front + direction_up_center;
                            Point2LL center_down = center_front - direction_up_center;

                            coord_t tip_shift = 0;
                            for (auto existing_center : all_tips_center)
                            {
                                Point2LL intersect;
                                bool centers_touch = LinearAlg2D::lineLineIntersection(center_up, center_down, existing_center.first, existing_center.second, intersect)
                                                  && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, existing_center.first, existing_center.second)
                                                  && ! LinearAlg2D::pointIsProjectedBeyondLine(intersect, center_up, center_down);
                                if (centers_touch || std::min(vSize(center_down - existing_center.first), vSize(center_up - existing_center.second)) < min_distance_between_lines)
                                {
                                    // This cradle line is to close to another.
                                    // Move it back todo If line gets smaller than min length => abort
                                    coord_t tip_shift_here = outer_radius - vSize(center_front - cradle.getCenter(cradle_line->layer_idx_));
                                    tip_shift += tip_shift_here;
                                    center_front = center_front + normal(direction, tip_shift_here);
                                    center_up = center_front + direction_up_center;
                                    center_down = center_front - direction_up_center;
                                }
                            }

                            Point2LL back_center = center_front + normal(direction, triangle_length);
                            Point2LL direction_up_back = normal(rotate(direction, std::numbers::pi / 2), current_cradle_line_width / 2);

                            Point2LL back_up = back_center + direction_up_back;
                            Point2LL back_down = back_center - direction_up_back;

                            line.front() = back_center + normal(direction, current_cradle_line_width / 2 - FUDGE_LENGTH / 2);
                            all_tips_center.emplace_back(center_up, center_down);

                            Polygon line_tip;
                            line_tip.push_back(back_down);
                            if (current_cradle_line_width == cradle.config_->cradle_line_width_)
                            {
                                coord_t distance_end_front = line_length - triangle_length + cradle.config_->cradle_line_width_ - tip_shift;
                                Point2LL line_end_down = back_down + normal(direction, distance_end_front);
                                Point2LL line_end_up = back_up + normal(direction, distance_end_front);
                                line_tip.push_back(line_end_down);
                                line_tip.push_back(line_end_up);
                            }
                            line_tip.push_back(back_up);
                            line_tip.push_back(center_up);
                            line_tip.push_back(center_down);
                            if (line_tip.area() < 0)
                            {
                                line_tip.reverse();
                            }
                            cradle_line->area_.push_back(line_tip);

                            Shape anti_preferred = cradle_line->area_.offset(xy_distance);
                            std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                            for (size_t z_distance_idx = 0; z_distance_idx < z_distance_top_layers; z_distance_idx++)
                            {
                                volumes_.addAreaToAntiPreferred(anti_preferred, layer_idx + cradle_height + z_distance_idx);
                            }
                        }
                    }

                    bool is_side_cradle = cradle.cradle_placement_method_ == CradlePlacementMethod::MANUAL_SIDE || cradle.cradle_placement_method_ == CradlePlacementMethod::AUTOMATIC_SIDE;
                    Shape cradle_base = is_side_cradle ? Shape() : cradle.shadow_[0];

                    if (support_roof_layers && ! is_side_cradle)
                    {
                        Shape cut_line_base;
                        Shape first_cradle_areas;
                        if (cradle.config_->large_cradle_base_)
                        {
                            // If a large cradle base is used there needs to be a small hole cut into it to ensure that there will be a line for the pointy overhang to rest
                            // on. This line is just a diagonal though the original pointy overhang area (from min to max). Technically this line is covering more than just
                            // the overhang, but that should not cause any issues.
                            AABB cradle_base_aabb = AABB(cradle_base);
                            OpenLinesSet rest_line;
                            rest_line.addSegment(cradle_base_aabb.min_, cradle_base_aabb.max_);
                            cut_line_base = rest_line.offset(small_hole_size);
                        }
                        {
                            std::lock_guard<std::mutex> critical_section_cradle(critical_support_free_areas_and_cradle_areas);
                            for (size_t interface_down = 0; interface_down < layer_idx && interface_down < support_roof_layers; interface_down++)
                            {
                                support_free_areas_[layer_idx - interface_down].push_back(cut_line_base);
                            }
                        }
                        if (cradle.config_->large_cradle_base_)
                        {
                            cradle_base = cradle_base.offset(cradle.config_->cradle_support_base_area_radius_, ClipperLib::jtRound);
                            Shape center_removed = cradle_base.difference(cut_line_base);
                            if (center_removed.area() > 1)
                            {
                                cradle_base = center_removed;
                            }
                        }
                        else if (cradle.config_->cradle_base_roof_)
                        {
                            // collect all inner points and connect to center for thin cradle base
                            OpenLinesSet connected_cradle_base;
                            for (size_t line_idx = 0; line_idx < cradle.lines_.size(); line_idx++)
                            {
                                std::optional<TreeSupportCradleLine*> line_opt = cradle.getCradleLineOfIndex(layer_idx + cradle.config_->cradle_z_distance_layers_ + 1, line_idx);
                                if (line_opt)
                                {
                                    connected_cradle_base.addSegment(cradle.getCenter(line_opt.value()->layer_idx_), line_opt.value()->line_.front());
                                }
                            }
                            cradle_base = connected_cradle_base.offset(cradle.config_->cradle_line_width_ / 2 + EPSILON).unionPolygons(cradle_base);
                        }
                    }

                    cradle_base = cradle_base.unionPolygons();

                    if (cradle.config_->cradle_lines_roof_)
                    {
                        Shape forbidden_here = volumes_.getAvoidance(
                            0,
                            layer_idx,
                            (only_gracious_ || ! support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                            support_rests_on_model,
                            ! xy_overrides);
                        std::vector<std::pair<Shape, int32_t>> roofs;

                        if(cradle.is_roof_)
                        {
                            roofs.emplace_back(cradle_base, -1);
                        }

                        for (size_t line_idx = 0; line_idx < cradle.lines_.size(); line_idx++)
                        {
                            std::optional<TreeSupportCradleLine*> line_opt = cradle.getCradleLineOfIndex(layer_idx + cradle.config_->cradle_z_distance_layers_ + 1, line_idx);
                            if (! line_opt)
                            {
                                continue;
                            }

                            coord_t line_base_offset = cradle.config_->large_cradle_base_ ? std::max(coord_t(0), cradle.config_->cradle_support_base_area_radius_ - cradle.config_->cradle_line_width_ / 2) : 0;
                            roofs.emplace_back(line_opt.value()->area_.offset(line_base_offset, ClipperLib::jtRound), line_idx);
                        }


                        for (auto roof_area_pair : roofs)
                        {
                            Shape full_overhang_area = TreeSupportUtils::safeOffsetInc(
                                roof_area_pair.first,
                                roof_outset,
                                forbidden_here,
                                xy_min_distance * 2,
                                0,
                                1,
                                support_line_width,
                                &simplifyer);
                            Shape roof_area_before = full_overhang_area;
                            for (LayerIndex dtt_roof = 0; dtt_roof <= support_roof_layers && layer_idx - dtt_roof >= 1; dtt_roof++)
                            {
                                const Shape forbidden_next = volumes_.getAvoidance(
                                    0,
                                    layer_idx - (dtt_roof + 1),
                                    (only_gracious_ || ! support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                    support_rests_on_model,
                                    ! xy_overrides);

                                full_overhang_area = full_overhang_area.difference(forbidden_next);

                                if (full_overhang_area.area() > EPSILON && dtt_roof < support_roof_layers)
                                {
                                    if (roof_area_pair.second != -1) // If is_line
                                    {
                                        TreeSupportCradleLine roof_base_line(cradle.lines_[roof_area_pair.second].front());
                                        roof_base_line.area_ = full_overhang_area;
                                        roof_base_line.is_base_ = true;
                                        roof_base_line.layer_idx_ = layer_idx - dtt_roof;
                                        cradle.lines_[roof_area_pair.second].emplace_front(roof_base_line);
                                    }
                                    else
                                    {
                                        if (dtt_roof < cradle.base_below_.size())
                                        {
                                            cradle.base_below_[dtt_roof].push_back(full_overhang_area);
                                        }
                                        else
                                        {
                                            cradle.base_below_.emplace_back(full_overhang_area);
                                        }
                                    }
                                    const Shape forbidden_before = volumes_.getAvoidance(
                                        0,
                                        layer_idx - dtt_roof,
                                        (only_gracious_ || ! support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                                        support_rests_on_model,
                                        ! xy_overrides);

                                    Shape supported_by_roof_below = TreeSupportUtils::safeOffsetInc( //todo better safeStepOffset values to improve performance
                                        full_overhang_area,
                                        max_roof_movement,
                                        forbidden_before,
                                        2 * xy_min_distance,
                                        0,
                                        1,
                                        support_line_width,
                                        &simplifyer);
                                    Shape overhang_part = roof_area_before.difference(supported_by_roof_below);
                                    if (overhang_part.area() > EPSILON)
                                    {
                                        OverhangInformation cradle_overhang(overhang_part, false, cradle_data_[mesh_idx][layer_idx][cradle_idx], layer_idx - dtt_roof, roof_area_pair.second);
                                        cradle.overhang_[layer_idx - dtt_roof].emplace_back(cradle_overhang);
                                    }
                                }
                                else
                                {

                                    if(dtt_roof == 0 && roof_area_pair.second < 0) // There was no roof base!
                                    {
                                        cradle.is_roof_ = false; //Try a regular base
                                    }
                                    else if (roof_area_before.area() > 1)
                                    {
                                        LayerIndex line_layer_idx
                                            = roof_area_pair.second < 0 ? LayerIndex(-1) : cradle_data_[mesh_idx][layer_idx][cradle_idx]->lines_[roof_area_pair.second].front().layer_idx_;
                                        OverhangInformation cradle_overhang(roof_area_before, true, cradle_data_[mesh_idx][layer_idx][cradle_idx], line_layer_idx, roof_area_pair.second);
                                        cradle.overhang_[layer_idx - dtt_roof].emplace_back(cradle_overhang);
                                    }
                                    break;
                                }
                                roof_area_before = full_overhang_area;
                            }
                        }
                    }

                    if(!cradle.is_roof_)
                    {
                        Shape forbidden_here = volumes_.getAvoidance(
                            0,
                            layer_idx,
                            (only_gracious_ || ! support_rests_on_model) ? AvoidanceType::FAST : AvoidanceType::COLLISION,
                            support_rests_on_model,
                            ! xy_overrides);

                        coord_t offset_radius = cradle.config_->cradle_support_base_area_radius_;
                        Shape cradle_base_offset = cradle_base.offset(offset_radius, ClipperLib::jtRound);
                        if (cradle_base_offset.difference(forbidden_here).area() > 1)
                        {
                            OverhangInformation cradle_overhang(support_moves ? cradle_base : cradle_base_offset, false, cradle_data_[mesh_idx][layer_idx][cradle_idx]);
                            cradle.overhang_[layer_idx].emplace_back(cradle_overhang);
                        }
                    }
                    if(!cradle.config_->cradle_lines_roof_)
                    {
                        for (size_t line_idx = 0; line_idx < cradle.lines_.size(); line_idx++)
                        {
                            if (! cradle.lines_[line_idx].empty())
                            {
                                LayerIndex support_cradle_on_layer_idx = cradle.lines_[line_idx].front().layer_idx_ - (cradle.config_->cradle_z_distance_layers_ + 1);
                                OverhangInformation line_overhang(
                                    cradle.lines_[line_idx].front().area_,
                                    false,
                                    cradle_data_[mesh_idx][layer_idx][cradle_idx],
                                    cradle.lines_[line_idx].front().layer_idx_,
                                    line_idx);
                                cradle.overhang_[support_cradle_on_layer_idx].emplace_back(line_overhang);
                            }
                        }
                    }
                    if (! cradle.overhang_.empty())
                    {
                        valid_cradles.emplace_back(cradle_data_[mesh_idx][layer_idx][cradle_idx]);
                    }
                    else
                    {
                        delete cradle_data_[mesh_idx][layer_idx][cradle_idx];
                    }
                }
                cradle_data_[mesh_idx][layer_idx] = valid_cradles;
            });
    }
}


void SupportCradleGeneration::addMeshToCradleCalculation(const SliceDataStorage& storage, size_t mesh_idx)
{
    const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
    if (! retrieveSetting<bool>(mesh.settings, "support_tree_cradle_enable"))
    {
        return;
    }
    calculateFloatingParts(storage, mesh_idx);
}

void SupportCradleGeneration::generateCradleForMesh(const SliceDataStorage& storage, size_t mesh_idx)
{
    const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
    if (! retrieveSetting<bool>(mesh.settings, "support_tree_cradle_enable"))
    {
        return;
    }
    if (floating_parts_cache_[mesh_idx].empty())
    {
        addMeshToCradleCalculation(storage, mesh_idx);
    }
    std::vector<std::vector<TreeSupportCradle*>> cradle_data_mesh = generateCradleCenters(mesh, mesh_idx);
    generateCradleLines(cradle_data_mesh, mesh);
    if(cradle_data_[mesh_idx].size() < cradle_data_mesh.size())
    {
        cradle_data_[mesh_idx].resize(cradle_data_mesh.size());
    }
    for (auto [layer_idx, cradles_on_layer] :cradle_data_mesh | ranges::views::enumerate)
    {
        cradle_data_[mesh_idx][layer_idx].insert(cradle_data_[mesh_idx][layer_idx].end(), cradles_on_layer.begin(), cradles_on_layer.end());
    }
}

void SupportCradleGeneration::generate(const SliceDataStorage& storage)
{
    cleanCradleLineOverlaps();
    generateCradleLineAreasAndBase(storage);
}

void SupportCradleGeneration::pushCradleData(std::vector<std::vector<TreeSupportCradle*>>& target, std::vector<Shape>& support_free_areas, size_t mesh_idx)
{

    if(target.size() < cradle_data_[mesh_idx].size())
    {
        target.resize(cradle_data_[mesh_idx].size());
    }

    for (auto [layer_idx, cradles_on_layer] :cradle_data_[mesh_idx] | ranges::views::enumerate)
    {
        target[layer_idx].insert(target[layer_idx].end(), cradles_on_layer.begin(), cradles_on_layer.end());
    }

    if(support_free_areas.size() < support_free_areas_.size())
    {
        support_free_areas.resize(support_free_areas_.size());
    }
    for (auto [layer_idx, support_free_on_layer] : support_free_areas_ | ranges::views::enumerate)
    {
        support_free_areas[layer_idx].push_back(support_free_on_layer);
    }
}

} //End Namespace