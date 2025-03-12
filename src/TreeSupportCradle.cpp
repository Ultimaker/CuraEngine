
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
    for (int64_t direction_idx = 0; direction_idx < size; direction_idx++)
    {
        Point2LL direction(direction_idx > size / 2 ? std::abs(int64_t(size) - direction_idx) : direction_idx, direction_idx - size / 2);
        double dot_a = dot(vector_direction, direction);
        double det_a = cross(vector_direction, direction);
        double angle_vd = std::atan2(det_a, dot_a);
        if (angle_vd < 0)
        {
            angle_vd += 2 * std::numbers::pi;
        }
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
    const double deformation_constant = retrieveSetting<double>(mesh.settings, "support_tree_part_deformation_constant");
    const coord_t min_wall_line_width = mesh.settings.get<coord_t>("min_wall_line_width");

    coord_t extent_x = 2*std::max(min_wall_line_width / 2, minimum_box.extent.X);
    coord_t extent_y = 2*std::max(min_wall_line_width / 2, minimum_box.extent.Y);

    Point2LL axis_min = extent_x < extent_y ? minimum_box.axis[0] : minimum_box.axis[1];
    Point2LL axis_max = extent_x < extent_y ? minimum_box.axis[1] : minimum_box.axis[0];

    for (int64_t direction_idx = 0; direction_idx < deform_part.size(); direction_idx++)
    {
        Point2LL direction(direction_idx > deform_part.size() / 2 ? std::abs(int64_t(deform_part.size()) - direction_idx) : direction_idx, direction_idx - deform_part.size() / 2);
        assert(direction_idx == getDirectionIdx(Point2LL(0, 0), direction));
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
        double percent_angle = 1 - angle / (std::numbers::pi / 2.0);

        double assumed_thickness_in_direction
            = percent_angle * std::min(assumed_part_thickness, double(std::min(extent_x, extent_y))) + (1.0 - percent_angle) * double(std::max(extent_x, extent_y));
        double assumed_thickness_opposite_direction
            = percent_angle * double(std::max(extent_x, extent_y)) + (1.0 - percent_angle) * std::min(assumed_part_thickness, double(std::min(extent_x, extent_y)));

        deform_part[direction_idx] = (deformation_constant / (std::pow(assumed_thickness_in_direction, 1) * (std::pow(assumed_thickness_opposite_direction , 0.33))));
    }
}

std::pair<MinimumBoundingBox, CradleDeformationHalfCircle> SupportCradleGeneration::getSimulatedConnection(const SliceMeshStorage& mesh,
                                                                                                           double assumed_part_thickness,
                                                                                                           std::set<UnsupportedAreaInformation*> elements)
{

    if(elements.empty())
    {
        return std::pair<MinimumBoundingBox,CradleDeformationHalfCircle>(MinimumBoundingBox(), CradleDeformationHalfCircle());
    }

    UnsupportedAreaInformation* random_area = *elements.begin();

    if(elements.size() == 1)
    {
        return std::pair<MinimumBoundingBox,CradleDeformationHalfCircle>(random_area->min_box, random_area->deformation);
    }

    {
        std::lock_guard<std::mutex> critical_section(*critical_simulated_connection_cache_);
        for(auto& cache_elements:simulated_connection_cache_[random_area->layer_idx])
        {
            if(cache_elements.first == elements)
            {
                return cache_elements.second;
            }
        }
    }

    Shape simulated_connection;
    // One cant be sure that the center of the min box is actually inside the polygon, e.g. a U shape
    // This may cause some inaccuracies, but better than issues later on.
    Point2LL current_inside = random_area->min_box.center;
    if (! random_area->area.inside(current_inside))
    {
        PolygonUtils::moveInside(random_area->area, current_inside);
    }

    for (UnsupportedAreaInformation* element_connect : elements)
    {
        if(random_area == element_connect)
        {
            continue;
        }
        OpenLinesSet line;
        Point2LL connect_inside = element_connect->min_box.center;
        if (! element_connect->area.inside(connect_inside))
        {
            PolygonUtils::moveInside(element_connect->area, connect_inside);
        }
        line.addSegment(current_inside, connect_inside);
        //todo that line thickness calculation needs improvement.
        simulated_connection.push_back(line.offset(std::max(
            std::min(random_area->min_box.extent.X, random_area->min_box.extent.Y),
            std::min(element_connect->min_box.extent.X, element_connect->min_box.extent.Y))));
        simulated_connection.push_back(element_connect->area);
    }

    simulated_connection.push_back(random_area->area);
    simulated_connection = simulated_connection.unionPolygons();
    MinimumBoundingBox simulated_connect_box(simulated_connection.getOutsidePolygons().splitIntoParts()[0]);
    CradleDeformationHalfCircle simulated_deform_part = {};
    getLayerDeformation(mesh, simulated_connect_box, assumed_part_thickness, simulated_deform_part);
    std::pair<MinimumBoundingBox, CradleDeformationHalfCircle> result(simulated_connect_box, simulated_deform_part);
    std::lock_guard<std::mutex> critical_section(*critical_simulated_connection_cache_);
    simulated_connection_cache_[random_area->layer_idx].emplace_back(elements, result);
    return result;

}

double SupportCradleGeneration::getTotalDeformation(size_t mesh_idx, const SliceMeshStorage& mesh, UnsupportedAreaInformation* element)
{
    if (element->deformation_total_calculated >= 0)
    {
        return element->deformation_total_calculated;
    }
    const size_t cradle_deformation_half_circle_size = CradleDeformationHalfCircle().size();
    const coord_t part_stable_radius = retrieveSetting<coord_t>(mesh.settings, "support_tree_part_deformation_diameter") / 2;
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
    std::set<UnsupportedAreaInformation*> iterate_elements;

    // todo instead of doing a single maximum, do a window over a few mm ?

    double largest_deformation_in_z_direction_deformation = 0;
    double largest_deformation_in_xy_direction_deformation = 0;

    for(LayerIndex iterate_layer_idx = 0; iterate_layer_idx <= layer_idx; iterate_layer_idx++)
    {
        std::set<UnsupportedAreaInformation*> next_iterate_elements;
        iterate_elements.insert(root_areas[iterate_layer_idx].begin(),root_areas[iterate_layer_idx].end());


        if(!iterate_elements.empty())
        {
            double deformation_limit_total_layer = -1;
            double deformation_limit_z_layer = -1;
            CradleDeformationHalfCircle deformation_layer;
            MinimumBoundingBox min_box_layer;

            for(UnsupportedAreaInformation* current_area : iterate_elements) //todo why do i iterate over all? It is either one or a simulated connection...
            {
                if(min_box_layer.center == Point2LL(0,0))
                {
                    deformation_layer = current_area->deformation;
                }

                //basically just a random init. If there are multiple present the simulated connection below will deal with it anyway.
                if(current_area->min_box.area > min_box_layer.area)
                {
                    min_box_layer = current_area->min_box;
                }

                for (UnsupportedAreaInformation* element_above : current_area->areas_above)
                {
                    if (elements_on_path_down.contains(element_above))
                    {
                        next_iterate_elements.emplace(element_above);
                    }
                }
                deformation_limit_total_layer = std::max(deformation_limit_total_layer, current_area->deformation_limit_total);
                deformation_limit_z_layer = std::max(deformation_limit_z_layer, current_area->deformation_limit_z);
                deformation_layer = elementWiseSelect(
                    current_area->deformation,
                    deformation_layer,
                    [](double a, double b)
                    {
                        return std::min(a, b);
                    });
            }
            if (iterate_elements.size() > 1)
            {

                std::pair<MinimumBoundingBox, CradleDeformationHalfCircle> simulated_connect_result = getSimulatedConnection(mesh, part_stable_radius, iterate_elements);

                deformation_layer = elementWiseSelect(
                    simulated_connect_result.second,
                    deformation_layer,
                    [](double a, double b)
                    {
                        return std::min(a, b);
                    });

                min_box_layer = simulated_connect_result.first;
            }

            for (size_t direction_idx = 0; direction_idx < deformation_layer.size(); direction_idx++)
            {
                double deformation_in_xy_direction = std::pow(deformation_layer[direction_idx] * double(layer_idx - iterate_layer_idx) / layer_per_mm, 3);
                if (deformation_in_xy_direction > largest_deformation_in_xy_direction_deformation)
                {
                    largest_deformation_in_xy_direction_deformation = deformation_in_xy_direction;
                }
            }

            // Estimate horizontal influence to deformation
            double horizontal_distance = min_box_layer.minimumDistance(element->min_box) * horizontal_movement_weight;
            size_t direction_z_idx = getDirectionIdx(element->min_box.center, min_box_layer.center);
            double deformation_in_z_direction = std::pow(deformation_layer[direction_z_idx] * double(horizontal_distance / layer_height) / layer_per_mm, 3);

            if (deformation_in_z_direction > largest_deformation_in_z_direction_deformation)
            {
                largest_deformation_in_z_direction_deformation = deformation_in_z_direction;
            }
            if(deformation_limit_z_layer > 0 && deformation_limit_z_layer < largest_deformation_in_z_direction_deformation)
            {
                largest_deformation_in_z_direction_deformation = std::min(largest_deformation_in_z_direction_deformation, deformation_limit_z_layer);
            }
            if(deformation_limit_total_layer > 0 && deformation_limit_total_layer < largest_deformation_in_z_direction_deformation + largest_deformation_in_xy_direction_deformation)
            {
                largest_deformation_in_z_direction_deformation = std::min(largest_deformation_in_z_direction_deformation, deformation_limit_total_layer);
                largest_deformation_in_xy_direction_deformation = std::min(largest_deformation_in_xy_direction_deformation, deformation_limit_total_layer);
            }
        }
        iterate_elements = next_iterate_elements;
    }

    double result_deformation_xy = largest_deformation_in_xy_direction_deformation;
    double result_deformation_z = largest_deformation_in_z_direction_deformation;
    element->deformation_total_calculated = result_deformation_xy + result_deformation_z;
    return element->deformation_total_calculated;
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
                bool rests_on_material = ! PolygonUtils::clipPolygonWithAABB(storage.getLayerOutlines(layer_idx - 1, false, false), part_aabb).intersection(part).empty();
                Shape overhang = PolygonUtils::clipPolygonWithAABB(mesh.overhang_areas[layer_idx], part_aabb).intersection(part);
                coord_t overhang_area = overhang.area();
                MinimumBoundingBox minimum_box(part);

                if (! rests_on_material || layer_idx == start_layer)
                {
                    overhang_area = std::max(overhang_area, coord_t(std::numbers::pi * min_wall_line_width * min_wall_line_width));
                    UnsupportedAreaInformation* area_info = new UnsupportedAreaInformation(part, layer_idx, 0, overhang_area, CradleDeformationHalfCircle{}, minimum_box);
                    bool add_manual_cradle = false;
                    if (! rests_on_material)
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
                    bool add_automatic_cradle = ! rests_on_material && overhang_area < cradle_area_threshold;
                    if (add_manual_cradle)
                    {
                        area_info->support_required = CradlePlacementMethod::MANUAL_POINTY;
                        top_most_cradle_layer_ = std::max(layer_idx + cradle_layers + 1, top_most_cradle_layer_);
                    }
                    else if (add_automatic_cradle)
                    {
                        area_info->support_required = CradlePlacementMethod::AUTOMATIC_POINTY;
                        top_most_cradle_layer_ = std::max(layer_idx + cradle_layers + 1, top_most_cradle_layer_);
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
                    for (LayerIndex scan_down_layer_idx = layer_idx - 1; scan_down_layer_idx > 0 && (layer_idx - scan_down_layer_idx) * layer_height < deform_radius;
                         scan_down_layer_idx--)
                    {
                        if (PolygonUtils::clipPolygonWithAABB(mesh.layers[scan_down_layer_idx].getOutlines(), part_aabb).intersection(part).empty())
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
                        top_most_cradle_layer_ = std::max(layer_idx + cradle_layers + 1, top_most_cradle_layer_);
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
                        LayerIndex last_cradle_at_layer_idx = -1;
                        for(auto& cradle : cradles_below)
                        {
                            last_cradle_at_layer_idx = std::max(last_cradle_at_layer_idx, cradle->layer_idx);
                        }
                        if(last_cradle_at_layer_idx != -1 && layer_idx - last_cradle_at_layer_idx < cradle_layers) // Assume any cradle reaches full height. This can be wrong! TODO
                        {
                            area_info->deformation_limit_total = EPSILON;
                        }
                        else if(overhang_area > EPSILON)
                        {
                            area_info->deformation_limit_z = EPSILON;
                        }
                        else if(area_info->support_required == CradlePlacementMethod::NONE && layer_idx % (std::max(coord_t(1),5000/layer_height)) == 0) // Only check total deformation every 5mm
                        {

                            double estimated_deformation = getTotalDeformation(mesh_idx, mesh, area_info);
                            if(estimated_deformation > side_cradle_support_threshold) // todo additional cradle if it rests on cradle earlier
                            {
                                top_most_cradle_layer_ = std::max(layer_idx + cradle_layers + 1, top_most_cradle_layer_);
                                area_info->support_required = CradlePlacementMethod::AUTOMATIC_SIDE;
                                area_info->deformation_limit_total = EPSILON;
                            }

                        }
                    }
                }
            });
    }
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
                    unsupported_model.clear();
                    unsupported_model.emplace_back(cradle_0);
                    delete cradle_main;
                }
                else
                {
                    cradle_main->shadow_ = accumulated_model;
                    cradle_main->part_outline_ = unsupported_model;
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
    coord_t maximum_move_distance_slow = 0;
    if(support_moves)
    {
        TreeSupportSettings config(mesh.settings);
        maximum_move_distance_slow = config.maximum_move_distance_slow;
    }

    const coord_t minimum_area_to_be_supportable = support_moves ? mesh.settings.get<coord_t>("support_tree_tip_diameter") / 2 : 0;
    cura::parallel_for<coord_t>(
        1,
        cradle_data_mesh.size(),
        [&](const LayerIndex layer_idx)
        {
            for (auto [center_idx, cradle] : cradle_data_mesh[layer_idx] | ranges::views::enumerate)
            {
                const coord_t max_cradle_jump_length_forward = cradle->config_->cradle_length_ / 3;
                coord_t center_move_distance = support_moves ? std::min(maximum_move_distance_slow, cradle->config_->cradle_line_width_ / 3) : cradle->config_->cradle_line_width_ / 3;
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
                        //  To prevent this offset by the delta before removing the influence of the model.
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

                        // Create lines that go from the furthest possible location to the center or model outline
                        OpenLinesSet lines_to_center;
                        Shape model_shadow_outer_point_outline = model_shadow.offset(current_cradle_length + current_cradle_xy_distance);
                        std::unique_ptr<LocToLineGrid> loc_to_grid_outer = PolygonUtils::createLocToLineGrid(model_shadow_outer_point_outline, 1000);
                        for (Point2LL outer : max_outer_points)
                        {
                            Point2LL direction = outer - center;
                            Point2LL inner = center + normal(direction, support_line_width);

                            // Space out the cradle lines by estimating where they should end and placing the line so that it directly goes towards the model instead of the center.
                            if (! cradle->config_->cradle_towards_center_)
                            {
                                Point2LL point_on_outer_outline = outer;
                                PolygonUtils::lineSegmentPolygonsIntersection(
                                    inner,
                                    outer,
                                    model_shadow_outer_point_outline,
                                    *loc_to_grid_outer,
                                    point_on_outer_outline,
                                    sqrt(max_distance2) + current_cradle_length * 2.0);

                                size_t angle_idx = cradle->getIndexForLineEnd(outer, layer_idx + idx);
                                Shape part_outline = cradle->part_outline_[idx];

                                // Place inner line-end on the outline closest to the point_on_outer_outline, and correct it if this would be causing a too large jump compared to the cradle line below.
                                inner = LinearAlg2D::getClosestOnLine(point_on_outer_outline, inner, outer);;
                                PolygonUtils::moveInside(part_outline.empty() ? model_shadow : part_outline, inner);
                                if (idx > 0 && idx > cradle->config_->cradle_z_distance_layers_ + 1 && ! cradle->lines_[angle_idx].empty())
                                {
                                    coord_t distance_from_line = LinearAlg2D::getDistFromLine(inner, cradle->lines_[angle_idx].back().line_[0], cradle->lines_[angle_idx].back().line_[1]);
                                    if (distance_from_line > center_move_distance)
                                    {
                                        Point2LL closest_to_outline_on_prev = LinearAlg2D::getClosestOnLine(inner, cradle->lines_[angle_idx].back().line_[0], cradle->lines_[angle_idx].back().line_[1]);
                                        Point2LL direction_closest_on_outline = inner - closest_to_outline_on_prev;
                                        inner = closest_to_outline_on_prev + normal(direction_closest_on_outline, center_move_distance);
                                    }
                                }
                            }
                            lines_to_center.addSegment(inner, outer);
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
                            coord_t distance2_from_cradle_line = LinearAlg2D::getDist2FromLine(line.line_.back(), line_end, line.line_.front());
                            coord_t distance2_from_cradle_segment = LinearAlg2D::getDist2FromLineSegment(line_end, line.line_.back(), line.line_.front());

                            if (distance2_from_cradle_segment > distance2_from_cradle_line)
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

        const coord_t max_roof_movement = support_moves ? TreeSupportSettings(mesh.settings).maximum_move_distance_slow : 0;
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
                                    coord_t tip_shift_here = (centers_touch && !cradle.config_->cradle_towards_center_)? min_distance_between_lines : outer_radius - vSize(center_front - cradle.getCenter(cradle_line->layer_idx_));
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