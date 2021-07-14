//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "TreeSupport.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h"
#include "settings/types/AngleRadians.h" //Creating the correct branch angles.
#include "settings/types/Ratio.h"
#include "utils/IntPoint.h" //To normalize vectors.
#include "utils/logoutput.h"
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/MinimumSpanningTree.h" //For connecting the correct nodes together to form an efficient tree.
#include "utils/polygon.h" //For splitting polygons into parts.
#include "utils/polygonUtils.h" //For moveInside.

#define SQRT_2 1.4142135623730950488 //Square root of 2.
#define CIRCLE_RESOLUTION 10 //The number of vertices in each circle.

//The various stages of the process can be weighted differently in the progress bar.
//These weights are obtained experimentally.
#define PROGRESS_WEIGHT_DROPDOWN 50 //Dropping down support.
#define PROGRESS_WEIGHT_AREAS 1 //Creating support areas.

namespace cura
{

TreeSupport::TreeSupport(const SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    const coord_t xy_distance = mesh_group_settings.get<coord_t>("support_xy_distance");
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const AngleRadians angle = mesh_group_settings.get<AngleRadians>("support_tree_angle");
    const coord_t maximum_move_distance
        = (angle < TAU / 4) ? (coord_t)(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max();
    const coord_t radius_sample_resolution = mesh_group_settings.get<coord_t>("support_tree_collision_resolution");

    volumes_ = ModelVolumes(storage, xy_distance, maximum_move_distance, radius_sample_resolution);
}

void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    const Settings& group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const bool global_use_tree_support =
        group_settings.get<bool>("support_enable")&&
        group_settings.get<ESupportStructure>("support_structure") == ESupportStructure::TREE;

    if (!(global_use_tree_support ||
          std::any_of(storage.meshes.cbegin(),
                      storage.meshes.cend(),
                      [](const SliceMeshStorage& m) {
                          return m.settings.get<bool>("support_enable") &&
                                 m.settings.get<ESupportStructure>("support_structure") == ESupportStructure::TREE;
                      })))
    {
        return;
    }

    std::vector<std::vector<Node*>> contact_nodes(storage.support.supportLayers.size()); //Generate empty layers to store the points in.
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.settings.get<ESupportStructure>("support_structure") == ESupportStructure::TREE)
        {
            generateContactPoints(mesh, contact_nodes);
        }
    }

    //Drop nodes to lower layers.
    dropNodes(contact_nodes);

    //Generate support areas.
    drawCircles(storage, contact_nodes);

    for (auto& layer : contact_nodes)
    {
        for (Node* p_node : layer)
        {
            delete p_node;
        }
        layer.clear();
    }
    contact_nodes.clear();

    storage.support.generated = true;
}

void TreeSupport::drawCircles(SliceDataStorage& storage, const std::vector<std::vector<Node*>>& contact_nodes)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t branch_radius = mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2;
    const size_t wall_count = mesh_group_settings.get<size_t>("support_wall_count");
    Polygon branch_circle; //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
    for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
    {
        const AngleRadians angle = static_cast<double>(i) / CIRCLE_RESOLUTION * TAU;
        branch_circle.emplace_back(cos(angle) * branch_radius, sin(angle) * branch_radius);
    }
    const coord_t circle_side_length = 2 * branch_radius * sin(M_PI / CIRCLE_RESOLUTION); //Side length of a regular polygon.
    const coord_t z_distance_bottom = mesh_group_settings.get<coord_t>("support_bottom_distance");
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const size_t z_distance_bottom_layers = round_up_divide(z_distance_bottom, layer_height) > 0 ? round_up_divide(z_distance_bottom, layer_height) : 1;
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    const double diameter_angle_scale_factor = sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t line_width = mesh_group_settings.get<coord_t>("support_line_width");
    const coord_t resolution = mesh_group_settings.get<coord_t>("support_tree_collision_resolution");
    size_t completed = 0; //To track progress in a multi-threaded environment.
#pragma omp parallel for shared(storage, contact_nodes)
    // Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
    for (int layer_nr = 0; layer_nr < static_cast<int>(contact_nodes.size()); layer_nr++)
    {
        Polygons support_layer;
        Polygons& roof_layer = storage.support.supportLayers[layer_nr].support_roof;

        //Draw the support areas and add the roofs appropriately to the support roof instead of normal areas.
        for (const Node* p_node : contact_nodes[layer_nr])
        {
            const Node& node = *p_node;

            Polygon circle;
            const double scale = static_cast<double>(node.distance_to_top + 1) / tip_layers;
            for (Point corner : branch_circle)
            {
                if (node.distance_to_top < tip_layers) //We're in the tip.
                {
                    const int mul = node.skin_direction ? 1 : -1;
                    corner = Point(corner.X * (0.5 + scale / 2) + mul * corner.Y * (0.5 - scale / 2),
                                   mul * corner.X * (0.5 - scale / 2) + corner.Y * (0.5 + scale / 2));
                }
                else
                {
                    corner = corner * (1 + static_cast<double>(node.distance_to_top - tip_layers) * diameter_angle_scale_factor);
                }
                circle.add(node.position + corner);
            }
            if (node.support_roof_layers_below > 0)
            {
                roof_layer.add(circle);
            }
            else
            {
                support_layer.add(circle);
            }
        }
        support_layer = support_layer.unionPolygons();
        roof_layer = roof_layer.unionPolygons();
        const size_t z_collision_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(z_distance_bottom_layers) + 1)); //Layer to test against to create a Z-distance.
        support_layer = support_layer.difference(volumes_.getCollision(0, z_collision_layer)); //Subtract the model itself (sample 0 is with 0 diameter but proper X/Y offset).
        roof_layer = roof_layer.difference(volumes_.getCollision(branch_radius, z_collision_layer));
        support_layer = support_layer.difference(roof_layer);
        //We smooth this support as much as possible without altering single circles. So we remove any line less than the side length of those circles.
        const double diameter_angle_scale_factor_this_layer = static_cast<double>(storage.support.supportLayers.size() - layer_nr - tip_layers) * diameter_angle_scale_factor; //Maximum scale factor.
        support_layer.simplify(circle_side_length * (1 + diameter_angle_scale_factor_this_layer), resolution); //Don't deviate more than the collision resolution so that the lines still stack properly.

        //Subtract support floors.
        if (mesh_group_settings.get<bool>("support_bottom_enable"))
        {
            Polygons& floor_layer = storage.support.supportLayers[layer_nr].support_bottom;
            const coord_t support_interface_resolution = mesh_group_settings.get<coord_t>("support_interface_skip_height");
            const size_t support_interface_skip_layers = round_up_divide(support_interface_resolution, layer_height);
            const coord_t support_bottom_height = mesh_group_settings.get<coord_t>("support_bottom_height");
            const size_t support_bottom_height_layers = round_up_divide(support_bottom_height, layer_height);
            for(size_t layers_below = 0; layers_below < support_bottom_height_layers; layers_below += support_interface_skip_layers)
            {
                const size_t sample_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(layers_below) - static_cast<int>(z_distance_bottom_layers)));
                constexpr bool no_support = false;
                constexpr bool no_prime_tower = false;
                floor_layer.add(support_layer.intersection(storage.getLayerOutlines(sample_layer, no_support, no_prime_tower)));
            }
            { //One additional sample at the complete bottom height.
                const size_t sample_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(support_bottom_height_layers) - static_cast<int>(z_distance_bottom_layers)));
                constexpr bool no_support = false;
                constexpr bool no_prime_tower = false;
                floor_layer.add(support_layer.intersection(storage.getLayerOutlines(sample_layer, no_support, no_prime_tower)));
            }
            floor_layer = floor_layer.unionPolygons();
            support_layer = support_layer.difference(floor_layer.offset(10)); //Subtract the support floor from the normal support.
        }

        std::vector<PolygonsPart> support_layer_parts = support_layer.splitIntoParts();
        for (PolygonsPart& part : support_layer_parts) //Convert every part into a PolygonsPart for the support.
        {
            storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(part, line_width, wall_count);
        }

#pragma omp critical (support_max_layer_nr)
        {
            if (!storage.support.supportLayers[layer_nr].support_infill_parts.empty() || !storage.support.supportLayers[layer_nr].support_roof.empty())
            {
                storage.support.layer_nr_max_filled_layer = std::max(storage.support.layer_nr_max_filled_layer, static_cast<int>(layer_nr));
            }
        }
#pragma omp atomic
        completed++;
#pragma omp critical (progress)
        {
            const double progress_contact_nodes = contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN;
            const double progress_current = completed * PROGRESS_WEIGHT_AREAS;
            const double progress_total = completed * PROGRESS_WEIGHT_AREAS;
            Progress::messageProgress(Progress::Stage::SUPPORT, progress_contact_nodes + progress_current, progress_contact_nodes + progress_total);
        }
    }
}

void TreeSupport::dropNodes(std::vector<std::vector<Node*>>& contact_nodes)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    //Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const double angle = mesh_group_settings.get<AngleRadians>("support_tree_angle");
    const coord_t maximum_move_distance = angle < 90 ? static_cast<coord_t>(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max();
    const coord_t branch_radius = mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2;
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    const double diameter_angle_scale_factor = sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t radius_sample_resolution = mesh_group_settings.get<coord_t>("support_tree_collision_resolution");
    const bool support_rests_on_model = mesh_group_settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;

    std::unordered_set<Node*> to_free_node_set;

    for (size_t layer_nr = contact_nodes.size() - 1; layer_nr > 0; layer_nr--) //Skip layer 0, since we can't drop down the vertices there.
    {
        auto& layer_contact_nodes = contact_nodes[layer_nr];
        std::deque<std::pair<size_t, Node*>> unsupported_branch_leaves; // All nodes that are leaves on this layer that would result in unsupported ('mid-air') branches.

        //Group together all nodes for each part.
        std::vector<PolygonsPart> parts = volumes_.getAvoidance(0, layer_nr).splitIntoParts();
        std::vector<std::unordered_map<Point, Node*>> nodes_per_part;
        nodes_per_part.emplace_back(); //All nodes that aren't inside a part get grouped together in the 0th part.
        for (size_t part_index = 0; part_index < parts.size(); part_index++)
        {
            nodes_per_part.emplace_back();
        }
        for (Node* p_node : layer_contact_nodes)
        {
            const Node& node = *p_node;

            if (!support_rests_on_model && !node.to_buildplate) //Can't rest on model and unable to reach the build plate. Then we must drop the node and leave parts unsupported.
            {
                unsupported_branch_leaves.push_front({ layer_nr, p_node });
                continue;
            }
            if (node.to_buildplate || parts.empty()) //It's outside, so make it go towards the build plate.
            {
                nodes_per_part[0][node.position] = p_node;
                continue;
            }
            /* Find which part this node is located in and group the nodes in
             * the same part together. Since nodes have a radius and the
             * avoidance areas are offset by that radius, the set of parts may
             * be different per node. Here we consider a node to be inside the
             * part that is closest. The node may be inside a bigger part that
             * is actually two parts merged together due to an offset. In that
             * case we may incorrectly keep two nodes separate, but at least
             * every node falls into some group.
             */
            coord_t closest_part_distance2 = std::numeric_limits<coord_t>::max();
            size_t closest_part = -1;
            for (size_t part_index = 0; part_index < parts.size(); part_index++)
            {
                constexpr bool border_result = true;
                if (parts[part_index].inside(node.position, border_result)) //If it's inside, the distance is 0 and this part is considered the best.
                {
                    closest_part = part_index;
                    closest_part_distance2 = 0;
                    break;
                }
                const ClosestPolygonPoint closest_point = PolygonUtils::findClosest(node.position, parts[part_index]);
                const coord_t distance2 = vSize2(node.position - closest_point.location);
                if (distance2 < closest_part_distance2)
                {
                    closest_part_distance2 = distance2;
                    closest_part = part_index;
                }
            }
            //Put it in the best one.
            nodes_per_part[closest_part + 1][node.position] = p_node; //Index + 1 because the 0th index is the outside part.

        }
        //Create a MST for every part.
        std::vector<MinimumSpanningTree> spanning_trees;
        for (const std::unordered_map<Point, Node*>& group : nodes_per_part)
        {
            std::vector<Point> points_to_buildplate;
            for (const std::pair<const Point, Node*>& entry : group)
            {
                points_to_buildplate.emplace_back(entry.first); //Just the position of the node.
            }
            spanning_trees.emplace_back(points_to_buildplate);
        }

        for (size_t group_index = 0; group_index < nodes_per_part.size(); group_index++)
        {
            const MinimumSpanningTree& mst = spanning_trees[group_index];
            //In the first pass, merge all nodes that are close together.
            std::unordered_set<Node*> to_delete;
            for (const std::pair<const Point, Node*>& entry : nodes_per_part[group_index])
            {
                Node* p_node = entry.second;
                Node& node = *p_node;
                if (to_delete.find(p_node) != to_delete.end())
                {
                    continue; //Delete this node (don't create a new node for it on the next layer).
                }
                const std::vector<Point>& neighbours = mst.adjacentNodes(node.position);
                if (neighbours.size() == 1 && vSize2(neighbours[0] - node.position) < maximum_move_distance * maximum_move_distance && mst.adjacentNodes(neighbours[0]).size() == 1) //We have just two nodes left, and they're very close!
                {
                    //Insert a completely new node and let both original nodes fade.
                    Point next_position = (node.position + neighbours[0]) / 2; //Average position of the two nodes.

                    const coord_t branch_radius_node = [&]() -> coord_t
                    {
                        if ((node.distance_to_top + 1) > tip_layers)
                        {
                             return branch_radius + branch_radius * (node.distance_to_top + 1) * diameter_angle_scale_factor;
                        }
                        else
                        {
                             return branch_radius * (node.distance_to_top + 1) / tip_layers;
                        }
                    }();
                    if (group_index == 0)
                    {
                        //Avoid collisions.
                        const coord_t maximum_move_between_samples = maximum_move_distance + radius_sample_resolution + 100; //100 micron extra for rounding errors.
                        PolygonUtils::moveOutside(volumes_.getAvoidance(branch_radius_node, layer_nr - 1), next_position, radius_sample_resolution + 100, maximum_move_between_samples * maximum_move_between_samples); //Some extra offset to prevent rounding errors with the sample resolution.
                    }

                    Node* neighbour = nodes_per_part[group_index][neighbours[0]];
                    size_t new_distance_to_top = std::max(node.distance_to_top, neighbour->distance_to_top) + 1;
                    size_t new_support_roof_layers_below = std::max(node.support_roof_layers_below, neighbour->support_roof_layers_below) - 1;

                    const bool to_buildplate = !volumes_.getAvoidance(branch_radius_node, layer_nr - 1).inside(next_position);
                    Node* next_node = new Node(next_position, new_distance_to_top, node.skin_direction, new_support_roof_layers_below, to_buildplate, p_node);
                    insertDroppedNode(contact_nodes[layer_nr - 1], next_node); //Insert the node, resolving conflicts of the two colliding nodes.

                    // Make sure the next pass doesn't drop down either of these (since that already happened).
                    node.merged_neighbours.push_front(neighbour);
                    to_delete.insert(neighbour);
                    to_delete.insert(p_node);
                }
                else if (neighbours.size() > 1) //Don't merge leaf nodes because we would then incur movement greater than the maximum move distance.
                {
                    //Remove all neighbours that are too close and merge them into this node.
                    for (const Point& neighbour : neighbours)
                    {
                        if (vSize2(neighbour - node.position) < maximum_move_distance * maximum_move_distance)
                        {
                            Node* neighbour_node = nodes_per_part[group_index][neighbour];
                            node.distance_to_top = std::max(node.distance_to_top, neighbour_node->distance_to_top);
                            node.support_roof_layers_below = std::max(node.support_roof_layers_below, neighbour_node->support_roof_layers_below);
                            node.merged_neighbours.push_front(neighbour_node);
                            node.merged_neighbours.insert_after(node.merged_neighbours.end(), neighbour_node->merged_neighbours.begin(), neighbour_node->merged_neighbours.end());
                            to_delete.insert(neighbour_node);
                        }
                    }
                }
            }
            //In the second pass, move all middle nodes.
            for (const std::pair<const Point, Node*>& entry : nodes_per_part[group_index])
            {
                Node* p_node = entry.second;
                const Node& node = *p_node;
                if (to_delete.find(p_node) != to_delete.end())
                {
                    continue;
                }
                //If the branch falls completely inside a collision area (the entire branch would be removed by the X/Y offset), delete it.
                if (group_index > 0 && volumes_.getCollision(0, layer_nr).inside(node.position))
                {
                    const coord_t branch_radius_node = [&]() -> coord_t
                    {
                        if (node.distance_to_top > tip_layers)
                        {
                            return branch_radius + branch_radius * node.distance_to_top * diameter_angle_scale_factor;
                        }
                        else
                        {
                            return branch_radius * node.distance_to_top / tip_layers;
                        }
                    }();
                    const ClosestPolygonPoint to_outside = PolygonUtils::findClosest(node.position, volumes_.getCollision(0, layer_nr));
                    if (vSize2(node.position - to_outside.location) >= branch_radius_node * branch_radius_node) //Too far inside.
                    {
                        if (! support_rests_on_model)
                        {
                            unsupported_branch_leaves.push_front({ layer_nr, p_node });
                        }
                        continue;
                    }
                }
                Point next_layer_vertex = node.position;
                const std::vector<Point> neighbours = mst.adjacentNodes(node.position);
                if (neighbours.size() > 1 || (neighbours.size() == 1 && vSize2(neighbours[0] - node.position) >= maximum_move_distance * maximum_move_distance)) //Only nodes that aren't about to collapse.
                {
                    //Move towards the average position of all neighbours.
                    Point sum_direction(0, 0);
                    for (const Point& neighbour : neighbours)
                    {
                        sum_direction += neighbour - node.position;
                    }
                    if(vSize2(sum_direction) <= maximum_move_distance * maximum_move_distance)
                    {
                        next_layer_vertex += sum_direction;
                    }
                    else
                    {
                        next_layer_vertex += normal(sum_direction, maximum_move_distance);
                    }
                }

                const coord_t branch_radius_node = [&]() -> coord_t
                {
                    if ((node.distance_to_top + 1) > tip_layers)
                    {
                        return branch_radius + branch_radius * (node.distance_to_top + 1) * diameter_angle_scale_factor;
                    }
                    else
                    {
                        return branch_radius * (node.distance_to_top + 1) / tip_layers;
                    }
                }();
                if (group_index == 0)
                {
                    //Avoid collisions.
                    const coord_t maximum_move_between_samples = maximum_move_distance + radius_sample_resolution + 100; //100 micron extra for rounding errors.
                    PolygonUtils::moveOutside(volumes_.getAvoidance(branch_radius_node, layer_nr - 1), next_layer_vertex, radius_sample_resolution + 100, maximum_move_between_samples * maximum_move_between_samples); //Some extra offset to prevent rounding errors with the sample resolution.
                }

                const bool to_buildplate = !volumes_.getAvoidance(branch_radius_node, layer_nr - 1).inside(next_layer_vertex);
                Node* next_node = new Node(next_layer_vertex, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, to_buildplate, p_node);
                insertDroppedNode(contact_nodes[layer_nr - 1], next_node);
            }
        }

        // Prune all branches that couldn't find support on either the model or the buildplate (resulting in 'mid-air' branches).
        for (;! unsupported_branch_leaves.empty(); unsupported_branch_leaves.pop_back())
        {
            const auto& entry = unsupported_branch_leaves.back();
            Node* i_node = entry.second;
            for (size_t i_layer = entry.first; i_node != nullptr; ++i_layer, i_node = i_node->parent)
            {
                std::vector<Node*>::iterator to_erase = std::find(contact_nodes[i_layer].begin(), contact_nodes[i_layer].end(), i_node);
                if (to_erase != contact_nodes[i_layer].end())
                {
                    to_free_node_set.insert(*to_erase);
                    contact_nodes[i_layer].erase(to_erase);
                    to_free_node_set.insert(i_node);

                    for (Node* neighbour : i_node->merged_neighbours)
                    {
                        unsupported_branch_leaves.push_front({ i_layer, neighbour });
                    }
                }
            }
        }

        const double progress_current = (contact_nodes.size() - layer_nr) * PROGRESS_WEIGHT_DROPDOWN;
        const double progress_total = contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + contact_nodes.size() * PROGRESS_WEIGHT_AREAS;
        Progress::messageProgress(Progress::Stage::SUPPORT, progress_current, progress_total);
    }

    for (Node *node : to_free_node_set)
    {
        delete node;
    }
    to_free_node_set.clear();
}

void TreeSupport::generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::vector<TreeSupport::Node*>>& contact_nodes)
{
    const coord_t point_spread = mesh.settings.get<coord_t>("support_tree_branch_distance");

    //First generate grid points to cover the entire area of the print.
    AABB bounding_box = mesh.bounding_box.flatten();
    // We want to create the grid pattern at an angle, so compute the bounding
    // box required to cover that angle.
    // Rotation of 22 degrees provides better support of diagonal lines.
    constexpr double rotate_angle = 22.0 / 180.0 * M_PI;
    const Point bounding_box_size = bounding_box.max - bounding_box.min;

    // Store centre of AABB so we can relocate the generated points
    const auto centre = bounding_box.getMiddle();
    const auto sin_angle = std::sin(rotate_angle);
    const auto cos_angle = std::cos(rotate_angle);
    // Calculate the dimensions of the AABB of the mesh AABB after being rotated
    // by `rotate_angle`. Halve the dimensions since we'll be using it as a +-
    // offset from the centre of `bounding_box`.
    // This formulation will only work with rotation angles <90 degrees. If the
    // rotation angle becomes a user-configurable value then this will need to
    // be changed
    const auto rotated_dims = Point(
        bounding_box_size.X * cos_angle + bounding_box_size.Y * sin_angle,
        bounding_box_size.X * sin_angle + bounding_box_size.Y * cos_angle) / 2;

    std::vector<Point> grid_points;
    for (auto x = -rotated_dims.X; x <= rotated_dims.X; x += point_spread)
    {
        for (auto y = -rotated_dims.Y; y <= rotated_dims.Y; y += point_spread)
        {
            // Construct a point as an offset from the mesh AABB centre, rotated
            // about the mesh AABB centre
            const auto pt = rotate(Point(x, y), rotate_angle) + centre;
            // Only add to grid points if we have a chance to collide with the
            // mesh
            if (bounding_box.contains(pt))
            {
                grid_points.push_back(pt);
            }
        }
    }


    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
    const size_t z_distance_top_layers = round_up_divide(z_distance_top, layer_height) + 1; //Support must always be 1 layer below overhang.
    const size_t support_roof_layers = [&]() -> size_t
    {
        if (mesh.settings.get<bool>("support_roof_enable"))
        {
            return round_divide(mesh.settings.get<coord_t>("support_roof_height"), mesh.settings.get<coord_t>("layer_height")); //How many roof layers, if roof is enabled.
        }
        else
        {
            return 0;
        }
    }();
    const coord_t half_overhang_distance = tan(mesh.settings.get<AngleRadians>("support_angle")) * layer_height / 2;
    for (size_t layer_nr = 1; static_cast<int>(layer_nr) < static_cast<int>(mesh.overhang_areas.size()) - static_cast<int>(z_distance_top_layers); layer_nr++)
    {
        const Polygons& overhang = mesh.overhang_areas[layer_nr + z_distance_top_layers];
        if (overhang.empty())
        {
            continue;
        }

        for (const ConstPolygonRef overhang_part : overhang)
        {
            AABB overhang_bounds(overhang_part); //Pre-generate the AABB for a quick pre-filter.
            overhang_bounds.expand(half_overhang_distance); //Allow for points to be within half an overhang step of the overhang area.
            bool added = false; //Did we add a point this way?
            for (Point candidate : grid_points)
            {
                if (overhang_bounds.contains(candidate))
                {
                    constexpr coord_t distance_inside = 0; //Move point towards the border of the polygon if it is closer than half the overhang distance: Catch points that fall between overhang areas on constant surfaces.
                    PolygonUtils::moveInside(overhang_part, candidate, distance_inside, half_overhang_distance * half_overhang_distance);
                    constexpr bool border_is_inside = true;
                    if (overhang_part.inside(candidate, border_is_inside) && !volumes_.getCollision(0, layer_nr).inside(candidate, border_is_inside))
                    {
                        constexpr size_t distance_to_top = 0;
                        constexpr bool to_buildplate = true;
                        Node* contact_node = new Node(candidate, distance_to_top, (layer_nr + z_distance_top_layers) % 2, support_roof_layers, to_buildplate, Node::NO_PARENT);
                        contact_nodes[layer_nr].emplace_back(contact_node);
                        added = true;
                    }
                }
            }
            if (!added) //If we didn't add any points due to bad luck, we want to add one anyway such that loose parts are also supported.
            {
                Point candidate = bounding_box.getMiddle();
                PolygonUtils::moveInside(overhang_part, candidate);
                constexpr size_t distance_to_top = 0;
                constexpr bool to_buildplate = true;
                Node* contact_node = new Node(candidate, distance_to_top, layer_nr % 2, support_roof_layers, to_buildplate, Node::NO_PARENT);
                contact_nodes[layer_nr].emplace_back(contact_node);
            }
        }
        for (const ConstPolygonRef overhang_part : overhang)
        {
            if (overhang_part.area() < 0)
            {
                for (auto iter = contact_nodes[layer_nr].begin(); iter != contact_nodes[layer_nr].end(); )
                {
                    if (overhang_part.inside((*iter)->position))
                    {
                        iter = contact_nodes[layer_nr].erase(iter);
                    }
                    else
                    {
                        ++iter;
                    }
                }
            }
        }
    }
}

void TreeSupport::insertDroppedNode(std::vector<Node*>& nodes_layer, Node* p_node)
{
    std::vector<Node*>::iterator conflicting_node_it = std::find(nodes_layer.begin(), nodes_layer.end(), p_node);
    if (conflicting_node_it == nodes_layer.end()) //No conflict.
    {
        nodes_layer.emplace_back(p_node);
        return;
    }

    Node* conflicting_node = *conflicting_node_it;
    conflicting_node->distance_to_top = std::max(conflicting_node->distance_to_top, p_node->distance_to_top);
    conflicting_node->support_roof_layers_below = std::max(conflicting_node->support_roof_layers_below, p_node->support_roof_layers_below);
}

ModelVolumes::ModelVolumes(const SliceDataStorage& storage, coord_t xy_distance, coord_t max_move,
                           coord_t radius_sample_resolution) :
    machine_border_{calculateMachineBorderCollision(storage.getMachineBorder())},
    xy_distance_{xy_distance},
    max_move_{max_move},
    radius_sample_resolution_{radius_sample_resolution}
{
    for (std::size_t layer_idx  = 0; layer_idx < storage.support.supportLayers.size(); ++layer_idx)
    {
        constexpr bool include_support = false;
        constexpr bool include_prime_tower = true;
        layer_outlines_.push_back(storage.getLayerOutlines(layer_idx, include_support, include_prime_tower));
    }
}

const Polygons& ModelVolumes::getCollision(coord_t radius, LayerIndex layer_idx) const
{
    radius = ceilRadius(radius);
    RadiusLayerPair key{radius, layer_idx};
    const auto it = collision_cache_.find(key);
    if (it != collision_cache_.end())
    {
        return it->second;
    }
    else
    {
        return calculateCollision(key);
    }
}

const Polygons& ModelVolumes::getAvoidance(coord_t radius, LayerIndex layer_idx) const
{
    radius = ceilRadius(radius);
    RadiusLayerPair key{radius, layer_idx};
    const auto it = avoidance_cache_.find(key);
    if (it != avoidance_cache_.end())
    {
        return it->second;
    }
    else
    {
        return calculateAvoidance(key);
    }
}

const Polygons& ModelVolumes::getInternalModel(coord_t radius, LayerIndex layer_idx) const
{
    radius = ceilRadius(radius);
    RadiusLayerPair key{radius, layer_idx};
    const auto it = internal_model_cache_.find(key);
    if (it != internal_model_cache_.end())
    {
        return it->second;
    }
    else
    {
        return calculateInternalModel(key);
    }
}

coord_t ModelVolumes::ceilRadius(coord_t radius) const
{
    const auto remainder = radius % radius_sample_resolution_;
    const auto delta = remainder != 0 ? radius_sample_resolution_- remainder : 0;
    return radius + delta;
}

const Polygons& ModelVolumes::calculateCollision(const RadiusLayerPair& key) const
{
    const auto& radius = key.first;
    const auto& layer_idx = key.second;

    auto collision_areas = machine_border_;
    if (layer_idx < static_cast<int>(layer_outlines_.size()))
    {
        collision_areas = collision_areas.unionPolygons(layer_outlines_[layer_idx]);
    }
    collision_areas = collision_areas.offset(xy_distance_ + radius, ClipperLib::JoinType::jtRound);
    const auto ret = collision_cache_.insert({key, std::move(collision_areas)});
    assert(ret.second);
    return ret.first->second;
}

const Polygons& ModelVolumes::calculateAvoidance(const RadiusLayerPair& key) const
{
    const auto& radius = key.first;
    const auto& layer_idx = key.second;

    if (layer_idx == 0)
    {
        avoidance_cache_[key] = getCollision(radius, 0);
        return avoidance_cache_[key];
    }

    // Avoidance for a given layer depends on all layers beneath it so could have very deep recursion depths if
    // called at high layer heights. We can limit the reqursion depth to N by checking if the if the layer N
    // below the current one exists and if not, forcing the calculation of that layer. This may cause another recursion
    // if the layer at 2N below the current one but we won't exceed our limit unless there are N*N uncalculated layers
    // below our current one.
    constexpr auto max_recursion_depth = 100;
    // Check if we would exceed the recursion limit by trying to process this layer
    if (layer_idx >= max_recursion_depth
        && avoidance_cache_.find({radius, layer_idx - max_recursion_depth}) == avoidance_cache_.end())
    {
        // Force the calculation of the layer `max_recursion_depth` below our current one, ignoring the result.
        getAvoidance(radius, layer_idx - max_recursion_depth);
    }
    auto avoidance_areas = getAvoidance(radius, layer_idx - 1).offset(-max_move_).smooth(5);
    avoidance_areas = avoidance_areas.unionPolygons(getCollision(radius, layer_idx));
    const auto ret = avoidance_cache_.insert({key, std::move(avoidance_areas)});
    assert(ret.second);
    return ret.first->second;
}

const Polygons& ModelVolumes::calculateInternalModel(const RadiusLayerPair& key) const
{
    const auto& radius = key.first;
    const auto& layer_idx = key.second;

    const auto& internal_areas = getAvoidance(radius, layer_idx).difference(getCollision(radius, layer_idx));
    const auto ret = internal_model_cache_.insert({key, internal_areas});
    assert(ret.second);
    return ret.first->second;
}

Polygons ModelVolumes::calculateMachineBorderCollision(Polygon machine_border)
{
    Polygons machine_volume_border;
    machine_volume_border.add(machine_border.offset(MM2INT(1000))); //Put a border of 1m around the print volume so that we don't collide.
    machine_border.reverse(); //Makes the polygon negative so that we subtract the actual volume from the collision area.
    machine_volume_border.add(machine_border);
    return machine_volume_border;
}

} //namespace cura
