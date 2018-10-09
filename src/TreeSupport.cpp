//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Application.h" //To get settings.
#include "TreeSupport.h"
#include "progress/Progress.h"
#include "settings/types/AngleRadians.h" //Creating the correct branch angles.
#include "utils/IntPoint.h" //To normalize vectors.
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/MinimumSpanningTree.h" //For connecting the correct nodes together to form an efficient tree.
#include "utils/polygon.h" //For splitting polygons into parts.
#include "utils/polygonUtils.h" //For moveInside.

#define SQRT_2 1.4142135623730950488 //Square root of 2.
#define CIRCLE_RESOLUTION 10 //The number of vertices in each circle.

//The various stages of the process can be weighted differently in the progress bar.
//These weights are obtained experimentally.
#define PROGRESS_WEIGHT_COLLISION 50 //Generating collision areas.
#define PROGRESS_WEIGHT_DROPDOWN 1 //Dropping down support.
#define PROGRESS_WEIGHT_AREAS 1 //Creating support areas.

namespace cura
{

TreeSupport::TreeSupport(const SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    //Compute the border of the build volume.
    Polygons actual_border;
    switch(mesh_group_settings.get<BuildPlateShape>("machine_shape"))
    {
        case BuildPlateShape::ELLIPTIC:
        {
            actual_border.emplace_back();
            //Construct an ellipse to approximate the build volume.
            const coord_t width = storage.machine_size.max.x - storage.machine_size.min.x;
            const coord_t depth = storage.machine_size.max.y - storage.machine_size.min.y;
            constexpr unsigned int circle_resolution = 50;
            for (unsigned int i = 0; i < circle_resolution; i++)
            {
                actual_border[0].emplace_back(storage.machine_size.getMiddle().x + cos(M_PI * 2 * i / circle_resolution) * width / 2, storage.machine_size.getMiddle().y + sin(M_PI * 2 * i / circle_resolution) * depth / 2);
            }
            break;
        }
        case BuildPlateShape::RECTANGULAR:
        default:
            actual_border.add(storage.machine_size.flatten().toPolygon());
            break;
    }

    coord_t adhesion_size = 0; //Make sure there is enough room for the platform adhesion around support.
    const ExtruderTrain& adhesion_extruder = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr");
    coord_t extra_skirt_line_width = 0;
    const std::vector<bool> is_extruder_used = storage.getExtrudersUsed();
    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
    {
        if (extruder_nr == adhesion_extruder.extruder_nr || !is_extruder_used[extruder_nr]) //Unused extruders and the primary adhesion extruder don't generate an extra skirt line.
        {
            continue;
        }
        const ExtruderTrain& other_extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];
        extra_skirt_line_width += other_extruder.settings.get<coord_t>("skirt_brim_line_width") * other_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    }
    switch (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type"))
    {
        case EPlatformAdhesion::BRIM:
            adhesion_size = adhesion_extruder.settings.get<coord_t>("skirt_brim_line_width") * adhesion_extruder.settings.get<Ratio>("initial_layer_line_width_factor") * adhesion_extruder.settings.get<size_t>("brim_line_count") + extra_skirt_line_width;
            break;
        case EPlatformAdhesion::RAFT:
            adhesion_size = adhesion_extruder.settings.get<coord_t>("raft_margin");
            break;
        case EPlatformAdhesion::SKIRT:
            adhesion_size = adhesion_extruder.settings.get<coord_t>("skirt_gap") + adhesion_extruder.settings.get<coord_t>("skirt_brim_line_width") * adhesion_extruder.settings.get<Ratio>("initial_layer_line_width_factor") * adhesion_extruder.settings.get<size_t>("skirt_line_count") + extra_skirt_line_width;
            break;
        case EPlatformAdhesion::NONE:
            adhesion_size = 0;
            break;
        default: //Also use 0.
            log("Unknown platform adhesion type! Please implement the width of the platform adhesion here.");
            break;
    }
    actual_border = actual_border.offset(-adhesion_size);

    machine_volume_border.add(actual_border.offset(1000000)); //Put a border of 1m around the print volume so that we don't collide.
    actual_border[0].reverse(); //Makes the polygon negative so that we subtract the actual volume from the collision area.
    machine_volume_border.add(actual_border);
}

void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    bool use_tree_support = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("support_tree_enable");

    if (!use_tree_support)
    {
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.settings.get<bool>("support_tree_enable"))
            {
                use_tree_support = true;
                break;
            }
        }
    }
    if (!use_tree_support)
    {
        return;
    }

    //Generate areas that have to be avoided.
    std::vector<std::vector<Polygons>> model_collision; //For every sample of branch radius, the areas that have to be avoided by branches of that radius.
    collisionAreas(storage, model_collision);
    std::vector<std::vector<Polygons>> model_avoidance; //For every sample of branch radius, the areas that have to be avoided in order to be able to go towards the build plate.
    propagateCollisionAreas(storage, model_collision, model_avoidance);
    std::vector<std::vector<Polygons>> model_internal_guide; //A model to guide branches that are stuck inside towards the centre of the model while avoiding the model itself.
    for (size_t radius_sample = 0; radius_sample < model_avoidance.size(); radius_sample++)
    {
        model_internal_guide.emplace_back();
        for (size_t layer_nr = 0; layer_nr < model_avoidance[radius_sample].size(); layer_nr++)
        {
            Polygons layer_internal_guide = model_avoidance[radius_sample][layer_nr].difference(model_collision[radius_sample][layer_nr]);
            model_internal_guide[radius_sample].push_back(layer_internal_guide);
        }
    }

    std::vector<std::unordered_set<Node*>> contact_nodes;
    contact_nodes.reserve(storage.support.supportLayers.size());
    for (size_t layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++) //Generate empty layers to store the points in.
    {
        contact_nodes.emplace_back();
    }
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (!mesh.settings.get<bool>("support_tree_enable"))
        {
            continue;
        }
        generateContactPoints(mesh, contact_nodes, model_collision[0]);
    }

    //Drop nodes to lower layers.
    dropNodes(contact_nodes, model_collision, model_avoidance, model_internal_guide);

    //Generate support areas.
    drawCircles(storage, contact_nodes, model_collision);

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

void TreeSupport::collisionAreas(const SliceDataStorage& storage, std::vector<std::vector<Polygons>>& model_collision)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t branch_radius = mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const double diameter_angle_scale_factor = sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t maximum_radius = branch_radius + storage.support.supportLayers.size() * branch_radius * diameter_angle_scale_factor;
    const coord_t radius_sample_resolution = mesh_group_settings.get<coord_t>("support_tree_collision_resolution");
    model_collision.resize((size_t)std::round((float)maximum_radius / radius_sample_resolution) + 1);

    const coord_t xy_distance = mesh_group_settings.get<coord_t>("support_xy_distance");
    constexpr bool include_helper_parts = false;
    size_t completed = 0; //To track progress in a multi-threaded environment.
#pragma omp parallel for shared(model_collision, storage) schedule(dynamic)
    for (size_t radius_sample = 0; radius_sample < model_collision.size(); radius_sample++)
    {
        const coord_t radius = radius_sample * radius_sample_resolution;
        for (size_t layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++)
        {
            Polygons collision = storage.getLayerOutlines(layer_nr, include_helper_parts);
            collision = collision.unionPolygons(machine_volume_border);
            collision = collision.offset(xy_distance + radius, ClipperLib::JoinType::jtRound); //Enough space to avoid the (sampled) width of the branch.
            model_collision[radius_sample].push_back(collision);
        }
#pragma omp atomic
        completed++;
#pragma omp critical (progress)
        {
            Progress::messageProgress(Progress::Stage::SUPPORT, (completed / 2) * PROGRESS_WEIGHT_COLLISION, model_collision.size() * PROGRESS_WEIGHT_COLLISION + storage.support.supportLayers.size() * PROGRESS_WEIGHT_DROPDOWN + storage.support.supportLayers.size() * PROGRESS_WEIGHT_AREAS);
        }
    }
}

void TreeSupport::drawCircles(SliceDataStorage& storage, const std::vector<std::unordered_set<Node*>>& contact_nodes, const std::vector<std::vector<Polygons>>& model_collision)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t branch_radius = mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2;
    const size_t wall_count = mesh_group_settings.get<size_t>("support_tree_wall_count");
    Polygon branch_circle; //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
    for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
    {
        const double angle = (double)i / CIRCLE_RESOLUTION * 2 * M_PI; //In radians.
        branch_circle.emplace_back(cos(angle) * branch_radius, sin(angle) * branch_radius);
    }
    const coord_t circle_side_length = 2 * branch_radius * sin(M_PI / CIRCLE_RESOLUTION); //Side length of a regular polygon.
    const coord_t z_distance_bottom = mesh_group_settings.get<coord_t>("support_bottom_distance");
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const size_t z_distance_bottom_layers = std::max(0U, round_up_divide(z_distance_bottom, layer_height));
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    const double diameter_angle_scale_factor = sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t line_width = mesh_group_settings.get<coord_t>("support_line_width");
    size_t completed = 0; //To track progress in a multi-threaded environment.
#pragma omp parallel for shared(storage, contact_nodes)
    for (size_t layer_nr = 0; layer_nr < contact_nodes.size(); layer_nr++)
    {
        Polygons support_layer;
        Polygons& roof_layer = storage.support.supportLayers[layer_nr].support_roof;

        //Draw the support areas and add the roofs appropriately to the support roof instead of normal areas.
        for (const Node* p_node : contact_nodes[layer_nr])
        {
            const Node& node = *p_node;

            Polygon circle;
            const double scale = (double)(node.distance_to_top + 1) / tip_layers;
            for (Point corner : branch_circle)
            {
                if (node.distance_to_top < tip_layers) //We're in the tip.
                {
                    if (node.skin_direction)
                    {
                        corner = Point(corner.X * (0.5 + scale / 2) + corner.Y * (0.5 - scale / 2), corner.X * (0.5 - scale / 2) + corner.Y * (0.5 + scale / 2));
                    }
                    else
                    {
                        corner = Point(corner.X * (0.5 + scale / 2) - corner.Y * (0.5 - scale / 2), corner.X * (-0.5 + scale / 2) + corner.Y * (0.5 + scale / 2));
                    }
                }
                else
                {
                    corner = corner * (1 + (double)(node.distance_to_top - tip_layers) * diameter_angle_scale_factor);
                }
                circle.add(node.position + corner);
            }
            if (node.support_roof_layers_below >= 0)
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
        support_layer = support_layer.difference(roof_layer);
        const size_t z_collision_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(z_distance_bottom_layers) + 1)); //Layer to test against to create a Z-distance.
        if (model_collision[0].size() > z_collision_layer)
        {
            support_layer = support_layer.difference(model_collision[0][z_collision_layer]); //Subtract the model itself (sample 0 is with 0 diameter but proper X/Y offset).
            roof_layer = roof_layer.difference(model_collision[0][z_collision_layer]);
        }
        //We smooth this support as much as possible without altering single circles. So we remove any line less than the side length of those circles.
        const double diameter_angle_scale_factor_this_layer = (double)(storage.support.supportLayers.size() - layer_nr - tip_layers) * diameter_angle_scale_factor; //Maximum scale factor.
        support_layer.simplify(circle_side_length * (1 + diameter_angle_scale_factor_this_layer), line_width >> 2); //Deviate at most a quarter of a line so that the lines still stack properly.

        //Subtract support floors.
        if (mesh_group_settings.get<bool>("support_bottom_enable"))
        {
            Polygons& floor_layer = storage.support.supportLayers[layer_nr].support_bottom;
            const coord_t support_interface_resolution = mesh_group_settings.get<coord_t>("support_interface_skip_height");
            const size_t support_interface_skip_layers = std::max(0U, round_up_divide(support_interface_resolution, layer_height));
            const coord_t support_bottom_height = mesh_group_settings.get<coord_t>("support_bottom_height");
            const size_t support_bottom_height_layers = std::max(0U, round_up_divide(support_bottom_height, layer_height));
            for(size_t layers_below = 0; layers_below < support_bottom_height_layers; layers_below += support_interface_skip_layers)
            {
                const size_t sample_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(layers_below) - static_cast<int>(z_distance_bottom_layers)));
                constexpr bool include_helper_parts = false;
                floor_layer.add(support_layer.intersection(storage.getLayerOutlines(sample_layer, include_helper_parts)));
            }
            { //One additional sample at the complete bottom height.
                const size_t sample_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(support_bottom_height_layers) - static_cast<int>(z_distance_bottom_layers)));
                constexpr bool include_helper_parts = false;
                floor_layer.add(support_layer.intersection(storage.getLayerOutlines(sample_layer, include_helper_parts)));
            }
            floor_layer.unionPolygons();
            support_layer = support_layer.difference(floor_layer.offset(10)); //Subtract the support floor from the normal support.
        }

        for (PolygonRef part : support_layer) //Convert every part into a PolygonsPart for the support.
        {
            PolygonsPart outline;
            outline.add(part);
            storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(outline, line_width, wall_count);
        }
#pragma omp critical (support_max_layer_nr)
        {
            if (!storage.support.supportLayers[layer_nr].support_infill_parts.empty() || !storage.support.supportLayers[layer_nr].support_roof.empty())
            {
                storage.support.layer_nr_max_filled_layer = std::max(storage.support.layer_nr_max_filled_layer, (int)layer_nr);
            }
        }
#pragma omp atomic
        completed++;
#pragma omp critical (progress)
        {
            Progress::messageProgress(Progress::Stage::SUPPORT, model_collision.size() * PROGRESS_WEIGHT_COLLISION + contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + completed * PROGRESS_WEIGHT_AREAS, model_collision.size() * PROGRESS_WEIGHT_COLLISION + contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + contact_nodes.size() * PROGRESS_WEIGHT_AREAS);
        }
    }
}

void TreeSupport::dropNodes(std::vector<std::unordered_set<Node*>>& contact_nodes, const std::vector<std::vector<Polygons>>& model_collision, const std::vector<std::vector<Polygons>>& model_avoidance, const std::vector<std::vector<Polygons>>& model_internal_guide)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    //Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const double angle = mesh_group_settings.get<AngleRadians>("support_tree_angle");
    const coord_t maximum_move_distance = angle < 90 ? (coord_t)(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max();
    const coord_t branch_radius = mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2;
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    const double diameter_angle_scale_factor = sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t radius_sample_resolution = mesh_group_settings.get<coord_t>("support_tree_collision_resolution");
    const bool support_rests_on_model = mesh_group_settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
    for (size_t layer_nr = contact_nodes.size() - 1; layer_nr > 0; layer_nr--) //Skip layer 0, since we can't drop down the vertices there.
    {
        auto& layer_contact_nodes = contact_nodes[layer_nr];
        std::deque<std::pair<size_t, Node*>> unsupported_branch_leaves; // All nodes that are leaves on this layer that would result in unsupported ('mid-air') branches.

        //Group together all nodes for each part.
        std::vector<PolygonsPart> parts = model_avoidance[0][layer_nr].splitIntoParts();
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
        for (std::unordered_map<Point, Node*>& group : nodes_per_part)
        {
            std::unordered_set<Point> points_to_buildplate;
            for (const std::pair<Point, Node*>& entry : group)
            {
                points_to_buildplate.insert(entry.first); //Just the position of the node.
            }
            spanning_trees.emplace_back(points_to_buildplate);
        }

        for (size_t group_index = 0; group_index < nodes_per_part.size(); group_index++)
        {
            const MinimumSpanningTree& mst = spanning_trees[group_index];
            //In the first pass, merge all nodes that are close together.
            std::unordered_set<Node*> to_delete;
            for (const std::pair<Point, Node*>& entry : nodes_per_part[group_index])
            {
                Node* p_node = entry.second;
                const Node& node = *p_node;
                if (to_delete.find(p_node) != to_delete.end())
                {
                    continue; //Delete this node (don't create a new node for it on the next layer).
                }
                const std::vector<Point>& neighbours = mst.adjacentNodes(node.position);
                if (neighbours.size() == 1 && vSize2(neighbours[0] - node.position) < maximum_move_distance * maximum_move_distance && mst.adjacentNodes(neighbours[0]).size() == 1) //We have just two nodes left, and they're very close!
                {
                    //Insert a completely new node and let both original nodes fade.
                    Point next_position = (node.position + neighbours[0]) / 2; //Average position of the two nodes.

                    const coord_t branch_radius_node = ((node.distance_to_top + 1) > tip_layers) ? (branch_radius + branch_radius * (node.distance_to_top + 1) * diameter_angle_scale_factor) : (branch_radius * (node.distance_to_top + 1) / tip_layers);
                    const size_t branch_radius_sample = std::round((float)(branch_radius_node) / radius_sample_resolution);
                    if (group_index == 0)
                    {
                        //Avoid collisions.
                        const coord_t maximum_move_between_samples = maximum_move_distance + radius_sample_resolution + 100; //100 micron extra for rounding errors.
                        PolygonUtils::moveOutside(model_avoidance[branch_radius_sample][layer_nr - 1], next_position, radius_sample_resolution + 100, maximum_move_between_samples * maximum_move_between_samples); //Some extra offset to prevent rounding errors with the sample resolution.
                    }
                    else
                    {
                        //Move towards centre of polygon.
                        const ClosestPolygonPoint closest_point_on_border = PolygonUtils::findClosest(node.position, model_internal_guide[branch_radius_sample][layer_nr - 1]);
                        const coord_t distance = vSize(node.position - closest_point_on_border.location);
                        //Try moving a bit further inside: Current distance + 1 step.
                        Point moved_inside = next_position;
                        PolygonUtils::ensureInsideOrOutside(model_internal_guide[branch_radius_sample][layer_nr - 1], moved_inside, closest_point_on_border, distance + maximum_move_distance);
                        Point difference = moved_inside - node.position;
                        if(vSize2(difference) > maximum_move_distance * maximum_move_distance)
                        {
                            difference = normal(difference, maximum_move_distance);
                        }
                        next_position = node.position + difference;
                    }

                    const bool to_buildplate = !model_avoidance[branch_radius_sample][layer_nr - 1].inside(next_position);
                    Node* next_node = new Node(next_position, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, to_buildplate, p_node);
                    insertDroppedNode(contact_nodes[layer_nr - 1], next_node); //Insert the node, resolving conflicts of the two colliding nodes.

                    // Make sure the next pass doens't drop down either of these (since that already happened).
                    Node *const neighbour = nodes_per_part[group_index][neighbours[0]];
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
            for (std::pair<Point, Node*> entry : nodes_per_part[group_index])
            {
                Node* p_node = entry.second;
                const Node& node = *p_node;
                if (to_delete.find(p_node) != to_delete.end())
                {
                    continue;
                }
                //If the branch falls completely inside a collision area (the entire branch would be removed by the X/Y offset), delete it.
                if (group_index > 0 && model_collision[0][layer_nr].inside(node.position))
                {
                    const coord_t branch_radius_node = (node.distance_to_top > tip_layers) ? (branch_radius + branch_radius * node.distance_to_top * diameter_angle_scale_factor) : (branch_radius * node.distance_to_top / tip_layers);
                    const ClosestPolygonPoint to_outside = PolygonUtils::findClosest(node.position, model_collision[0][layer_nr]);
                    if (vSize2(node.position - to_outside.location) >= branch_radius_node * branch_radius_node) //Too far inside.
                    {
                        unsupported_branch_leaves.push_front({layer_nr, p_node});
                        continue;
                    }
                }
                Point next_layer_vertex = node.position;
                std::vector<Point> neighbours = mst.adjacentNodes(node.position);
                if (neighbours.size() > 1 || (neighbours.size() == 1 && vSize2(neighbours[0] - node.position) >= maximum_move_distance * maximum_move_distance)) //Only nodes that aren't about to collapse.
                {
                    //Move towards the average position of all neighbours.
                    Point sum_direction(0, 0);
                    for (Point neighbour : neighbours)
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

                const coord_t branch_radius_node = ((node.distance_to_top + 1) > tip_layers) ? (branch_radius + branch_radius * (node.distance_to_top + 1) * diameter_angle_scale_factor) : (branch_radius * (node.distance_to_top + 1) / tip_layers);
                const size_t branch_radius_sample = std::round((float)(branch_radius_node) / radius_sample_resolution);
                if (group_index == 0)
                {
                    //Avoid collisions.
                    const coord_t maximum_move_between_samples = maximum_move_distance + radius_sample_resolution + 100; //100 micron extra for rounding errors.
                    PolygonUtils::moveOutside(model_avoidance[branch_radius_sample][layer_nr - 1], next_layer_vertex, radius_sample_resolution + 100, maximum_move_between_samples * maximum_move_between_samples); //Some extra offset to prevent rounding errors with the sample resolution.
                }
                else
                {
                    //Move towards centre of polygon.
                    const ClosestPolygonPoint closest_point_on_border = PolygonUtils::findClosest(next_layer_vertex, model_internal_guide[branch_radius_sample][layer_nr - 1]);
                    const coord_t distance = vSize(node.position - closest_point_on_border.location);
                    //Try moving a bit further inside: Current distance + 1 step.
                    Point moved_inside = next_layer_vertex;
                    PolygonUtils::ensureInsideOrOutside(model_internal_guide[branch_radius_sample][layer_nr - 1], moved_inside, closest_point_on_border, distance + maximum_move_distance);
                    Point difference = moved_inside - node.position;
                    if(vSize2(difference) > maximum_move_distance * maximum_move_distance)
                    {
                        difference = normal(difference, maximum_move_distance);
                    }
                    next_layer_vertex = node.position + difference;
                }

                const bool to_buildplate = !model_avoidance[branch_radius_sample][layer_nr - 1].inside(next_layer_vertex);
                Node* next_node = new Node(next_layer_vertex, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, to_buildplate, p_node);
                insertDroppedNode(contact_nodes[layer_nr - 1], next_node);
            }
        }

        // Prune all branches that couldn't find support on either the model or the buildplate (resulting in 'mid-air' branches).
        std::set<Node*> to_free;
        for (;! unsupported_branch_leaves.empty(); unsupported_branch_leaves.pop_back())
        {
            const auto& entry = unsupported_branch_leaves.back();
            Node* i_node = entry.second;
            for (size_t i_layer = entry.first; i_node != nullptr; ++i_layer, i_node = i_node->parent)
            {
                contact_nodes[i_layer].erase(i_node);
                to_free.insert(i_node);
                for (Node* neighbour : i_node->merged_neighbours)
                {
                    unsupported_branch_leaves.push_front({i_layer, neighbour});
                }
            }
        }
        for (Node* old_node : to_free)
        {
            delete old_node;
        }
        to_free.clear();

        Progress::messageProgress(Progress::Stage::SUPPORT, model_avoidance.size() * PROGRESS_WEIGHT_COLLISION + (contact_nodes.size() - layer_nr) * PROGRESS_WEIGHT_DROPDOWN, model_avoidance.size() * PROGRESS_WEIGHT_COLLISION + contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + contact_nodes.size() * PROGRESS_WEIGHT_AREAS);
    }
}

void TreeSupport::generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<TreeSupport::Node*>>& contact_nodes, const std::vector<Polygons>& collision_areas)
{
    const coord_t point_spread = mesh.settings.get<coord_t>("support_tree_branch_distance");

    //First generate grid points to cover the entire area of the print.
    AABB bounding_box = mesh.bounding_box.flatten();
    //We want to create the grid pattern at an angle, so compute the bounding box required to cover that angle.
    constexpr double rotate_angle = 22.0 / 180.0 * M_PI; //Rotation of 22 degrees provides better support of diagonal lines.
    const Point bounding_box_size = bounding_box.max - bounding_box.min;
    AABB rotated_bounding_box; //Bounding box is rotated around the lower left corner of the original bounding box, so translate everything to 0,0 and rotate.
    rotated_bounding_box.include(Point(0, 0));
    rotated_bounding_box.include(rotate(bounding_box_size, -rotate_angle));
    rotated_bounding_box.include(rotate(Point(0, bounding_box_size.Y), -rotate_angle));
    rotated_bounding_box.include(rotate(Point(bounding_box_size.X, 0), -rotate_angle));
    AABB unrotated_bounding_box; //Take the AABB of that and rotate back around the lower left corner of the original bounding box (still 0,0 coordinate).
    unrotated_bounding_box.include(rotate(rotated_bounding_box.min, rotate_angle));
    unrotated_bounding_box.include(rotate(rotated_bounding_box.max, rotate_angle));
    unrotated_bounding_box.include(rotate(Point(rotated_bounding_box.min.X, rotated_bounding_box.max.Y), rotate_angle));
    unrotated_bounding_box.include(rotate(Point(rotated_bounding_box.max.X, rotated_bounding_box.min.Y), rotate_angle));

    std::vector<Point> grid_points;
    for (coord_t x = unrotated_bounding_box.min.X; x <= unrotated_bounding_box.max.X; x += point_spread)
    {
        for (coord_t y = unrotated_bounding_box.min.Y; y <= unrotated_bounding_box.max.Y; y += point_spread)
        {
            grid_points.push_back(rotate(Point(x, y), rotate_angle) + bounding_box.min); //Make the points absolute again by adding the position of the lower left corner of the original bounding box.
        }
    }

    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
    const size_t z_distance_top_layers = std::max(0U, round_up_divide(z_distance_top, layer_height)) + 1; //Support must always be 1 layer below overhang.
    const size_t support_roof_layers = mesh.settings.get<bool>("support_roof_enable") ? round_divide(mesh.settings.get<coord_t>("support_roof_height"), mesh.settings.get<coord_t>("layer_height")) : 0; //How many roof layers, if roof is enabled.
    const coord_t half_overhang_distance = tan(mesh.settings.get<AngleRadians>("support_angle")) * layer_height / 2;
    for (size_t layer_nr = 1; (int)layer_nr < (int)mesh.overhang_areas.size() - (int)z_distance_top_layers; layer_nr++)
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
                    if (overhang_part.inside(candidate, border_is_inside) && !collision_areas[layer_nr].inside(candidate, border_is_inside))
                    {
                        constexpr size_t distance_to_top = 0;
                        constexpr bool to_buildplate = true;
                        Node* contact_node = new Node(candidate, distance_to_top, (layer_nr + z_distance_top_layers) % 2, support_roof_layers, to_buildplate, Node::NO_PARENT);
                        contact_nodes[layer_nr].insert(contact_node);
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
                contact_nodes[layer_nr].insert(contact_node);
            }
        }
    }
}

void TreeSupport::insertDroppedNode(std::unordered_set<Node*>& nodes_layer, Node* p_node)
{
    std::unordered_set<Node*>::iterator conflicting_node_it = nodes_layer.find(p_node);
    if (conflicting_node_it == nodes_layer.end()) //No conflict.
    {
        nodes_layer.insert(p_node);
        return;
    }

    Node* conflicting_node = *conflicting_node_it;
    conflicting_node->distance_to_top = std::max(conflicting_node->distance_to_top, p_node->distance_to_top);
    conflicting_node->support_roof_layers_below = std::max(conflicting_node->support_roof_layers_below, p_node->support_roof_layers_below);
}

void TreeSupport::propagateCollisionAreas(const SliceDataStorage& storage, const std::vector<std::vector<Polygons>>& model_collision, std::vector<std::vector<Polygons>>& model_avoidance)
{
    model_avoidance.resize(model_collision.size());

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const AngleRadians angle = mesh_group_settings.get<AngleRadians>("support_tree_angle");
    const coord_t maximum_move_distance = (angle < TAU / 4) ? (coord_t)(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max();
    size_t completed = 0; //To track progress in a multi-threaded environment.
#pragma omp parallel for shared(model_avoidance) schedule(dynamic)
    for (size_t radius_sample = 0; radius_sample < model_avoidance.size(); radius_sample++)
    {
        model_avoidance[radius_sample].push_back(model_collision[radius_sample][0]);
        for (size_t layer_nr = 1; layer_nr < storage.support.supportLayers.size(); layer_nr++)
        {
            Polygons previous_layer = model_avoidance[radius_sample][layer_nr - 1].offset(-maximum_move_distance).smooth(5); //Inset previous layer with maximum_move_distance to allow some movement. Smooth to avoid micrometre-segments.
            previous_layer = previous_layer.unionPolygons(model_collision[radius_sample][layer_nr]);
            model_avoidance[radius_sample].push_back(previous_layer);
        }
#pragma omp atomic
        completed++;
#pragma omp critical (progress)
        {
            Progress::messageProgress(Progress::Stage::SUPPORT, ((model_collision.size() / 2) + (completed / 2)) * PROGRESS_WEIGHT_COLLISION, model_avoidance.size() * PROGRESS_WEIGHT_COLLISION + storage.support.supportLayers.size() * PROGRESS_WEIGHT_DROPDOWN + storage.support.supportLayers.size() * PROGRESS_WEIGHT_AREAS);
        }
    }
}

}
