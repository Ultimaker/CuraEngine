//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "progress/Progress.h"
#include "utils/intpoint.h" //To normalize vectors.
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/MinimumSpanningTree.h" //For connecting the correct nodes together to form an efficient tree.
#include "utils/polygonUtils.h" //For moveInside.

#include "TreeSupport.h"

#define SQRT_2 1.4142135623730950488 //Square root of 2.
#define CIRCLE_RESOLUTION 10 //The number of vertices in each circle.

//The various stages of the process can be weighted differently in the progress bar.
//These weights are obtained experimentally.
#define PROGRESS_WEIGHT_COLLISION 50 //Generating collision areas.
#define PROGRESS_WEIGHT_DROPDOWN 1 //Dropping down support.
#define PROGRESS_WEIGHT_AREAS 1 //Creating support areas.

namespace cura
{

TreeSupport::TreeSupport()
{
}

void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    std::vector<std::unordered_set<Node>> contact_nodes;
    contact_nodes.reserve(storage.support.supportLayers.size());
    for (size_t layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++) //Generate empty layers to store the points in.
    {
        contact_nodes.emplace_back();
    }
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (!mesh.getSettingBoolean("support_tree_enable"))
        {
            return;
        }
        generateContactPoints(mesh, contact_nodes);
    }

    //Generate areas that have to be avoided.
    std::vector<std::vector<Polygons>> model_collision; //For every sample of branch radius, the areas that have to be avoided by branches of that radius.
    collisionAreas(storage, model_collision);

    //Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
    const coord_t layer_height = storage.getSettingInMicrons("layer_height");
    const double angle = storage.getSettingInAngleRadians("support_tree_angle");
    const coord_t maximum_move_distance = tan(angle) * layer_height;
    const coord_t line_width = storage.getSettingInMicrons("support_line_width");
    const coord_t branch_radius = storage.getSettingInMicrons("support_tree_branch_diameter") >> 1;
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    const double diameter_angle_scale_factor = sin(storage.getSettingInAngleRadians("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t radius_sample_resolution = storage.getSettingInMicrons("support_tree_collision_resolution");
    for (size_t layer_nr = contact_nodes.size() - 1; layer_nr > 0; layer_nr--) //Skip layer 0, since we can't drop down the vertices there.
    {
        std::unordered_set<Point> points_to_buildplate;
        for (Node node : contact_nodes[layer_nr])
        {
            if (node.to_buildplate)
            {
                points_to_buildplate.insert(node.position);
            }
        }
        MinimumSpanningTree mst(points_to_buildplate);

        //In the first pass, simply copy all nodes going to the build plate, so that we have a layer of nodes that we are allowed to edit.
        //All nodes that are not going to the build plate need to be moved don't need multiple passes. They move directly towards the model.
        std::unordered_set<Node> current_nodes;
        for (const Node& node : contact_nodes[layer_nr])
        {
            if (node.to_buildplate)
            {
                current_nodes.insert(node);
            }
            else
            {
                const coord_t branch_radius_node = (node.distance_to_top > tip_layers) ? (branch_radius + branch_radius * node.distance_to_top * diameter_angle_scale_factor) : (branch_radius * node.distance_to_top / tip_layers);
                const size_t branch_radius_sample = std::round((float)(branch_radius_node) / radius_sample_resolution);

                //Move towards centre of polygon.
                Point closest_point_on_border = node.position;
                PolygonUtils::moveInside(model_collision[branch_radius_sample][layer_nr - 1], closest_point_on_border);
                const coord_t distance = vSize(node.position - closest_point_on_border);
                Point next_position = node.position;
                PolygonUtils::moveInside(model_collision[branch_radius_sample][layer_nr - 1], next_position, distance + maximum_move_distance); //Try moving a bit further inside.
                //TODO: This starts vibrating at the centre. We may want to check the distance again and if it didn't become less then don't move at all.
                
                Node next_node(next_position, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, node.to_buildplate);
                insertDroppedNode(contact_nodes[layer_nr - 1], next_node);
            }
        }
        //In the second pass, merge all leaf nodes.
        for (Node node : current_nodes)
        {
            std::vector<Point> neighbours = mst.adjacentNodes(node.position);
            if (neighbours.size() == 1)
            if (neighbours.size() == 1 && vSize2(neighbours[0] - node.position) < maximum_move_distance * maximum_move_distance) //This leaf is about to collapse. Merge it with the neighbour.
            {
                if (mst.adjacentNodes(neighbours[0]).size() == 1) //We just have two nodes left!
                {
                    //Insert a completely new node and let both original nodes fade.
                    Point next_position = (node.position + neighbours[0]) / 2; //Average position of the two nodes.

                    //Avoid collisions.
                    const coord_t branch_radius_node = (node.distance_to_top > tip_layers) ? (branch_radius + branch_radius * node.distance_to_top * diameter_angle_scale_factor) : (branch_radius * node.distance_to_top / tip_layers);
                    const size_t branch_radius_sample = std::round((float)(branch_radius_node) / radius_sample_resolution);
                    PolygonUtils::moveOutside(model_collision[branch_radius_sample][layer_nr - 1], next_position, radius_sample_resolution + 100, maximum_move_distance * maximum_move_distance); //Some extra offset to prevent rounding errors with the sample resolution.
                    const bool to_buildplate = !model_collision[branch_radius_sample][layer_nr - 1].inside(next_position);

                    Node next_node(next_position, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, to_buildplate);
                    insertDroppedNode(contact_nodes[layer_nr - 1], next_node); //Insert the node, resolving conflicts of the two colliding nodes.
                }
                else
                {
                    //We'll drop this node, but modify some of the properties of its neighbour.
                    Node neighbour_finder;
                    neighbour_finder.position = neighbours[0]; //Find the node by its position (which is the hash and equals function).
                    std::unordered_set<Node>::iterator neighbour = current_nodes.find(neighbour_finder);
                    neighbour->distance_to_top = std::max(neighbour->distance_to_top, node.distance_to_top);
                    neighbour->support_roof_layers_below = std::max(neighbour->support_roof_layers_below, node.support_roof_layers_below);
                    current_nodes.erase(node);
                }
            }
        }
        //In the third pass, move all middle nodes.
        for (Node node : current_nodes)
        {
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
                next_layer_vertex += normal(sum_direction, maximum_move_distance);
            }

            //Avoid collisions.
            const coord_t branch_radius_node = (node.distance_to_top > tip_layers) ? (branch_radius + branch_radius * node.distance_to_top * diameter_angle_scale_factor) : (branch_radius * node.distance_to_top / tip_layers);
            const size_t branch_radius_sample = std::round((float)(branch_radius_node) / radius_sample_resolution);
            PolygonUtils::moveOutside(model_collision[branch_radius_sample][layer_nr - 1], next_layer_vertex, radius_sample_resolution + 100, maximum_move_distance * maximum_move_distance); //Some extra offset to prevent rounding errors with the sample resolution.
            const bool to_buildplate = !model_collision[branch_radius_sample][layer_nr].inside(next_layer_vertex);

            Node next_node(next_layer_vertex, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, to_buildplate);
            insertDroppedNode(contact_nodes[layer_nr - 1], next_node);
        }
        Progress::messageProgress(Progress::Stage::SUPPORT, model_collision.size() * PROGRESS_WEIGHT_COLLISION + (contact_nodes.size() - layer_nr) * PROGRESS_WEIGHT_DROPDOWN, model_collision.size() * PROGRESS_WEIGHT_COLLISION + contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + contact_nodes.size() * PROGRESS_WEIGHT_AREAS);
    }

    const unsigned int wall_count = storage.getSettingAsCount("support_tree_wall_count");
    Polygon branch_circle; //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
    for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
    {
        const double angle = (double)i / CIRCLE_RESOLUTION * 2 * M_PI; //In radians.
        branch_circle.emplace_back(cos(angle) * branch_radius, sin(angle) * branch_radius);
    }
    const coord_t circle_side_length = 2 * branch_radius * sin(M_PI / CIRCLE_RESOLUTION); //Side length of a regular polygon.
#pragma omp parallel for shared(storage, contact_nodes)
    for (size_t layer_nr = 0; layer_nr < contact_nodes.size(); layer_nr++)
    {
        Polygons support_layer;
        Polygons& roof_layer = storage.support.supportLayers[layer_nr].support_roof;

        for (const Node node : contact_nodes[layer_nr])
        {
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
                    corner *= 1 + (double)(node.distance_to_top - tip_layers) * diameter_angle_scale_factor;
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
        //We smooth this support as much as possible without altering single circles. So we remove any line less than the side length of those circles.
        const double diameter_angle_scale_factor_this_layer = (double)(storage.support.supportLayers.size() - layer_nr - tip_layers) * diameter_angle_scale_factor; //Maximum scale factor.
        support_layer.simplify(circle_side_length * (1 + diameter_angle_scale_factor_this_layer), line_width >> 2); //Deviate at most a quarter of a line so that the lines still stack properly.
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
#pragma omp critical (progress)
        {
            Progress::messageProgress(Progress::Stage::SUPPORT, model_collision.size() * PROGRESS_WEIGHT_COLLISION + contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + layer_nr * PROGRESS_WEIGHT_AREAS, model_collision.size() * PROGRESS_WEIGHT_COLLISION + contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + contact_nodes.size() * PROGRESS_WEIGHT_AREAS);
        }
    }

    storage.support.generated = true;
}

void TreeSupport::collisionAreas(const SliceDataStorage& storage, std::vector<std::vector<Polygons>>& model_collision)
{
    const coord_t branch_radius = storage.getSettingInMicrons("support_tree_branch_diameter") >> 1;
    const coord_t layer_height = storage.getSettingInMicrons("layer_height");
    const double angle = storage.getSettingInAngleRadians("support_tree_angle");
    const coord_t maximum_move_distance = tan(angle) * layer_height;
    const double diameter_angle_scale_factor = sin(storage.getSettingInAngleRadians("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t maximum_radius = branch_radius + storage.support.supportLayers.size() * branch_radius * diameter_angle_scale_factor;
    const coord_t radius_sample_resolution = storage.getSettingInMicrons("support_tree_collision_resolution");
    model_collision.resize((size_t)std::round((float)maximum_radius / radius_sample_resolution) + 1);

    const coord_t xy_distance = storage.getSettingInMicrons("support_xy_distance");
    constexpr bool include_helper_parts = false;
    size_t completed = 0; //To track progress in a multi-threaded environment.
    #pragma omp parallel for shared(model_collision, storage) schedule(dynamic)
    for (size_t radius_sample = 0; radius_sample <= (size_t)std::round((float)maximum_radius / radius_sample_resolution); radius_sample++)
    {
        const coord_t diameter = radius_sample * radius_sample_resolution;
        model_collision[radius_sample].push_back(storage.getLayerOutlines(0, include_helper_parts).offset(xy_distance + diameter, ClipperLib::JoinType::jtRound));
        for (size_t layer_nr = 1; layer_nr < storage.support.supportLayers.size(); layer_nr++)
        {
            model_collision[radius_sample].push_back(model_collision[radius_sample][layer_nr - 1].offset(-maximum_move_distance)); //Inset previous layer with maximum_move_distance to allow some movement.
            model_collision[radius_sample][layer_nr] = model_collision[radius_sample][layer_nr].unionPolygons(storage.getLayerOutlines(layer_nr, include_helper_parts).offset(xy_distance + diameter, ClipperLib::JoinType::jtRound));
        }
#pragma omp atomic
        completed++;
#pragma omp critical (progress)
        {
            Progress::messageProgress(Progress::Stage::SUPPORT, completed * PROGRESS_WEIGHT_COLLISION, model_collision.size() * PROGRESS_WEIGHT_COLLISION + storage.support.supportLayers.size() * PROGRESS_WEIGHT_DROPDOWN + storage.support.supportLayers.size() * PROGRESS_WEIGHT_AREAS);
        }
    }
}

void TreeSupport::generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<TreeSupport::Node>>& contact_nodes)
{
    const coord_t layer_height = mesh.getSettingInMicrons("layer_height");
    const coord_t z_distance_top = mesh.getSettingInMicrons("support_top_distance");
    const size_t z_distance_top_layers = std::max(0U, round_up_divide(z_distance_top, layer_height)) + 1; //Support must always be 1 layer below overhang.
    const size_t support_roof_layers = mesh.getSettingBoolean("support_roof_enable") ? round_divide(mesh.getSettingInMicrons("support_roof_height"), mesh.getSettingInMicrons("layer_height")) : 0; //How many roof layers, if roof is enabled.
    for (size_t layer_nr = 0; layer_nr < mesh.overhang_areas.size() - z_distance_top_layers; layer_nr++)
    {
        const Polygons& overhang = mesh.overhang_areas[layer_nr + z_distance_top_layers];
        if (overhang.empty())
        {
            continue;
        }

        //First generate a lot of points in a grid pattern.
        const Polygons outside_polygons = overhang.getOutsidePolygons();
        const AABB bounding_box(outside_polygons); //To know how far we should generate points.
        const coord_t point_spread = mesh.getSettingInMicrons("support_tree_branch_distance");

        //We want to create the grid pattern at an angle, so compute the bounding box required to cover that angle.
        constexpr double rotate_angle = 22.0 / 180.0 * M_PI; //Rotation of 22 degrees provides better support of diagonal lines.
        const Point bounding_box_size = bounding_box.max - bounding_box.min;
        AABB rotated_bounding_box;
        rotated_bounding_box.include(Point(0, 0));
        rotated_bounding_box.include(rotate(bounding_box_size, -rotate_angle));
        rotated_bounding_box.include(rotate(Point(0, bounding_box_size.Y), -rotate_angle));
        rotated_bounding_box.include(rotate(Point(bounding_box_size.X, 0), -rotate_angle));
        AABB unrotated_bounding_box;
        unrotated_bounding_box.include(rotate(rotated_bounding_box.min, rotate_angle));
        unrotated_bounding_box.include(rotate(rotated_bounding_box.max, rotate_angle));
        unrotated_bounding_box.include(rotate(Point(rotated_bounding_box.min.X, rotated_bounding_box.max.Y), rotate_angle));
        unrotated_bounding_box.include(rotate(Point(rotated_bounding_box.max.X, rotated_bounding_box.min.Y), rotate_angle));

        for (const ConstPolygonRef overhang_part : outside_polygons)
        {
            bool added = false; //Did we add a point this way?
            for (coord_t x = unrotated_bounding_box.min.X; x <= unrotated_bounding_box.max.X; x += point_spread)
            {
                for (coord_t y = unrotated_bounding_box.min.Y; y <= unrotated_bounding_box.max.Y; y += point_spread)
                {
                    Point candidate = rotate(Point(x, y), rotate_angle) + bounding_box.min;
                    constexpr bool border_is_inside = true;
                    if (overhang_part.inside(candidate, border_is_inside))
                    {
                        constexpr size_t distance_to_top = 0;
                        constexpr bool to_buildplate = true;
                        Node contact_node(candidate, distance_to_top, (layer_nr + z_distance_top_layers) % 2, support_roof_layers, to_buildplate);
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
                Node contact_node(candidate, distance_to_top, layer_nr % 2, support_roof_layers, to_buildplate);
                contact_nodes[layer_nr].insert(contact_node);
            }
        }
    }
}

void TreeSupport::insertDroppedNode(std::unordered_set<Node>& nodes_layer, Node& node)
{
    std::unordered_set<Node>::iterator conflicting_node = nodes_layer.find(node);
    if (conflicting_node == nodes_layer.end()) //No conflict.
    {
        nodes_layer.insert(node);
        return;
    }

    conflicting_node->distance_to_top = std::max(conflicting_node->distance_to_top, node.distance_to_top);
    conflicting_node->support_roof_layers_below = std::max(conflicting_node->support_roof_layers_below, node.support_roof_layers_below);
}

}