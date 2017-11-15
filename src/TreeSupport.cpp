//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/intpoint.h" //To normalize vectors.
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/MinimumSpanningTree.h" //For connecting the correct nodes together to form an efficient tree.
#include "utils/polygonUtils.h" //For moveInside.

#include "TreeSupport.h"

#define SQRT_2 1.4142135623730950488 //Square root of 2.
#define CIRCLE_RESOLUTION 10 //The number of vertices in each circle.

namespace cura
{

TreeSupport::TreeSupport()
{
}

void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    std::vector<std::unordered_set<Point>> contact_points;
    std::vector<std::unordered_map<Point, Node>> contact_nodes;
    contact_points.reserve(storage.support.supportLayers.size());
    contact_nodes.reserve(storage.support.supportLayers.size());
    for (size_t layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++) //Generate empty layers to store the points in.
    {
        contact_points.emplace_back();
        contact_nodes.emplace_back();
    }
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (!mesh.getSettingBoolean("support_tree_enable"))
        {
            return;
        }
        generateContactPoints(mesh, contact_points, contact_nodes);
    }

    //Generate areas that have to be avoided.
    const coord_t layer_height = storage.getSettingInMicrons("layer_height");
    const double angle = storage.getSettingInAngleRadians("support_tree_angle");
    const coord_t maximum_move_distance = tan(angle) * layer_height;
    const coord_t branch_radius = storage.getSettingInMicrons("support_tree_branch_diameter") >> 1;
    std::vector<Polygons> model_collision; //Areas that have to be avoided by the tips of the branches.
    std::vector<Polygons> model_collision_branch_radius; //Areas that have to be avoided by the base of the branches (with branch diameter added for a better approximation).
    const coord_t xy_distance = storage.getSettingInMicrons("support_xy_distance");
    constexpr bool include_helper_parts = false;
    model_collision.push_back(storage.getLayerOutlines(0, include_helper_parts).offset(xy_distance, ClipperLib::JoinType::jtRound));
    model_collision_branch_radius.push_back(model_collision[0].offset(branch_radius, ClipperLib::JoinType::jtRound));
    //TODO: If allowing support to rest on model, these need to be just the model outlines.
    for (size_t layer_nr = 1; layer_nr < storage.print_layer_count; layer_nr++)
    {
        //Generate an area above the current layer where you'd still collide with the current layer if you were to move with at most maximum_move_distance.
        model_collision.push_back(model_collision[layer_nr - 1].offset(-maximum_move_distance, ClipperLib::JoinType::jtRound)); //Inset previous layer with maximum_move_distance to allow some movement.
        model_collision[layer_nr] = model_collision[layer_nr].unionPolygons(storage.getLayerOutlines(layer_nr, include_helper_parts).offset(xy_distance, ClipperLib::JoinType::jtRound)); //Add current layer's collision to that.
        model_collision_branch_radius.push_back(model_collision_branch_radius[layer_nr - 1].offset(-maximum_move_distance, ClipperLib::JoinType::jtRound));
        model_collision_branch_radius[layer_nr] = model_collision_branch_radius[layer_nr].unionPolygons(storage.getLayerOutlines(layer_nr, include_helper_parts).offset(xy_distance + branch_radius, ClipperLib::JoinType::jtRound));
    }

    //Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
    const coord_t line_width = storage.getSettingInMicrons("support_line_width");
    const double diameter_angle_scale_factor = sin(storage.getSettingInAngleRadians("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    for (size_t layer_nr = contact_points.size() - 1; layer_nr > 0; layer_nr--) //Skip layer 0, since we can't drop down the vertices there.
    {
        MinimumSpanningTree mst(contact_points[layer_nr]);
        for (Point vertex : contact_points[layer_nr])
        {
            const Node node = contact_nodes[layer_nr][vertex];
            Point next_layer_vertex = vertex;
            std::vector<Point> neighbours = mst.adjacentNodes(vertex);
            if (!neighbours.empty()) //Need to move towards the neighbours.
            {
                Point sum_direction(0, 0);
                for (Point neighbour : neighbours)
                {
                    sum_direction += neighbour - vertex;
                }
                Point motion = normal(sum_direction, maximum_move_distance);
                if (neighbours.size() == 1 && vSize2(sum_direction) < maximum_move_distance * maximum_move_distance)
                {
                    if (mst.adjacentNodes(neighbours[0]).size() == 1) //We just have two nodes left!
                    {
                        next_layer_vertex += motion / 2;
                    }
                    else //This is a leaf that's about to collapse. Leave it out on the next layer.
                    {
                        contact_nodes[layer_nr - 1][neighbours[0]].distance_to_top = std::max(contact_nodes[layer_nr - 1][neighbours[0]].distance_to_top, node.distance_to_top);
                        contact_nodes[layer_nr - 1][neighbours[0]].support_roof_layers_below = std::max(contact_nodes[layer_nr - 1][neighbours[0]].support_roof_layers_below, node.support_roof_layers_below);
                        continue;
                    }
                }
                else
                {
                    next_layer_vertex += motion;
                }
            }

            //Avoid collisions.
            if (node.distance_to_top > tip_layers) //Main branch.
            {
                const coord_t branch_radius_node = branch_radius * node.distance_to_top * diameter_angle_scale_factor;
                PolygonUtils::moveOutside(model_collision_branch_radius[layer_nr - 1], next_layer_vertex, branch_radius_node, maximum_move_distance * maximum_move_distance);
            }
            else //Tip.
            {
                const coord_t branch_radius_node = branch_radius * node.distance_to_top / tip_layers;
                PolygonUtils::moveOutside(model_collision[layer_nr - 1], next_layer_vertex, branch_radius_node, maximum_move_distance * maximum_move_distance);
            }
            if (model_collision[layer_nr].inside(next_layer_vertex)) //We're stuck inside the model down there! Sadly, this branch will have to rest on the model.
            {
                continue;
            }
            contact_points[layer_nr - 1].insert(next_layer_vertex);
            contact_nodes[layer_nr - 1][next_layer_vertex].distance_to_top = node.distance_to_top + 1;
            contact_nodes[layer_nr - 1][next_layer_vertex].skin_direction = node.skin_direction;
            contact_nodes[layer_nr - 1][next_layer_vertex].support_roof_layers_below = node.support_roof_layers_below - 1;
        }
    }

    const unsigned int wall_count = storage.getSettingAsCount("support_tree_wall_count");
    Polygon branch_circle; //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
    for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
    {
        const double angle = (double)i / CIRCLE_RESOLUTION * 2 * M_PI; //In radians.
        branch_circle.emplace_back(cos(angle) * branch_radius, sin(angle) * branch_radius);
    }
    const coord_t circle_side_length = 2 * branch_radius * sin(M_PI / CIRCLE_RESOLUTION); //Side length of a regular polygon.
    for (size_t layer_nr = 0; layer_nr < contact_points.size(); layer_nr++)
    {
        Polygons support_layer;
        Polygons& roof_layer = storage.support.supportLayers[layer_nr].support_roof;

        for (const Point point : contact_points[layer_nr])
        {
            const Node& node = contact_nodes[layer_nr][point];
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
                circle.add(point + corner);
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
        PolygonsPart debug;
        debug.add(model_collision_branch_radius[layer_nr]);
        storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(debug, line_width, wall_count);
        if (!storage.support.supportLayers[layer_nr].support_infill_parts.empty() || !storage.support.supportLayers[layer_nr].support_roof.empty())
        {
            storage.support.layer_nr_max_filled_layer = layer_nr;
        }
    }

    storage.support.generated = true;
}

void TreeSupport::generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<Point>>& contact_points, std::vector<std::unordered_map<Point, TreeSupport::Node>>& contact_nodes)
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
                        contact_points[layer_nr].insert(candidate);
                        contact_nodes[layer_nr][candidate] = Node();
                        contact_nodes[layer_nr][candidate].skin_direction = (layer_nr + z_distance_top_layers) % 2;
                        contact_nodes[layer_nr][candidate].support_roof_layers_below = support_roof_layers;
                        added = true;
                    }
                }
            }
            if (!added) //If we didn't add any points due to bad luck, we want to add one anyway such that loose parts are also supported.
            {
                Point candidate = bounding_box.getMiddle();
                PolygonUtils::moveInside(overhang_part, candidate);
                contact_points[layer_nr].insert(candidate);
                contact_nodes[layer_nr][candidate] = Node();
                contact_nodes[layer_nr][candidate].skin_direction = layer_nr % 2;
                contact_nodes[layer_nr][candidate].support_roof_layers_below = support_roof_layers;
            }
        }
    }
}

}