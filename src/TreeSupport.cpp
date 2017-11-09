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
    contact_points.reserve(storage.support.supportLayers.size());
    for (size_t layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++) //Generate empty layers to store the points in.
    {
        contact_points.emplace_back();
    }
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (!mesh.getSettingBoolean("support_tree_enable"))
        {
            return;
        }
        generateContactPoints(mesh, contact_points);
    }
    
    //Generate areas that have to be avoided.
    const coord_t layer_height = storage.getSettingInMicrons("layer_height");
    const double angle = storage.getSettingInAngleRadians("support_tree_angle");
    const coord_t maximum_move_distance = tan(angle) * layer_height;
    std::vector<Polygons> model_collision;
    const coord_t branch_radius = storage.getSettingInMicrons("support_tree_branch_diameter") >> 1;
    const coord_t xy_distance = storage.getSettingInMicrons("support_xy_distance") + branch_radius;
    constexpr bool include_helper_parts = false;
    model_collision.push_back(storage.getLayerOutlines(0, include_helper_parts).offset(xy_distance));
    //TODO: If allowing support to rest on model, these need to be just the model outlines.
    for (size_t layer_nr = 1; layer_nr < storage.print_layer_count; layer_nr ++)
    {
        //Generate an area above the current layer where you'd still collide with the current layer if you were to move with at most maximum_move_distance.
        model_collision.push_back(model_collision[layer_nr - 1].offset(-maximum_move_distance)); //Inset previous layer with maximum_move_distance to allow some movement.
        model_collision[layer_nr] = model_collision[layer_nr].unionPolygons(storage.getLayerOutlines(layer_nr, include_helper_parts).offset(xy_distance)); //Add current layer's collision to that.
    }

    //Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
    for (size_t layer_nr = contact_points.size() - 1; layer_nr > 0; layer_nr--) //Skip layer 0, since we can't drop down the vertices there.
    {
        MinimumSpanningTree mst(contact_points[layer_nr]);
        for (Point vertex : contact_points[layer_nr])
        {
            std::vector<Point> neighbours = mst.adjacentNodes(vertex);
            if (neighbours.empty()) //Just a single vertex.
            {
                PolygonUtils::moveOutside(model_collision[layer_nr], vertex, maximum_move_distance); //Avoid collision.
                contact_points[layer_nr - 1].insert(vertex);
            }
            Point sum_direction(0, 0);
            for (Point neighbour : neighbours)
            {
                sum_direction += neighbour - vertex;
            }
            if (neighbours.size() == 1 && vSize2(sum_direction) < maximum_move_distance * maximum_move_distance) //Smaller than one step. Leave this one out.
            {
                continue;
            }
            Point motion = normal(sum_direction, maximum_move_distance);
            Point next_layer_vertex = vertex + motion;
            PolygonUtils::moveOutside(model_collision[layer_nr], next_layer_vertex, maximum_move_distance); //Avoid collision.
            contact_points[layer_nr - 1].insert(next_layer_vertex);
        }
    }

    //TODO: When reaching the bottom, cut away all edges of the MST that are still not contracted.
    //TODO: Do a second pass of dropping down but with leftover edges removed.

    const unsigned int wall_count = storage.getSettingAsCount("support_tree_wall_count");
    const coord_t line_width = storage.getSettingInMicrons("support_line_width");
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

        for (const Point point : contact_points[layer_nr])
        {
            Polygon circle;
            for (Point corner : branch_circle)
            {
                circle.add(point + corner);
            }
            support_layer.add(circle);
        }
        support_layer = support_layer.unionPolygons();
        //We smooth this support as much as possible without altering single circles. So we remove any line less than the side length of those circles.
        support_layer.simplify(circle_side_length, line_width >> 2); //Deviate at most a quarter of a line so that the lines still stack properly.
        for (PolygonRef part : support_layer) //Convert every part into a PolygonsPart for the support.
        {
            PolygonsPart outline;
            outline.add(part);
            storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(outline, line_width, wall_count);
        }
        if (!storage.support.supportLayers[layer_nr].support_infill_parts.empty())
        {
            storage.support.layer_nr_max_filled_layer = layer_nr;
        }
    }
    
    storage.support.generated = true;
}

void TreeSupport::generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<Point>>& contact_points)
{
    const coord_t layer_height = mesh.getSettingInMicrons("layer_height");
    const coord_t z_distance_top = mesh.getSettingInMicrons("support_top_distance");
    const size_t z_distance_top_layers = std::max(0U, round_up_divide(z_distance_top, layer_height)) + 1; //Support must always be 1 layer below overhang.
    for (size_t layer_nr = 0; layer_nr < mesh.overhang_areas.size() - z_distance_top_layers; layer_nr++)
    {
        const Polygons& overhang = mesh.overhang_areas[layer_nr + z_distance_top_layers];
        if (overhang.empty())
        {
            continue;
        }

        //First generate a lot of points in a grid pattern.
        Polygons outside_polygons = overhang.getOutsidePolygons();
        AABB bounding_box(outside_polygons); //To know how far we should generate points.
        coord_t point_spread = mesh.getSettingInMicrons("support_tree_branch_distance");
        point_spread *= SQRT_2; //We'll rotate these points 45 degrees, so this is the point distance when axis-aligned.
        bounding_box.round(point_spread);
        for (PolygonRef overhang_part : outside_polygons)
        {
            AABB bounding_box(outside_polygons);
            bounding_box.round(point_spread);
            bool added = false; //Did we add a point this way?
            for (coord_t x = bounding_box.min.X; x <= bounding_box.max.X; x += point_spread >> 1)
            {
                for (coord_t y = bounding_box.min.Y + (point_spread << 1) * (x % 2); y <= bounding_box.max.Y; y += point_spread) //This produces points in a 45-degree rotated grid.
                {
                    Point candidate(x, y);
                    constexpr bool border_is_inside = true;
                    if (overhang_part.inside(candidate, border_is_inside))
                    {
                        contact_points[layer_nr].insert(candidate);
                        added = true;
                    }
                }
            }
            if (!added) //If we didn't add any points due to bad luck, we want to add one anyway such that loose parts are also supported.
            {
                Point candidate = bounding_box.getMiddle();
                PolygonUtils::moveInside(overhang_part, candidate);
                contact_points[layer_nr].insert(candidate);
            }
        }
    }
}

}