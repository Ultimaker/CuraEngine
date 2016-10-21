#include "subDivCube.h"
#include "functional"
#include "../utils/polygonUtils.h"
#include "../sliceDataStorage.h"
namespace cura {
std::vector<int64_t> SubDivCube::side_length;
std::vector<int64_t> SubDivCube::height;
std::vector<int64_t> SubDivCube::square_height;
std::vector<int64_t> SubDivCube::max_draw_z_diff;
std::vector<int64_t> SubDivCube::max_line_offset;
double SubDivCube::rad_mult = 1;
int32_t SubDivCube::rad_add = 0;
double SubDivCube::rot_coef_x;
double SubDivCube::rot_coef_y;

void SubDivCube::precomputeOctree(SliceMeshStorage& mesh)
{
    rad_mult = 1;//mesh.getSettingInMillimeters("sub_div_rad_mult"); //NOTE: these are the new settings for this infill. They don't both have to be used, but one would be nice... Your choice!
    rad_add = 0;//mesh.getSettingInMicrons("sub_div_rad_add");
    double infill_angle = M_PI / 4.0;
    rot_coef_x = cos(infill_angle);
    rot_coef_y = sin(infill_angle);
    int curr_recursion_depth = 0;
    for(int64_t curr_side_length = mesh.getSettingInMicrons("infill_line_distance") * 2; curr_side_length < 25600000; curr_side_length *= 2) //!< 25600000 is an arbitrarily large number. It is imperative that any infill areas are inside of the cube defined by this number.
    {
        side_length.push_back(curr_side_length);
        height.push_back(sqrt(curr_side_length * curr_side_length * 3));
        square_height.push_back(sqrt(curr_side_length * curr_side_length * 2));
        max_draw_z_diff.push_back((1.0 / sqrt(3.0)) * curr_side_length);
        max_line_offset.push_back((sqrt(2.0 / 3.0) * curr_side_length) / 2);
        curr_recursion_depth++;
    }
    Point3 center(0, 0, 0);
    mesh.base_subdiv_cube = new SubDivCube(mesh, center, curr_recursion_depth - 1);
}

void SubDivCube::generateSubdivisionLines(int64_t z, Polygons& result, Polygons** directional_line_groups)
{
    auto addLine = [&](Point from, Point to)
    {
        PolygonRef p = result.newPoly();
        p.add(from);
        p.add(to);
    };
    /*!
     * Adds the defined line to the specified polygons. It assumes that the specified polygons are all parallel lines. Combines line segments with touching ends closer than epsilon.
     * \param group the polygons to add the line to
     * \param from the first endpoint of the line
     * \param to the second endpoint of the line
     */
    auto addLineAndCombine = [&](Polygons& group, Point from, Point to)
    {
        int epsilon = 10;
        for(unsigned int idx = 0; idx < group.size(); idx++)
        {
            if(abs(from.X - group[idx][1].X) < epsilon && abs(from.Y - group[idx][1].Y) < epsilon)
            {
                from = group[idx][0];
                group.remove(idx);
                idx--;
                continue;
            }
            if(abs(to.X - group[idx][0].X) < epsilon && abs(to.Y - group[idx][0].Y) < epsilon)
            {
                to = group[idx][1];
                group.remove(idx);
                idx--;
                continue;
            }
        }
        PolygonRef p = group.newPoly();
        p.add(from);
        p.add(to);
    };
    bool top_level = false; //!< if this cube is the top level of the recursive call
    if(directional_line_groups == nullptr) //!< if directional_line_groups is null then set the top level flag and create directional line groups
    {
        top_level = true;
        directional_line_groups = (Polygons**)calloc(3, sizeof(Polygons*));
        for(int idx = 0; idx < 3; idx++)
        {
            directional_line_groups[idx] = new Polygons();
        }
    }
    int32_t z_diff = abs(z - center.z); //!< the difference between the cube center and the target layer.
    if(z_diff > height[depth] / 2) //!< this cube does not touch the target layer. Early exit.
    {
        return;
    }
    if(z_diff < max_draw_z_diff[depth]) //!< this cube has lines that need to be drawn.
    {
        Point relative_a, relative_b; //!< relative coordinates of line endpoints around cube center
        Point a, b; //!< absolute coordinates of line endpoints
        relative_a.X = (square_height[depth] / 2) * ((double)(max_draw_z_diff[depth] - z_diff) / (double)max_draw_z_diff[depth]);
        relative_b.X = -relative_a.X;
        relative_a.Y = max_line_offset[depth] - ((z - (center.z - max_draw_z_diff[depth])) * one_over_sqrt_2);
        relative_b.Y = relative_a.Y;
        rotatePointInitial(relative_a);
        rotatePointInitial(relative_b);
        for(int idx = 0; idx < 3; idx++)//!< draw the line, then rotate 120 degrees.
        {
            a.X = center.x + relative_a.X;
            a.Y = center.y + relative_a.Y;
            b.X = center.x + relative_b.X;
            b.Y = center.y + relative_b.Y;
            addLineAndCombine(*(directional_line_groups[idx]), a, b);
            if(idx < 2)
            {
                rotatePoint120(relative_a);
                rotatePoint120(relative_b);
            }
        }
    }
    for(int idx = 0; idx < 8; idx++) //!< draws the eight children
    {
        if(children[idx] != nullptr)
        {
            children[idx]->generateSubdivisionLines(z, result, directional_line_groups);
        }
    }
    if(top_level) //!< copy directional groups into result, then free the directional groups
    {
        for(int temp = 0; temp < 3; temp++)
        {
            for(unsigned int idx = 0; idx < directional_line_groups[temp]->size(); idx++)
            {
                addLine((*directional_line_groups[temp])[idx][0], (*directional_line_groups[temp])[idx][1]);
            }
            delete directional_line_groups[temp];
        }
        free(directional_line_groups);
    }
}

SubDivCube::SubDivCube(SliceMeshStorage& mesh, Point3& center, int depth)
{
    this->depth = depth;
    this->center = center;

    if(depth == 0) // lowest layer, no need for subdivision, exit.
    {
        return;
    }
    Point3 child_center;
    long int radius = rad_mult * height[depth] / 4 + rad_add;
    // top child cube
    child_center.x = center.x;
    child_center.y = center.y;
    child_center.z = center.z + (height[depth] / 4);
    if(isValidSubdivision(mesh, child_center, radius))
    {
        children[0] = new SubDivCube(mesh, child_center, depth - 1);
    }
    // top three children
    Point relative_center; //!< center of the child cube relative to the center of the parent.
    child_center.z = center.z + height[depth] / 12;
    relative_center.X = 0;
    relative_center.Y = -max_line_offset[depth];
    rotatePointInitial(relative_center);
    for(int temp = 0; temp < 3; temp++)
    {
        child_center.x = relative_center.X + center.x;
        child_center.y = relative_center.Y + center.y;
        if(isValidSubdivision(mesh, child_center, radius)){
            children[temp + 1] = new SubDivCube(mesh, child_center, depth - 1);
        }
        if(temp < 2){
            rotatePoint120(relative_center);
        }
    }
    // bottom child
    child_center.x = center.x;
    child_center.y = center.y;
    child_center.z = center.z - (height[depth] / 4);
    if(isValidSubdivision(mesh, child_center, radius)){
        children[4] = new SubDivCube(mesh, child_center, depth - 1);
    }
    // bottom three children
    child_center.z = center.z - height[depth] / 12;
    relative_center.X = 0;
    relative_center.Y = max_line_offset[depth];
    rotatePointInitial(relative_center);
    for(int temp = 0; temp < 3; temp++)
    {
        child_center.x = relative_center.X + center.x;
        child_center.y = relative_center.Y + center.y;
        if(isValidSubdivision(mesh, child_center, radius))
        {
            children[temp + 5] = new SubDivCube(mesh, child_center, depth - 1);
        }
        if(temp < 2){
            rotatePoint120(relative_center);
        }
    }
}

bool SubDivCube::isValidSubdivision(SliceMeshStorage& mesh, Point3& center, int64_t radius)
{
    int64_t distance;
    long int sphere_slice_radius;//!< radius of bounding sphere slice on target layer
    bool inside_somewhere = false;
    bool outside_somewhere = false;
    int inside;
    double part_dist;//what percentage of the radius the target layer is away from the center along the z axis. 0 - 1
    const long int layer_height = mesh.getSettingInMicrons("layer_height");
    long int bottom_layer = (center.z - radius) / layer_height;
    long int top_layer = (center.z + radius) / layer_height;
    for(long int test_layer = bottom_layer; test_layer <= top_layer; test_layer += 3) // steps of three. Low-hanging speed gain.
    {
        part_dist = (double)(test_layer * layer_height - center.z) / radius;
        sphere_slice_radius = radius * (sqrt(1 - (part_dist * part_dist)));
        Point loc(center.x, center.y);

        inside = distanceFromPointToMesh(mesh, test_layer, loc, &distance);
        if(inside == 1)
        {
            inside_somewhere = true;
        }
        else
        {
            outside_somewhere = true;
        }
        if(outside_somewhere && inside_somewhere)
        {
            return true;
        }
        if((inside != 2) && abs(distance) < sphere_slice_radius)
        {
            return true;
        }
    }
    return false;
}

int SubDivCube::distanceFromPointToMesh(SliceMeshStorage& mesh, long int layer_nr, Point& location, int64_t* distance)
    {
    if(layer_nr < 0 || (unsigned long int)layer_nr >= mesh.layers.size()) //!< this layer is outside of valid range
    {
        return 2;
    }
    Polygons collide;
    mesh.layers[layer_nr].getSecondOrInnermostWalls(collide);
    Point centerpoint = location;
    bool inside = collide.inside(centerpoint);
    ClosestPolygonPoint border_point = PolygonUtils::moveInside2(collide, centerpoint);
    *distance = sqrt((border_point.location.X - location.X) * (border_point.location.X - location.X) + (border_point.location.Y - location.Y) * (border_point.location.Y - location.Y));
    if(inside)
    {
        return 1;
    }
    return 0;
}


void SubDivCube::rotatePointInitial(Point& target)
{
    int64_t x;
    x = rot_coef_x * target.X - rot_coef_y * target.Y;
    target.Y = rot_coef_x * target.Y + rot_coef_y * target.X;
    target.X = x;
}

void SubDivCube::rotatePoint120(Point& target)
{
    int64_t x;
    x = (-0.5) * target.X - sqrt_three_fourths * target.Y;
    target.Y = (-0.5)*target.Y + sqrt_three_fourths * target.X;
    target.X = x;
}

}//namespace cura
