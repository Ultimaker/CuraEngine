#include "SubDivCube.h"

#include <functional>

#include "../utils/polygonUtils.h"
#include "../sliceDataStorage.h"
#include "../utils/math.h"

#define ONE_OVER_SQRT_2 0.7071067811865475244008443621048490392848359376884740 //1 / sqrt(2)
#define ONE_OVER_SQRT_3 0.577350269189625764509148780501957455647601751270126876018 //1 / sqrt(3)
#define ONE_OVER_SQRT_6 0.408248290463863016366214012450981898660991246776111688072 //1 / sqrt(6)
#define SQRT_TWO_THIRD 0.816496580927726032732428024901963797321982493552223376144 //sqrt(2 / 3)

namespace cura
{

std::vector<SubDivCube::CubeProperties> SubDivCube::cube_properties_per_recursion_step;
int32_t SubDivCube::radius_addition = 0;
Point3Matrix SubDivCube::rotation_matrix;
PointMatrix SubDivCube::infill_rotation_matrix;

SubDivCube::~SubDivCube()
{
    for (int child_idx = 0; child_idx < 8; child_idx++)
    {
        if (children[child_idx])
        {
            delete children[child_idx];
        }
    }
}

void SubDivCube::precomputeOctree(SliceMeshStorage& mesh)
{
    radius_addition = mesh.getSettingInMicrons("sub_div_rad_add");
    double infill_angle = M_PI / 4.0;

    coord_t furthest_dist_from_origin = std::sqrt(square(mesh.getSettingInMicrons("machine_height")) + square(mesh.getSettingInMicrons("machine_depth") / 2) + square(mesh.getSettingInMicrons("machine_width") / 2));
    coord_t max_side_length = furthest_dist_from_origin * 2;

    int curr_recursion_depth = 0;
    const int64_t infill_line_distance = mesh.getSettingInMicrons("infill_line_distance");
    if (infill_line_distance > 0)
    {
        for (int64_t curr_side_length = infill_line_distance * 2; curr_side_length < max_side_length * 2; curr_side_length *= 2)
        {
            cube_properties_per_recursion_step.emplace_back();
            CubeProperties& cube_properties_here = cube_properties_per_recursion_step.back();
            cube_properties_here.side_length = curr_side_length;
            cube_properties_here.height = sqrt(3) * curr_side_length;
            cube_properties_here.square_height = sqrt(2) * curr_side_length;
            cube_properties_here.max_draw_z_diff = ONE_OVER_SQRT_3 * curr_side_length;
            cube_properties_here.max_line_offset = ONE_OVER_SQRT_6 * curr_side_length;
            curr_recursion_depth++;
        }
    }
    Point3 center(0, 0, 0);

    Point3Matrix tilt; // rotation matrix to get from axis aligned cubes to cubes standing on their tip
    // The Z axis is transformed to go in positive Y direction
    //
    //  cross section in a horizontal plane      horizontal plane showing
    //  looking down at the origin O             positive X and positive Y
    //                 Z                                                        .
    //                /:\                              Y                        .
    //               / : \                             ^                        .
    //              /  :  \                            |                        .
    //             /  .O.  \                           |                        .
    //            /.~'   '~.\                          O---->X                  .
    //          X """"""""""" Y                                                 .
    tilt.matrix[0] = -ONE_OVER_SQRT_2;  tilt.matrix[1] = ONE_OVER_SQRT_2; tilt.matrix[2] = 0;
    tilt.matrix[3] = -ONE_OVER_SQRT_6; tilt.matrix[4] = -ONE_OVER_SQRT_6; tilt.matrix[5] = SQRT_TWO_THIRD ;
    tilt.matrix[6] = ONE_OVER_SQRT_3;  tilt.matrix[7] = ONE_OVER_SQRT_3;  tilt.matrix[8] = ONE_OVER_SQRT_3;

    infill_rotation_matrix = PointMatrix(infill_angle);
    Point3Matrix infill_angle_mat(infill_rotation_matrix);

    rotation_matrix = infill_angle_mat.compose(tilt);

    mesh.base_subdiv_cube = new SubDivCube(mesh, center, curr_recursion_depth - 1);
}

void SubDivCube::generateSubdivisionLines(int64_t z, Polygons& result)
{
    if (cube_properties_per_recursion_step.empty()) //Infill is set to 0%.
    {
        return;
    }
    Polygons directional_line_groups[3];

    generateSubdivisionLines(z, result, directional_line_groups);

    for (int dir_idx = 0; dir_idx < 3; dir_idx++)
    {
        Polygons& line_group = directional_line_groups[dir_idx];
        for (unsigned int line_idx = 0; line_idx < line_group.size(); line_idx++)
        {
            result.addLine(line_group[line_idx][0], line_group[line_idx][1]);
        }
    }
}

void SubDivCube::generateSubdivisionLines(int64_t z, Polygons& result, Polygons (&directional_line_groups)[3])
{
    CubeProperties cube_properties = cube_properties_per_recursion_step[depth];

    int32_t z_diff = std::abs(z - center.z); //!< the difference between the cube center and the target layer.
    if (z_diff > cube_properties.height / 2) //!< this cube does not touch the target layer. Early exit.
    {
        return;
    }
    if (z_diff < cube_properties.max_draw_z_diff) //!< this cube has lines that need to be drawn.
    {
        Point relative_a, relative_b; //!< relative coordinates of line endpoints around cube center
        Point a, b; //!< absolute coordinates of line endpoints
        relative_a.X = (cube_properties.square_height / 2) * (cube_properties.max_draw_z_diff - z_diff) / cube_properties.max_draw_z_diff;
        relative_b.X = -relative_a.X;
        relative_a.Y = cube_properties.max_line_offset - ((z - (center.z - cube_properties.max_draw_z_diff)) * ONE_OVER_SQRT_2);
        relative_b.Y = relative_a.Y;
        rotatePointInitial(relative_a);
        rotatePointInitial(relative_b);
        for (int dir_idx = 0; dir_idx < 3; dir_idx++)//!< draw the line, then rotate 120 degrees.
        {
            a.X = center.x + relative_a.X;
            a.Y = center.y + relative_a.Y;
            b.X = center.x + relative_b.X;
            b.Y = center.y + relative_b.Y;
            addLineAndCombine(directional_line_groups[dir_idx], a, b);
            if (dir_idx < 2)
            {
                rotatePoint120(relative_a);
                rotatePoint120(relative_b);
            }
        }
    }
    for (int idx = 0; idx < 8; idx++) //!< draws the eight children
    {
        if (children[idx] != nullptr)
        {
            children[idx]->generateSubdivisionLines(z, result, directional_line_groups);
        }
    }
}

SubDivCube::SubDivCube(SliceMeshStorage& mesh, Point3& center, unsigned int depth)
{
    this->depth = depth;
    this->center = center;

    if (depth == 0) // lowest layer, no need for subdivision, exit.
    {
        return;
    }
    if (depth >= cube_properties_per_recursion_step.size()) //Depth is out of bounds of what we pre-computed.
    {
        return;
    }

    CubeProperties cube_properties = cube_properties_per_recursion_step[depth];
    Point3 child_center;
    coord_t radius = double(cube_properties.height) / 4.0 + radius_addition;

    int child_nr = 0;
    std::vector<Point3> rel_child_centers;
    rel_child_centers.emplace_back(1, 1, 1); // top
    rel_child_centers.emplace_back(-1, 1, 1); // top three
    rel_child_centers.emplace_back(1, -1, 1);
    rel_child_centers.emplace_back(1, 1, -1);
    rel_child_centers.emplace_back(-1, -1, -1); // bottom
    rel_child_centers.emplace_back(1, -1, -1); // bottom three
    rel_child_centers.emplace_back(-1, 1, -1);
    rel_child_centers.emplace_back(-1, -1, 1);
    for (Point3 rel_child_center : rel_child_centers)
    {
        child_center = center + rotation_matrix.apply(rel_child_center * int32_t(cube_properties.side_length / 4));
        if (isValidSubdivision(mesh, child_center, radius))
        {
            children[child_nr] = new SubDivCube(mesh, child_center, depth - 1);
            child_nr++;
        }
    }
}

bool SubDivCube::isValidSubdivision(SliceMeshStorage& mesh, Point3& center, int64_t radius)
{
    int64_t distance2 = 0;
    coord_t sphere_slice_radius2;//!< squared radius of bounding sphere slice on target layer
    bool inside_somewhere = false;
    bool outside_somewhere = false;
    int inside;
    double part_dist;//what percentage of the radius the target layer is away from the center along the z axis. 0 - 1
    const coord_t layer_height = mesh.getSettingInMicrons("layer_height");
    int bottom_layer = (center.z - radius) / layer_height;
    int top_layer = (center.z + radius) / layer_height;
    for (int test_layer = bottom_layer; test_layer <= top_layer; test_layer += 3) // steps of three. Low-hanging speed gain.
    {
        part_dist = (double)(test_layer * layer_height - center.z) / radius;
        sphere_slice_radius2 = radius * radius * (1.0 - (part_dist * part_dist));
        Point loc(center.x, center.y);

        inside = distanceFromPointToMesh(mesh, test_layer, loc, &distance2);
        if (inside == 1)
        {
            inside_somewhere = true;
        }
        else
        {
            outside_somewhere = true;
        }
        if (outside_somewhere && inside_somewhere)
        {
            return true;
        }
        if ((inside != 2) && distance2 < sphere_slice_radius2)
        {
            return true;
        }
    }
    return false;
}

int SubDivCube::distanceFromPointToMesh(SliceMeshStorage& mesh, int layer_nr, Point& location, int64_t* distance2)
{
    if (layer_nr < 0 || (unsigned int)layer_nr >= mesh.layers.size()) //!< this layer is outside of valid range
    {
        return 2;
        *distance2 = 0;
    }
    Polygons collide;
    mesh.layers[layer_nr].getSecondOrInnermostWalls(collide);
    Point centerpoint = location;
    bool inside = collide.inside(centerpoint);
    ClosestPolygonPoint border_point = PolygonUtils::moveInside2(collide, centerpoint);
    Point diff = border_point.location - location;
    *distance2 = vSize2(diff);
    if (inside)
    {
        return 1;
    }
    return 0;
}


void SubDivCube::rotatePointInitial(Point& target)
{
    target = infill_rotation_matrix.apply(target);
}

void SubDivCube::rotatePoint120(Point& target)
{
    constexpr double sqrt_three_fourths = 0.8660254037844386467637231707529361834714026269051903; //!< sqrt(3.0 / 4.0) = sqrt(3) / 2
    int64_t x;
    x = (-0.5) * target.X - sqrt_three_fourths * target.Y;
    target.Y = (-0.5)*target.Y + sqrt_three_fourths * target.X;
    target.X = x;
}

void SubDivCube::addLineAndCombine(Polygons& group, Point from, Point to)
{
    int epsilon = 10; // the smallest distance of two points which are viewed as coincident (dist > 0 due to rounding errors)
    for (unsigned int idx = 0; idx < group.size(); idx++)
    {
        if (std::abs(from.X - group[idx][1].X) < epsilon && std::abs(from.Y - group[idx][1].Y) < epsilon)
        {
            from = group[idx][0];
            group.remove(idx);
            idx--;
            continue;
        }
        if (std::abs(to.X - group[idx][0].X) < epsilon && std::abs(to.Y - group[idx][0].Y) < epsilon)
        {
            to = group[idx][1];
            group.remove(idx);
            idx--;
            continue;
        }
    }
    group.addLine(from, to);
}

}//namespace cura
