#ifndef INFILL_SUBDIVCUBE_H
#define INFILL_SUBDIVCUBE_H

#include "../sliceDataStorage.h"

namespace cura
{

class Infill;

class SubDivCube
{
public:
    /*!
     * Constructor for SubDivCube. Recursively calls itself eight times to flesh out the octree.
     * \param mesh contains infill layer data and settings
     * \param my_center the center of the cube
     * \param depth the recursion depth of the cube (0 is most recursed)
     */
    SubDivCube(SliceMeshStorage& mesh, Point3& center, int depth);

    ~SubDivCube(); //!< destructor (also destroys children

    /*!
     * Precompute the octree of subdivided cubes
     * \param mesh contains infill layer data and settings
     */
    static void precomputeOctree(SliceMeshStorage& mesh);
    /*!
     * Generates the lines of subdivision of the specific cube at the specific layer. It recursively calls itself, so it ends up drawing all the subdivision lines of sub-cubes too.
     * \param z the specified layer height
     * \param result (output) The resulting lines
     */
    void generateSubdivisionLines(int64_t z, Polygons& result);
private:
    /*!
     * Generates the lines of subdivision of the specific cube at the specific layer. It recursively calls itself, so it ends up drawing all the subdivision lines of sub-cubes too.
     * \param z the specified layer height
     * \param result (output) The resulting lines
     * \param directional_line_groups Array of 3 times a polylines. Used to keep track of line segments that are all pointing the same direction for line segment combining
     */
    void generateSubdivisionLines(int64_t z, Polygons& result, Polygons (&directional_line_groups)[3]);
    struct CubeProperties
    {
        int64_t side_length; //!< side length of cubes
        int64_t height; //!< height of cubes based. This is the distance from one point of a cube to its 3d opposite.
        int64_t square_height; //!< square cut across lengths. This is the diagonal distance across a face of the cube.
        int64_t max_draw_z_diff; //!< maximum draw z differences. This is the maximum difference in z at which lines need to be drawn.
        int64_t max_line_offset; //!< maximum line offsets. This is the maximum distance at which subdivision lines should be drawn from the 2d cube center.
    };
    /*!
     * Rotates a point 120 degrees about the origin.
     * \param target the point to rotate.
     */
    static void rotatePoint120(Point& target);
    /*!
     * Rotates a point to align it with the orientation of the infill.
     * \param target the point to rotate.
     */
    static void rotatePointInitial(Point& target);
    /*!
     * Determines if a described theoretical cube should be subdivided based on if a sphere that encloses the cube touches the infill mesh.
     * \param mesh contains infill layer data and settings
     * \param center the center of the described cube
     * \param radius the radius of the enclosing sphere
     * \return the described cube should be subdivided
     */
    static bool isValidSubdivision(SliceMeshStorage& mesh, Point3& center, int64_t radius);
    /*!
     * Finds the distance to the infill border at the specified layer from the specified point.
     * \param mesh contains infill layer data and settings
     * \param layer_nr the number of the specified layer
     * \param location the location of the specified point
     * \param[out] distance2 the squared distance to the infill border
     * \return Code 0: outside, 1: inside, 2: boundary does not exist at specified layer
     */
    static int distanceFromPointToMesh(SliceMeshStorage& mesh, long int layer_nr, Point& location, int64_t* distance2);

    /*!
     * Adds the defined line to the specified polygons. It assumes that the specified polygons are all parallel lines. Combines line segments with touching ends closer than epsilon.
     * \param[out] group the polygons to add the line to
     * \param from the first endpoint of the line
     * \param to the second endpoint of the line
     */
    void addLineAndCombine(Polygons& group, Point from, Point to);

    int depth; //!< the recursion depth of the cube (0 is most recursed)
    Point3 center; //!< center location of the cube in absolute coordinates
    SubDivCube* children[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}; //!< pointers to this cube's eight octree children
    static std::vector<CubeProperties> cube_properties_per_recursion_step; //!< precomputed array of basic properties of cubes based on recursion depth.
    static double radius_multiplier; //!< multiplier for the bounding radius when determining if a cube should be subdivided
    static Point3Matrix rotation_matrix; //!< The rotation matrix to get from axis aligned cubes to cubes standing on a corner point aligned with the infill_angle
    static PointMatrix infill_rotation_matrix; //!< Horizontal rotation applied to infill
    static int32_t radius_addition; //!< addition to the bounding radius when determining if a cube should be subdivided
    static constexpr double one_over_sqrt_2 = 0.7071067811865475244008443621048490392848359376884740; //!< 1.0 / sqrt(2.0)
};

}
#endif //INFILL_SUBDIVCUBE_H
