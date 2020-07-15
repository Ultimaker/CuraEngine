//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATHORDEROPTIMIZER_H
#define PATHORDEROPTIMIZER_H

#include "settings/EnumSettings.h" //To get the seam settings.
#include "utils/polygonUtils.h"
#include "utils/linearAlg2D.h" //To find the angle of corners to hide seams.

namespace cura {

/*!
 * Helper class that encapsulates the various criteria that define the location
 * of the z-seam.
 * Instances of this are passed to the PathOrderOptimizer to specify where the
 * seam is to be located.
 */
struct ZSeamConfig
{
    /*!
     * Strategy to place the seam (user-specified, shortest distance, sharpest
     * corner, etc.).
     */
    EZSeamType type;

    /*!
     * When using a user-specified position for the seam, this is the position
     * that the user specified.
     */
    Point pos;

    /*!
     * Corner preference type, if using the sharpest corner strategy.
     */
    EZSeamCornerPrefType corner_pref;

    /*!
     * Default constructor for use when memory must be allocated before it gets
     * filled (like with some data structures).
     *
     * This will select the "shortest" seam strategy.
     */
    ZSeamConfig()
    : type(EZSeamType::SHORTEST)
    , pos(Point(0, 0))
    , corner_pref(EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
    {
    }

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, when using the sharpest corner
     * strategy.
     */
    ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref)
    : type(type)
    , pos(pos)
    , corner_pref(corner_pref)
    {
    }
};

/*!
 * Path order optimization class.
 * 
 * Utility class for optimizing the order in which things are printed, by
 * minimizing the distance traveled between different items to be printed. For
 * each item to be printed, it also chooses a starting point as to where on the
 * polygon or polyline to start printing, and determines which direction to
 * print in.
 *
 * To use this class, first create an instance and provide some parameters as
 * metadata. Then add polygons and polylines to the class. Then call the
 * \ref optimize function to compute the optimization. Finally, print the
 * polygons and polylines in the \ref paths field in the order in which they are
 * given.
 *
 * In the output of this class, polylines and polygons are combined into a
 * single vector: \ref paths . Each path contains a pointer to the original
 * polygon data, as well as whether that data represented a polygon or a
 * polyline, which direction to print that path in, and where to start along the
 * path.
 *
 * The optimizer will always start a polyline from either end, never halfway.
 * The Z seam is not used for these.
 * \tparam PathType The type of paths that will be optimized by this optimizer.
 * Make sure that a specialization is available for \ref get_vertex and
 * \ref get_size in order for the optimizer to know how to read information from
 * your path.
 */
template<typename PathType>
class PathOrderOptimizer
{
public:
    /*!
     * Represents a path which has been optimized, the output of the
     * optimization.
     *
     * This small data structure contains the vertex data of a path,  where to
     * start along the path and in which direction to print it, as well as
     * whether the path should be closed (in case of a polygon) or open (in case
     * of a polyline.
     *
     * After optimization is completed, the \ref paths vector will be filled
     * with optimized paths.
     */
    struct Path
    {
        /*!
         * Construct a new path.
         */
        Path(const PathType& vertices, const bool is_closed = false, const size_t start_vertex = 0, const bool backwards = false)
        : vertices(vertices)
        , start_vertex(start_vertex)
        , is_closed(is_closed)
        , backwards(backwards)
        {
        }

        /*!
         * The vertex data of the path.
         */
        const PathType vertices;

        /*!
         * Which vertex along the path to start printing with.
         *
         * If this path represents a polyline, this will always be one of the
         * endpoints of the path; either 0 or ``vertices->size() - 1``.
         */
        size_t start_vertex;

        /*!
         * Whether the path should be closed at the ends or not.
         *
         * If this path should be closed, it represents a polygon. If it should
         * not be closed, it represents a polyline.
         */
        bool is_closed;

        /*!
         * Whether the path should be traversed in backwards direction.
         *
         * For a polyline it may be more efficient to print the path in
         * backwards direction, if the last vertex is closer than the first.
         */
        bool backwards;

        /*!
         * Swap position with another path.
         *
         * This is required in order to use std::swap on the paths, or to use
         * algorithms like std::reverse.
         */
        void swap(Path& other)
        {
            std::swap(vertices, other.vertices);
            std::swap(start_vertex, other.start_vertex);
            std::swap(is_closed, other.is_closed);
            std::swap(backwards, other.backwards);
        }
    };

    /*!
     * After optimizing, this contains the paths that need to be printed in the
     * correct order.
     *
     * Each path contains the information necessary to print the parts: A
     * pointer to the vertex data, whether or not to close the loop, the
     * direction in which to print the path and where to start the path.
     */
    std::vector<Path> paths;

    /*!
     * The location where the nozzle is assumed to start from before printing
     * these parts.
     */
    Point start_point;

    /*!
     * Seam settings.
     */
    ZSeamConfig seam_config;

    /*!
     * Construct a new optimizer.
     *
     * This doesn't actually optimize the order yet, so the ``paths`` field will
     * not be filled yet.
     * \param start_point The location where the nozzle is assumed to start from
     * before printing these parts.
     * \param config Seam settings.
     * \param detect_chains Whether to try to connect endpoints of paths that
     * are close together. If they are just a few microns apart, it will merge
     * the two endpoints together. Also detects if the two endpoints of a
     * polyline are close together, and turns that polyline into a polygon if
     * they are.
     * \param combing_boundary Boundary to avoid when making travel moves.
     */
    PathOrderOptimizer(const Point start_point, const ZSeamConfig seam_config = ZSeamConfig(), const bool detect_chains = false, const Polygons* combing_boundary = nullptr)
    : start_point(start_point)
    , seam_config(seam_config)
    , combing_boundary((combing_boundary != nullptr && combing_boundary->size() > 0) ? combing_boundary : nullptr)
    , detect_chains(detect_chains)
    {
    }

    /*!
     * Add a new polygon to be optimized.
     * \param polygon The polygon to optimize.
     */
    void addPolygon(const PathType& polygon)
    {
        constexpr bool is_closed = true;
        paths.emplace_back(polygon, is_closed);
    }

    /*!
     * Add a new polyline to be optimized.
     * \param polyline The polyline to optimize.
     */
    void addPolyline(const PathType& polyline)
    {
        bool is_closed = false;
        if(detect_chains)
        {
            is_closed = isLoopingPolyline(polyline);
        }
        paths.emplace_back(polyline, is_closed);
    }

    /*!
     * Perform the calculations to optimize the order of the parts.
     *
     * This reorders the \ref paths field and fills their starting vertices and
     * directions.
     */
    void optimize()
    {
        if(paths.empty())
        {
            return;
        }
        ConstPolygonRef vertices = getVertexData(paths[0].vertices); //Placeholder to catch compilation errors.
        //TODO: Implement optimization!
    }

protected:
    /*!
     * If \ref detect_chains is enabled, endpoints of polylines that are closer
     * than this distance together will be considered to be coincident.
     */
    constexpr static coord_t coincident_point_distance = 5;

    /*!
     * Some input data structures need to be converted to polygons before use.
     * For those, we need to store the vertex data somewhere during the lifetime
     * of the object. Store them here.
     *
     * For example, if the ``PathType`` is a list of ``ExtrusionJunction``s,
     * this will store the coordinates of those junctions.
     */
    std::vector<Polygon> cached_vertices;

    /*!
     * Hash map storing where each line is.
     *
     * This allows us to quickly find any nearby other lines.
     */
    LocToLineGrid* loc_to_line;

    /*!
     * Boundary to avoid when making travel moves.
     */
    const Polygons* combing_boundary;

    /*!
     * Whether to detect chains and loops before optimizing.
     *
     * If this is enabled, the optimizer will first attempt to find endpoints of
     * polylines that are very close together. If they are closer than
     * \ref coincident_point_distance, the polylines will always be ordered end-
     * to-end.
     *
     * This will also similarly detect when the two endpoints of a single
     * polyline are close together, such that it forms a loop. It will turn the
     * polyline into a polygon then.
     */
    bool detect_chains;

    /*!
     * Find the vertex which will be the starting point of printing a polygon or
     * polyline.
     *
     * This will be the seam location (for polygons) or the closest endpoint
     * (for polylines). Usually the seam location is some combination of being
     * the closest point and/or being a sharp inner or outer corner.
     * \param vertices The vertex data of a path. This will never be empty (so
     * no need to check again) but might have size 1.
     * \param target_pos The point that the starting vertex must be close to, if
     * applicable.
     * \param is_closed Whether the polygon is closed (a polygon) or not
     * (a polyline). If the path is not closed, it will choose between the two
     * endpoints rather than 
     * \return An index to a vertex in that path where printing must start.
     */
    size_t findStartLocation(ConstPolygonRef vertices, const Point& target_pos, const bool is_closed) const
    {
        if(!is_closed)
        {
            //For polylines, the seam settings are not applicable. Simply choose the position closest to target_pos then.
            if(vSize2(vertices.back() - target_pos) > vSize2(vertices.front() - target_pos))
            {
                return vertices.size() - 1; //Back end is closer.
            }
            else
            {
                return 0; //Front end is closer.
            }
        }

        //Rest of the function only deals with (closed) polygons. We need to be able to find the seam location of those polygons.

        if(seam_config.type == EZSeamType::RANDOM)
        {
            return getRandomPointInPolygon(vertices);
        }

        size_t best_index = 0;
        float best_score = std::numeric_limits<float>::infinity();
        Point previous = vertices.back();
        for(size_t i = 0; i < vertices.size(); ++i)
        {
            const Point& here = vertices[i];
            const Point& next = vertices[(i + 1) % vertices.size()];

            //For most seam types, the shortest distance matters. Not for SHARPEST_CORNER though.
            //For SHARPEST_CORNER, use a fixed starting score of 0.
            const float score_distance = (seam_config.type == EZSeamType::SHARPEST_CORNER && seam_config.corner_pref != EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE) ? 0 : vSize(here - target_pos);
            const float corner_angle = LinearAlg2D::getAngleLeft(previous, here, next) / M_PI - 1; //Between -1 and 1.

            float score;
            if(seam_config.type == EZSeamType::USER_SPECIFIED) //If user-specified, the seam always needs to be the closest vertex that applies within the filter of the CornerPrefType.
            {
                switch(seam_config.corner_pref)
                {
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER:
                    score = corner_angle > 0 ? score_distance : std::numeric_limits<float>::max(); break; //If it's indeed a concave corner, allow the distance as score. Otherwise a very high score so it won't get picked.
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER:
                    score = corner_angle < 0 ? score_distance : std::numeric_limits<float>::max(); break; //If it's indeed a convex corner, allow the distance as score. Otherwise a very high score so it won't get picked.
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY:
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE:
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED:
                default:
                    score = score_distance; break; //If not filtering on inner or outer corners, just take only distance.
                }
            }
            else //SHARPEST_CORNER or SHORTEST.
            {
                constexpr float corner_shift = 10000 * 10000; //Allow up to 20mm shifting of the seam to find a good location. For SHARPEST_CORNER, this shift is the only factor.
                switch(seam_config.corner_pref)
                {
                default:
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER:
                    score = score_distance;
                    if(corner_angle > 0) //Indeed a concave corner? Give it some advantage over other corners. More advantage for sharper corners.
                    {
                        score -= (corner_angle + 1.0) * corner_shift;
                    }
                    break;
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER:
                    score = score_distance;
                    if(corner_angle < 0) //Indeed a convex corner?
                    {
                        score -= (-corner_angle + 1.0) * corner_shift;
                    }
                    break;
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY:
                    score = score_distance - fabs(corner_angle) * corner_shift; //Still give sharper corners more advantage.
                    break;
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE:
                    score = score_distance; //No advantage for sharper corners.
                    break;
                case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED: //Give sharper corners some advantage, but sharper concave corners even more.
                    {
                        float score_corner = fabs(corner_angle) * corner_shift;
                        if(corner_angle < 0) //Concave corner.
                        {
                            score_corner *= 2;
                        }
                        score = score_distance - score_corner;
                        break;
                    }
                }
            }

            if(score < best_score)
            {
                best_index = i;
                best_score = score;
            }
            previous = here;
        }
        
        return best_index;
    }

    /*!
     * Get a random vertex of a polygon.
     * \param polygon A polygon to get a random vertex of.
     * \return A random index in that polygon.
     */
    size_t getRandomPointInPolygon(ConstPolygonRef polygon) const
    {
        return rand() % polygon.size();
    }

    bool isLoopingPolyline(const PathType& path)
    {
        ConstPolygonRef vertices = getVertexData(path);
        if(vertices.empty())
        {
            return false;
        }
        return vSize2(vertices.back() - vertices[0]) < coincident_point_distance * coincident_point_distance;
    }

    /*!
     * Get vertex data from the custom path type.
     *
     * This is a function that allows the optimization algorithm to work with
     * any type of input data structure. It provides a translation from the
     * input data structure that the user would like to have ordered to a data
     * structure that the optimization algorithm can work with. It's unknown how
     * the ``PathType`` object is structured or how to get the vertex data from
     * it. This function tells the optimizer how, but it needs to be specialized
     * for each different type that this optimizer is used. See the .cpp file
     * for examples and where to add a new specialization.
     */
    ConstPolygonRef getVertexData(const PathType path);
};

} //namespace cura

#endif //PATHORDEROPTIMIZER_H
