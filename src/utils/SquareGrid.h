//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_SQUARE_GRID_H
#define UTILS_SQUARE_GRID_H

#include "IntPoint.h"

#include <cassert>
#include <unordered_map>
#include <vector>
#include <functional>

namespace cura {

/*! Square grid
 * 
 * Doesn't contain any data, except cell size
 * 
 * Provides util functions
 */
class SquareGrid
{
public:

    /*! \brief Constructs a grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     */
    SquareGrid(coord_t cell_size);

    coord_t getCellSize() const;

    using GridPoint = Point;
    using grid_coord_t = coord_t;

    /*! \brief Process cells along a line indicated by \p line.
     *
     * \param[in] line The line along which to process cells
     * \param[in] process_func Processes each cell.  process_func(elem) is
     *    called for each cell. Processing stops if function returns false.
     * \return Whether we need to continue processing after this function
     */
    bool processLineCells(const std::pair<Point, Point> line,
                         const std::function<bool (GridPoint)>& process_cell_func);

    /*! \brief Process cells along a line indicated by \p line.
     *
     * \param[in] line The line along which to process cells
     * \param[in] process_func Processes each cell.  process_func(elem) is
     *    called for each cell. Processing stops if function returns false.
     * \return Whether we need to continue processing after this function
     */
    bool processLineCells(const std::pair<Point, Point> line,
                         const std::function<bool (GridPoint)>& process_cell_func) const;

    /*!
     * process all cells in between the line from,to and (to.x,from.y),to
     */
    bool processAxisAlignedTriangle(const Point from, const Point to, const std::function<bool (GridPoint)>& process_cell_func) const;

    bool processAxisAlignedTriangle(const Point from, const Point to, bool to_the_right, const std::function<bool (GridPoint)>& process_cell_func) const;

    /*! \brief Process cells that might contain sought after points.
    * 
    * Processes cells that might be within \p radius of \p query_pt.
    * May process elements that are up to radius + cell_size from query_pt.
    *
    * \param[in] query_pt The point to search around.
    * \param[in] radius The search radius.
    * \param[in] process_func Processes each cell.  process_func(loc) is
    *    called for each cell coord. Processing stops if function returns false.
    * \return Whether we need to continue processing after this function
    */
    bool processNearby(const Point &query_pt, coord_t radius,
                       const std::function<bool (const GridPoint&)>& process_func) const;

    /*! \brief Compute the grid coordinates of a point.
     *
     * \param[in] point The actual location.
     * \return The grid coordinates that correspond to \p point.
     */
    GridPoint toGridPoint(const Point& point) const;

    /*! \brief Compute the grid coordinate of a print space coordinate.
     *
     * \param[in] coord The actual location.
     * \return The grid coordinate that corresponds to \p coord.
     */
    grid_coord_t toGridCoord(const coord_t& coord) const;

    /*! \brief Compute the lowest point in a grid cell.
     * The lowest point is the point in the grid cell closest to the origin.
     *
     * \param[in] location The grid location.
     * \return The print space coordinates that correspond to \p location.
     */
    Point toLowerCorner(const GridPoint& location) const; 

    /*! \brief Compute the lowest coord in a grid cell.
     * The lowest point is the point in the grid cell closest to the origin.
     *
     * \param[in] grid_coord The grid coordinate.
     * \return The print space coordinate that corresponds to \p grid_coord.
     */
    coord_t toLowerCoord(const grid_coord_t& grid_coord) const; 

protected:
    /*! \brief The cell (square) size. */
    coord_t m_cell_size;

    grid_coord_t nonzero_sign(const grid_coord_t z) const;
};

} // namespace cura

#endif // UTILS_SQUARE_GRID_H
