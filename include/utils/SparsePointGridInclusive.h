// Copyright (c) 2026 Scott Lenser
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <unordered_map>
#include <vector>

#include "utils/sparse_point_grid_inclusive_impl/Locatoror.h"
#include "SparsePointGrid.h"
#include "utils/sparse_point_grid_inclusive_impl/SparsePointGridInclusiveElem.h"
#include "geometry/Point2LL.h"

namespace curaengine::utils
{


/*! \brief Sparse grid which can locate spatially nearby values efficiently.
 *
 * \tparam Val The value type to store.
 */
template<class Val>
class SparsePointGridInclusive : public SparsePointGrid<::curaengine::utils::sparse_point_grid_inclusive_impl::SparsePointGridInclusiveElem<Val>, ::curaengine::utils::sparse_point_grid_inclusive_impl::Locatoror<Val>>
{
public:
    using Base = SparsePointGrid<::curaengine::utils::sparse_point_grid_inclusive_impl::SparsePointGridInclusiveElem<Val>, ::curaengine::utils::sparse_point_grid_inclusive_impl::Locatoror<Val>>;

    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     * \param[in] elem_reserve Number of elements to research space for.
     * \param[in] max_load_factor Maximum average load factor before rehashing.
     */
    SparsePointGridInclusive(coord_t cell_size, size_t elem_reserve = 0U, double max_load_factor = 1.0);

    /*! \brief Inserts an element with specified point and value into the sparse grid.
     *
     * This is a convenience wrapper over \ref SparsePointGrid::insert()
     *
     * \param[in] point The location for the element.
     * \param[in] val The value for the element.
     */
    void insert(const ::curaengine::geometry::Point2LL& point, const Val& val);

    /*! \brief Returns all values within radius of query_pt.
     *
     * Finds all values with location within radius of \p query_pt.  May
     * return additional values that are beyond radius.
     *
     * See \ref getNearby().
     *
     * \param[in] query_pt The point to search around.
     * \param[in] radius The search radius.
     * \return Vector of values found
     */
    std::vector<Val> getNearbyVals(const ::curaengine::geometry::Point2LL& query_pt, coord_t radius) const;
};

#define SG_TEMPLATE template<class Val>
#define SG_THIS SparsePointGridInclusive<Val>

SG_TEMPLATE
SG_THIS::SparsePointGridInclusive(coord_t cell_size, size_t elem_reserve, double max_load_factor)
    : Base(cell_size, elem_reserve, max_load_factor)
{
}

SG_TEMPLATE
void SG_THIS::insert(const ::curaengine::geometry::Point2LL& point, const Val& val)
{
    typename SG_THIS::Elem elem(point, val);
    Base::insert(elem);
}

SG_TEMPLATE
std::vector<Val> SG_THIS::getNearbyVals(const ::curaengine::geometry::Point2LL& query_pt, coord_t radius) const
{
    std::vector<Val> ret;
    std::function<bool(const ::curaengine::utils::sparse_point_grid_inclusive_impl::SparsePointGridInclusiveElem<Val>&)> process_func = [&ret](const typename SG_THIS::Elem& elem)
    {
        ret.push_back(elem.val);
        return true;
    };
    this->processNearby(query_pt, radius, process_func);
    return ret;
}


#undef SG_TEMPLATE
#undef SG_THIS

} // namespace curaengine::utils
