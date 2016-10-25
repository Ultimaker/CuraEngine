/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef UTILS_SPARSE_POINT_GRID_INCLUSIVE_H
#define UTILS_SPARSE_POINT_GRID_INCLUSIVE_H

#include <cassert>
#include <unordered_map>
#include <vector>

#include "intpoint.h"
#include "SparsePointGrid.h"

namespace cura {


namespace SparsePointGridInclusiveImpl {

template<class Val>
struct SparsePointGridInclusiveElem
{
    SparsePointGridInclusiveElem()
    {
    }

    SparsePointGridInclusiveElem(const Point &point_, const Val &val_) :
        point(point_),
        val(val_)
    {
    }

    Point point;
    Val val;
};

template<class T>
struct Locatoror
{
    Point operator()(const SparsePointGridInclusiveElem<T> &elem)
    {
        return elem.point;
    }
};

} // namespace SparseGridImpl

/*! \brief Sparse grid which can locate spatially nearby values efficiently.
 *
 * \tparam Val The value type to store.
 */
template<class Val>
class SparsePointGridInclusive : public SparsePointGrid<SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Val>,
                                             SparsePointGridInclusiveImpl::Locatoror<Val> >
{
public:
    using Base = SparsePointGrid<SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Val>,
                                    SparsePointGridInclusiveImpl::Locatoror<Val> >;

    /*! \brief Constructs a sparse grid with the specified cell size.
     *
     * \param[in] cell_size The size to use for a cell (square) in the grid.
     *    Typical values would be around 0.5-2x of expected query radius.
     * \param[in] elem_reserve Number of elements to research space for.
     * \param[in] max_load_factor Maximum average load factor before rehashing.
     */
    SparsePointGridInclusive(coord_t cell_size, size_t elem_reserve=0U, float max_load_factor=1.0f);

    /*! \brief Inserts an element with specified point and value into the sparse grid.
     *
     * This is a convenience wrapper over \ref SparsePointGrid::insert()
     *
     * \param[in] point The location for the element.
     * \param[in] val The value for the element.
     */
    void insert(const Point &point, const Val &val);

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
    std::vector<Val> getNearbyVals(const Point &query_pt, coord_t radius) const;

};

#define SG_TEMPLATE template<class Val>
#define SG_THIS SparsePointGridInclusive<Val>

SG_TEMPLATE
SG_THIS::SparsePointGridInclusive(coord_t cell_size, size_t elem_reserve, float max_load_factor) :
    Base(cell_size,elem_reserve,max_load_factor)
{
}

SG_TEMPLATE
void SG_THIS::insert(const Point &point, const Val &val)
{
    typename SG_THIS::Elem elem(point,val);
    Base::insert(elem);
}

SG_TEMPLATE
std::vector<Val>
SG_THIS::getNearbyVals(const Point &query_pt, coord_t radius) const
{
    std::vector<Val> ret;
    std::function<void (const SparsePointGridInclusiveImpl::SparsePointGridInclusiveElem<Val>&)> process_func = [&ret](const typename SG_THIS::Elem &elem)
        {
            ret.push_back(elem.val);
        };
    this->processNearby(query_pt, radius, process_func);
    return ret;
}


#undef SG_TEMPLATE
#undef SG_THIS

} // namespace cura

#endif // UTILS_SPARSE_POINT_GRID_INCLUSIVE_H
