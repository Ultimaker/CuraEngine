/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */

#include <cassert>
#include <sstream>  // debug TODO
#include <unordered_set>

#include "../utils/math.h"
#include "../utils/linearAlg2D.h"
#include "../utils/gettime.h"


namespace cura {


template<typename CellGeometry>
typename InfillFractal2D<CellGeometry>::Direction InfillFractal2D<CellGeometry>::opposite(InfillFractal2D<CellGeometry>::Direction in) const
{
    switch(in)
    {
        case Direction::LEFT: return Direction::RIGHT;
        case Direction::RIGHT: return Direction::LEFT;
        case Direction::UP: return Direction::DOWN;
        case Direction::DOWN: return Direction::UP;
        default: return Direction::COUNT;
    }
}

template<typename CellGeometry>
uint_fast8_t InfillFractal2D<CellGeometry>::opposite(uint_fast8_t in) const
{
    return static_cast<uint_fast8_t>(opposite(static_cast<Direction>(in)));
}



template<typename CellGeometry>
typename InfillFractal2D<CellGeometry>::ChildSide InfillFractal2D<CellGeometry>::toChildSide(uint_fast8_t in) const
{
    return static_cast<ChildSide>(in);
}

template<typename CellGeometry>
uint_fast8_t InfillFractal2D<CellGeometry>::toInt(ChildSide in) const
{
    return static_cast<uint_fast8_t>(in);
}

template<typename CellGeometry>
typename InfillFractal2D<CellGeometry>::ChildSide InfillFractal2D<CellGeometry>::opposite(ChildSide in, uint_fast8_t dimension) const
{
    switch(in)
    { //                                                  flip over Z                 flip over X
        case ChildSide::LEFT_BOTTOM:    return dimension? ChildSide::LEFT_TOP       : ChildSide::RIGHT_BOTTOM;
        case ChildSide::RIGHT_BOTTOM:   return dimension? ChildSide::RIGHT_TOP      : ChildSide::LEFT_BOTTOM;
        case ChildSide::LEFT_TOP:       return dimension? ChildSide::LEFT_BOTTOM    : ChildSide::RIGHT_TOP;
        case ChildSide::RIGHT_TOP:      return dimension? ChildSide::RIGHT_BOTTOM   : ChildSide::LEFT_TOP;
        default: return ChildSide::COUNT;
    }
}

template<typename CellGeometry>
typename InfillFractal2D<CellGeometry>::Direction InfillFractal2D<CellGeometry>::getChildToNeighborChildDirection(ChildSide in, uint_fast8_t dimension) const
{
    switch(in)
    { //                                                  flip over Z          flip over X
        case ChildSide::LEFT_BOTTOM:    return dimension? Direction::UP     : Direction::RIGHT;
        case ChildSide::RIGHT_BOTTOM:   return dimension? Direction::UP     : Direction::LEFT;
        case ChildSide::LEFT_TOP:       return dimension? Direction::DOWN   : Direction::RIGHT;
        case ChildSide::RIGHT_TOP:      return dimension? Direction::DOWN   : Direction::LEFT;
        default: return Direction::COUNT;
    }
}

template<typename CellGeometry>
uint_fast8_t InfillFractal2D<CellGeometry>::Cell::getChildCount() const
{
    if (children[0] < 0)
    {
        return 0;
    }
    else if (children[2] < 0)
    {
        return 2;
    }
    else
    {
        return 4;
    }
}

template<typename CellGeometry>
InfillFractal2D<CellGeometry>::InfillFractal2D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width, bool root_is_bogus)
: root_is_bogus(root_is_bogus)
, aabb(aabb)
, max_depth(max_depth)
, line_width(line_width)
, density_provider(density_provider)
{
}

template<typename CellGeometry>
InfillFractal2D<CellGeometry>::~InfillFractal2D()
{
}

template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::initialize()
{
    TimeKeeper tk;
    createTree();
    debugCheckDepths();
    debugCheckVolumeStats();
    logDebug("Created InfillFractal2D tree with %i nodes and max depth %i in %5.2fs.\n", cell_data.size(), max_depth, tk.restart());
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::setSpecificationAllowance(Cell& sub_tree_root)
{
    bool has_children = sub_tree_root.children[0] >= 0;
    if (has_children)
    {
        for (idx_t child_idx : sub_tree_root.children)
        {
            if (child_idx < 0)
            {
                break;
            }
            Cell& child = cell_data[child_idx];
            setSpecificationAllowance(child);
            sub_tree_root.filled_volume_allowance += child.filled_volume_allowance;
            sub_tree_root.minimally_required_density = std::max(sub_tree_root.minimally_required_density, child.minimally_required_density);
        }
    }
    else
    {
        float requested_density = getDensity(sub_tree_root);
        sub_tree_root.minimally_required_density = requested_density;
        sub_tree_root.filled_volume_allowance = sub_tree_root.volume * requested_density;
    }
}

/*
 * Tree creation /\                         .
 * 
 * =======================================================
 * 
 * Lower bound sequence \/                  .
 */


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::createMinimalDensityPattern()
{
    TimeKeeper tk;
    std::list<idx_t> all_to_be_subdivided;
    
    std::function<bool(const Cell&)> shouldBeSubdivided =
        [this](const Cell& cell)
        {
            return getActualizedVolume(cell) / cell.volume < cell.minimally_required_density;
        };
    
    assert(cell_data.size() > 0);

    if (root_is_bogus)
    { // always subdivide the root, which is a bogus node!
        Cell& root = cell_data[0];
        for (idx_t child_idx : root.children)
        {
            if (child_idx >= 0 && shouldBeSubdivided(cell_data[child_idx]))
            {
                all_to_be_subdivided.push_back(child_idx);
            }
        }
    }
    else
    {
        all_to_be_subdivided.push_back(0); // check whether we need to subdivide root
    }
    
    while (!all_to_be_subdivided.empty())
    {
        idx_t to_be_subdivided_idx = all_to_be_subdivided.front();
        Cell& to_be_subdivided = cell_data[to_be_subdivided_idx];

        if (to_be_subdivided.children[0] < 0 || to_be_subdivided.depth >= max_depth)
        { // this is a leaf cell
            all_to_be_subdivided.pop_front();
            continue;
        }

        if (!isConstrained(to_be_subdivided))
        {
            all_to_be_subdivided.pop_front();
            constexpr bool redistribute_errors = false;
            subdivide(to_be_subdivided, redistribute_errors);
            for (idx_t child_idx : to_be_subdivided.children)
            {
                if (child_idx >= 0 && shouldBeSubdivided(cell_data[child_idx]))
                {
                    all_to_be_subdivided.push_back(child_idx);
                }
            }
        }
        else
        {
//             all_to_be_subdivided.push_front(to_be_subdivided_idx); // retry after subdividing all neighbors
            for (std::list<Link>& side : to_be_subdivided.adjacent_cells)
            {
                for (Link& neighbor : side)
                {
                    if (isConstrainedBy(to_be_subdivided, cell_data[neighbor.to_index]))
                    {
                        all_to_be_subdivided.push_front(neighbor.to_index);
                    }
                }
            }
        }
    }
    logDebug("InfillFractal2D::createMinimalDensityPattern finished in %5.2fs.\n", tk.restart());
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::createDitheredPattern()
{
    createBalancedPattern();

    settleLoans();

    dither(cell_data[0]);

    // debug check for total actualized volume
    float total_actualized_volume = getTotalActualizedVolume(cell_data[0]);
    float total_requested_volume = cell_data[0].filled_volume_allowance;
    logAlways("Realized %f of %f requested volume (%f%% error).\n", total_actualized_volume, total_requested_volume, (total_actualized_volume * 100.0 / total_requested_volume) - 100.0);
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::createMinimalErrorPattern(bool middle_decision_boundary)
{
    std::list<Cell*> to_be_checked;
    to_be_checked.push_back(&cell_data[0]);

    while (!to_be_checked.empty())
    {
        Cell* checking = to_be_checked.front();
        to_be_checked.pop_front();

        if (checking->children[0] < 0)
        { // cell has no children
            continue;
        }

        float decision_boundary = (middle_decision_boundary)?
                                    (getActualizedVolume(*checking) + getChildrenActualizedVolume(*checking)) / 2
                                    : getChildrenActualizedVolume(*checking);
        if (canSubdivide(*checking) && checking->filled_volume_allowance > decision_boundary)
        {
            constexpr bool redistribute_errors = true;
            subdivide(*checking, redistribute_errors);
            for (idx_t child_idx : checking->children)
            {
                if (child_idx < 0)
                {
                    break;
                }
                to_be_checked.push_back(&cell_data[child_idx]);
            }
        }
    }
}

template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::createBalancedPattern()
{
    for (int iteration = 0; iteration < 999; iteration++)
    {
        bool change = false;
        change |= subdivisionPhase();

        change |= handOutLoansPhase();

        if (!change)
        {
            logDebug("Finished after %i iterations, with a max depth of %i.\n", iteration + 1, max_depth);
            break;
        }
    }
}


template<typename CellGeometry>
std::vector<std::vector<typename InfillFractal2D<CellGeometry>::Cell*>> InfillFractal2D<CellGeometry>::getDepthOrdered()
{
    std::vector<std::vector<Cell*>> depth_ordered(max_depth + 1);
    depth_ordered.resize(max_depth + 1);
    getDepthOrdered(cell_data[0], depth_ordered);
    return depth_ordered;
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::getDepthOrdered(Cell& sub_tree_root, std::vector<std::vector<Cell*>>& output)
{
    if (sub_tree_root.is_subdivided)
    {
        for (idx_t child_idx : sub_tree_root.children)
        {
            if (child_idx < 0)
            {
                break;
            }
            getDepthOrdered(cell_data[child_idx], output);
        }
    }
    else
    {
        assert(sub_tree_root.depth >= 0);
        assert(static_cast<size_t>(sub_tree_root.depth) < output.size());
        output[sub_tree_root.depth].push_back(&sub_tree_root);
    }
}

template<typename CellGeometry>
bool InfillFractal2D<CellGeometry>::subdivisionPhase()
{
    std::vector<std::vector<Cell*>> depth_ordered = getDepthOrdered();

    bool change = false;
    for (std::vector<Cell*>& depth_nodes : depth_ordered)
        for (Cell* cell : depth_nodes)
        {
            bool is_constrained = isConstrained(*cell);

            if (cell->depth == max_depth) //Never subdivide beyond maximum depth.
            {
                continue;
            }
            float total_subdiv_error = getSubdivisionError(*cell);
            if (
                total_subdiv_error >= 0
                && !is_constrained
                )
            {
                constexpr bool redistribute_errors = true;
                subdivide(*cell, redistribute_errors);
                change = true;
            }
        }
    return change;
}

template<typename CellGeometry>
bool InfillFractal2D<CellGeometry>::handOutLoansPhase()
{
    std::vector<std::vector<Cell*>> depth_ordered = getDepthOrdered();

    bool redistributed_anything = false;

    for (int depth = max_depth; depth >= 0; depth--)
    {
        std::vector<Cell*>& depth_nodes = depth_ordered[depth];
        for (Cell* cell : depth_nodes)
        {
            debugCheckLoans(*cell);
            if (!isConstrained(*cell))
            {
                continue;
            }
            float unresolvable_error = getValueError(*cell); // the value allowance this cell cannot use because it is constrianed, while it would like to subdivide
            if (unresolvable_error < allowed_volume_error)
            {
                continue;
            }
            // hand out loans proportional to the volume
            float weighted_constrainer_count = 0.0;

            for (const std::list<Link>& neighbors_in_a_given_direction : cell->adjacent_cells)
            {
                const float weight = 1.0; // TODO: add weights based on direction : UP/DOWN vs L/R might get differnt weights in cross3D
                for (const Link& link : neighbors_in_a_given_direction)
                {
                    if (isConstrainedBy(*cell, cell_data[link.to_index]))
                    {
                        weighted_constrainer_count += weight;
                    }
                }
            }

            assert(weighted_constrainer_count > allowed_volume_error && "a constrained cell must have a positive weighted constrainer count!");

            for (std::list<Link>& neighbors_in_a_given_direction : cell->adjacent_cells)
            {
                const float weight = 1.0; // TODO: add weights based on direction : UP/DOWN vs L/R might get differnt weights in cross3D
                for (Link& link : neighbors_in_a_given_direction)
                {
                    if (isConstrainedBy(*cell, cell_data[link.to_index]))
                    {
                        float value_transfer = unresolvable_error * weight / weighted_constrainer_count;
                        // first reduce the loan that cell has passed to this cell (occurs very rarely in cases where a cell gets subdivided because of another neighbor (?))
                        float value_transfer_reduction = std::min(value_transfer, link.getReverse().loan);
                        link.getReverse().loan -= value_transfer_reduction;
                        value_transfer -= value_transfer_reduction;
                        link.loan += value_transfer;
                    }
                }
            }

            debugCheckLoans(*cell);
            redistributed_anything = true;
        }
    }
    return redistributed_anything;
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::dither(Cell& parent)
{
    if (parent.is_subdivided)
    {
        for (int path_dir = static_cast<int>(ChildSide::FIRST); path_dir < static_cast<int>(ChildSide::COUNT); path_dir++)
        {
            idx_t child_idx = parent.children[path_dir];
            if (child_idx > 0)
            {
                Cell& child = cell_data[child_idx];
                dither(child);
            }
        }
    }
    else
    {
        const float balance = getValueError(parent);
        const float parent_actualized_volume = getActualizedVolume(parent);
        const float subdivided_actualized_volume = getChildrenActualizedVolume(parent);
        const float range = subdivided_actualized_volume - parent_actualized_volume;
//         assert(range > -allowed_volume_error || parent.getChildCount() == 0); // TODO: investigate why this doesn't hold!
//         const float decision_boundary = (rand() % 100) * range / 100;
//         const float decision_boundary = (.01 * (rand() % 101) * .5 + .25) * range;
        const float decision_boundary = range / 2;
        bool do_subdivide = balance > decision_boundary && canSubdivide(parent);
        float actualized_volume = (do_subdivide)? subdivided_actualized_volume : parent_actualized_volume;

        const float left_over = balance - (actualized_volume - parent_actualized_volume); // might be negative

        // divide left_over equally per cell capacity, which is linearly related to the cell volume
        float total_weighted_forward_cell_volume = 0;
        Direction forwards[] = { Direction::RIGHT, Direction::UP };

        /* Stucki weights:
         * 1 2 1
         *   * 2
         * 
         * Note: these weights work better than Floyd-Steinberg-like weights.
         * Also minor variations to either of these two basic weight matrices seem to perform worse.
         */

        float direction_weights[] = { 2, 2 };
        float diag_weight = 1;
        float backward_diag_weight = 1;

        for (int side_idx = 0; side_idx < 2; side_idx++)
        {
            Direction direction = forwards[side_idx];
            std::list<Link>& side = parent.adjacent_cells[static_cast<size_t>(direction)];
            for (Link& link : side)
            {
                Cell& neighbor = cell_data[link.to_index];
                assert(neighbor.volume > 0 && "volume must be computed");
                total_weighted_forward_cell_volume += direction_weights[side_idx] * neighbor.volume;
            }
        }
        Link* diag_neighbor = getDiagonalNeighbor(parent, Direction::RIGHT);
        if (diag_neighbor)
        {
            total_weighted_forward_cell_volume += diag_weight * cell_data[diag_neighbor->to_index].volume;
        }
        Link* backward_diag_neighbor = getDiagonalNeighbor(parent, Direction::LEFT);
        if (backward_diag_neighbor && cell_data[backward_diag_neighbor->to_index].is_dithered)
        {
            /*!
             * avoid error being propagated to already processed cells!!:
             *  ___ ___ ___ ___
             * | 3 | 4 | 7 | 8 |
             * |___|__↖|___|___|
             * | 1 | 2 |↖5 | 6 |
             * |___|___|___|___|
             */
            backward_diag_neighbor = nullptr;
        }
        if (backward_diag_neighbor)
        {
            total_weighted_forward_cell_volume += backward_diag_weight * cell_data[backward_diag_neighbor->to_index].volume;
        }

        assert(total_weighted_forward_cell_volume >= 0.0);
        assert(total_weighted_forward_cell_volume < 27000000.0); // 30x30x30cm
        for (int side_idx = 0; side_idx < 2; side_idx++)
        {
            Direction direction = forwards[side_idx];
            std::list<Link>& side = parent.adjacent_cells[static_cast<size_t>(direction)];
            for (Link& neighbor : side)
            {
                Link& loan_link = (left_over > 0)? neighbor : neighbor.getReverse();
                loan_link.loan += std::abs(left_over) * direction_weights[side_idx] * cell_data[neighbor.to_index].volume / total_weighted_forward_cell_volume;
            }
        }
        if (diag_neighbor)
        {
            Link& diag_loan_link = (left_over > 0)? *diag_neighbor : diag_neighbor->getReverse();
            float transfer = std::abs(left_over) * diag_weight * cell_data[diag_neighbor->to_index].volume / total_weighted_forward_cell_volume;
            // transfer the amount via the correct upstairs neighbor
            assert(!parent.adjacent_cells[toInt(Direction::UP)].empty());
            Link& up_neighbor = parent.adjacent_cells[toInt(Direction::UP)].back();
            Link& up_loan_link = (left_over > 0)? up_neighbor : up_neighbor.getReverse();
            up_loan_link.loan += transfer;
            diag_loan_link.loan += transfer;
        }
        if (backward_diag_neighbor)
        {
            Link& diag_loan_link = (left_over > 0)? *backward_diag_neighbor : backward_diag_neighbor->getReverse();
            float transfer = std::abs(left_over) * backward_diag_weight * cell_data[backward_diag_neighbor->to_index].volume / total_weighted_forward_cell_volume;
            // transfer the amount via the correct upstairs neighbor
            assert(!parent.adjacent_cells[toInt(Direction::UP)].empty());
            Link& up_neighbor = parent.adjacent_cells[toInt(Direction::UP)].front();
            Link& up_loan_link = (left_over > 0)? up_neighbor : up_neighbor.getReverse();
            up_loan_link.loan += transfer;
            diag_loan_link.loan += transfer;
        }

        if (do_subdivide)
        {
            constexpr bool redistribute_errors = false;
            subdivide(parent, redistribute_errors);
        }

        parent.is_dithered = true;
        if (do_subdivide)
        {
            for (idx_t child_idx : parent.children)
            {
                if (child_idx < 0) break;
                cell_data[child_idx].is_dithered = true;
            }
        }
    }
}

template<typename CellGeometry>
float InfillFractal2D<CellGeometry>::getChildrenActualizedVolume(const Cell& cell) const
{
    // The actualized volume of squares doesn't depend on surrounding cells,
    // so we just call getActualizedVolume(.)
    float actualized_volume = 0;
    for (idx_t child_idx : cell.children)
    {
        if (child_idx < 0)
        {
            continue;
        }
        actualized_volume += getActualizedVolume(cell_data[child_idx]);
    }
    return actualized_volume;
}

template<typename CellGeometry>
bool InfillFractal2D<CellGeometry>::canSubdivide(const Cell& cell) const
{
    if (cell.depth >= max_depth || isConstrained(cell))
    {
        return false;
    }
    else
    {
        return true;
    }
}

template<typename CellGeometry>
bool InfillFractal2D<CellGeometry>::isConstrained(const Cell& cell) const
{
    for (const std::list<Link>& side : cell.adjacent_cells)
    {
        for (const Link& neighbor : side)
        {
            if (isConstrainedBy(cell, cell_data[neighbor.to_index]))
            {
                return true;
            }
        }
    }
    return false;
}

template<typename CellGeometry>
bool InfillFractal2D<CellGeometry>::isConstrainedBy(const Cell& constrainee, const Cell& constrainer) const
{
    return constrainer.depth < constrainee.depth;
}



template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::subdivide(Cell& cell, bool redistribute_errors)
{
    if (redistribute_errors)
    {
        debugCheckLoans(cell);
    }

    assert(cell.children[0] >= 0 && cell.children[1] >= 0 && "Children must be initialized for subdivision!");
    Cell& child_lb = cell_data[cell.children[0]];
    Cell& child_rb = cell_data[cell.children[1]];
    initialConnection(child_lb, child_rb, Direction::RIGHT);

    if (redistribute_errors)
    { // move left-over errors
        settleLoans(cell, getSubdivisionError(cell));
    }

    const float total_loan_error_balance_before = getTotalLoanError(cell);

    if (cell.getChildCount() == 4)
    {
        Cell& child_lt = cell_data[cell.children[2]];
        Cell& child_rt = cell_data[cell.children[3]];
        initialConnection(child_lt, child_rt, Direction::RIGHT);
        initialConnection(child_lb, child_lt, Direction::UP);
        initialConnection(child_rb, child_rt, Direction::UP);
    }

    // reconnect neighbors of the parent to the children
    for (uint_fast8_t side = 0; side < getNumberOfSides(); side++)
    {
        /* two possible cases:
         * 1                                                                             ______          __  __
         * neighbor is refined more                                                   [][      ]      [][  ][  ]
         *      __                                                     deeper example [][      ]  =>  [][__][__]
         * [][][  ]  => [][][][]                                                      [][      ]      [][  ][  ]
         * [][][__]     [][][][]    We have the same amount of links                  [][______]      [][__][__]
         *       ^parent cell
         * 2
         * neighbor is refined less or equal                                           ______  __       ______
         *  __  __        __                                                          [      ][  ]     [      ][][]
         * [  ][  ]  =>  [  ][][]                                                     [      ][__]  => [      ][][]
         * [__][__]      [__][][]                                      deeper example [      ][  ]     [      ][][]
         *       ^parent cell                                                         [______][__]     [______][][]
         * Each link from a neighbor cell is split
         * into two links to two child cells
         * 
         * Both cases are cought by replacing each link with as many as needed,
         * which is either 1 or 2, because
         * in the new situation there are either 1 or 2 child cells neigghboring a beighbor cell of the parent.
         */
        for (LinkIterator neighbor_it = cell.adjacent_cells[side].begin(); neighbor_it != cell.adjacent_cells[side].end(); ++neighbor_it)
        {
            Link& neighbor = *neighbor_it;
            assert(neighbor.reverse);
            Cell& neighboring_cell = cell_data[neighbor.to_index];
            std::list<Link>& neighboring_edge_links = neighboring_cell.adjacent_cells[opposite(side)];
            
            std::list<Link*> new_incoming_links;
            std::list<Link*> new_outgoing_links;
            for (idx_t child_idx : cell.children)
            {
                if (child_idx < 0)
                {
                    break;
                }
                Cell& child = cell_data[child_idx];
                Cell& neighbor_cell = cell_data[neighbor.to_index];
                assert(neighbor.to_index > 0);
                if (isNextTo(child, neighbor_cell, static_cast<Direction>(side)))
                {
                    child.adjacent_cells[side].emplace_front(neighbor.to_index);
                    LinkIterator outlink = child.adjacent_cells[side].begin();

                    neighboring_edge_links.emplace(*neighbor.reverse, child_idx);
                    LinkIterator inlink = *neighbor.reverse;
                    inlink--;

                    outlink->reverse = inlink;
                    inlink->reverse = outlink;

                    new_incoming_links.push_back(&*inlink);
                    new_outgoing_links.push_back(&*outlink);
                }
            }
            // loans should always be transfered from parent links to the new child links, so that we never loose value
            transferLoans(neighbor.getReverse(), new_incoming_links);
            transferLoans(neighbor, new_outgoing_links);

            neighboring_edge_links.erase(*neighbor.reverse);
        }
        cell.adjacent_cells[side].clear();
    }

    cell.is_subdivided = true;

    if (redistribute_errors)
    { // make positive errors in children well balanced
        // Pass along error from parent
        solveChildDebts(cell);
    }
    
    
    // debug check:
    if (redistribute_errors)
    {
        float total_loan_error_balance_after = 0.0;
        for (const idx_t child_idx : cell.children)
        {
            if (child_idx < 0)
            {
                break;
            }
            total_loan_error_balance_after += getTotalLoanError(cell_data[child_idx]);
        }
        assert(std::abs(total_loan_error_balance_after - total_loan_error_balance_before) < allowed_volume_error);
    }

    // debug check:
    
    if (redistribute_errors)
    {
        for (const idx_t child_idx : cell.children)
        {
            if (child_idx < 0)
            {
                break;
            }
            debugCheckLoans(cell_data[child_idx]);
        }
    }

    debugCheckChildrenOverlap(cell);
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::initialConnection(Cell& before, Cell& after, Direction dir)
{
    before.adjacent_cells[static_cast<size_t>(dir)].emplace_front(after.index);
    after.adjacent_cells[static_cast<size_t>(opposite(dir))].emplace_front(before.index);
    LinkIterator before_to_after = before.adjacent_cells[static_cast<size_t>(dir)].begin();
    LinkIterator after_to_before = after.adjacent_cells[static_cast<size_t>(opposite(dir))].begin();
    before_to_after->reverse = after_to_before;
    after_to_before->reverse = before_to_after;
}

template<typename CellGeometry>
typename InfillFractal2D<CellGeometry>::Link* InfillFractal2D<CellGeometry>::getDiagonalNeighbor(Cell& cell, Direction left_right) const
{ // implementation naming assumes left_right is right
    std::list<Link>& right_side = cell.adjacent_cells[static_cast<size_t>(left_right)];
    if (!right_side.empty())
    {
        const std::list<Link>& right_side_up_side = cell_data[right_side.back().to_index].adjacent_cells[static_cast<size_t>(Direction::UP)];
        if (!right_side_up_side.empty())
        {
            const Link& ru_diag_neighbor = right_side_up_side.front();
            /*  ___ ___
             * | 3 | 4 |
             * |___|_↑_|  ru_diag_neighbor is link from 2 to 4
             * | 1...2 |
             * |___|___|
             */
            const std::list<Link>& up_side = cell.adjacent_cells[static_cast<size_t>(Direction::UP)];
            if (!up_side.empty())
            {
                const Link& up_neighbor = (left_right == Direction::RIGHT)? up_side.back() : up_side.front();
                const std::list<Link>& up_side_right_side = cell_data[up_neighbor.to_index].adjacent_cells[static_cast<size_t>(left_right)];
                if (!up_side_right_side.empty())
                {
                    const Link& ur_diag_neighbor = up_side_right_side.front();
                    /*  ___ ___
                     * | 3 → 4 |  ur_diag_neighbor is link from 3 to 4
                     * |_:_|___|
                     * | 1 | 2 |
                     * |___|___|
                     */
                    if (ru_diag_neighbor.to_index == ur_diag_neighbor.to_index)
                    {
                        return const_cast<Link*>(&ur_diag_neighbor);
                    }
                }
            }
        }
    }
    return nullptr;
}


/*
 * Tree creation /\                         .
 * 
 * =======================================================
 * 
 * Error redistribution \/                  .
 */

template<typename CellGeometry>
float InfillFractal2D<CellGeometry>::getValueError(const Cell& cell) const
{
    float balance = cell.filled_volume_allowance - getActualizedVolume(cell) + getTotalLoanError(cell);
    return balance;
}

template<typename CellGeometry>
float InfillFractal2D<CellGeometry>::getSubdivisionError(const Cell& cell) const
{
    float balance = cell.filled_volume_allowance - getChildrenActualizedVolume(cell) + getTotalLoanError(cell);
    return balance;
}

template<typename CellGeometry>
float InfillFractal2D<CellGeometry>::getTotalLoanError(const Cell& cell) const
{
    float loan = 0.0;

    for (const std::list<Link>& neighbors_in_a_given_direction : cell.adjacent_cells)
    {
        for (const Link& link : neighbors_in_a_given_direction)
        {
            assert(std::isfinite(link.loan));
            assert(std::isfinite(link.getReverse().loan));
            loan -= link.loan;
            loan += link.getReverse().loan;
        }
    }
    return loan;
}

template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::solveChildDebts(const Cell& parent)
{
    // Algorithm overview:
    // for each child which has a negative balance (is in debt):
    //   get required amount from its positive balance neighbors
    //   or if no such neighbors present:
    //     get the required amount from both negative balance neighbors
    //
    // Repeat this 2 because the transfered value might make preveously handled children in debt again.
    // We don't need to repeat more, because this function presupposes that the children _can_ be balanced;
    // the total amount of value must be enough to satisfy all children.
    // Therefore, some children must have eough value left over.
    // Supposing the left upper child is the one with the surplus which must be redistributed to
    // its farthest cell: the bottom right cell.
    // This requires at most 2 steps, so we only need two iterations below
    for (uint_fast8_t iter = 0; iter < 2; iter++)
    {
        for (uint_fast8_t child_side = 0; child_side < toInt(ChildSide::COUNT); child_side++)
        {
            ChildSide child_side_to_check = toChildSide(child_side);
            idx_t child_idx = parent.children[toInt(child_side_to_check)];
            if (child_idx < 0)
            {
                continue;
            }
            Cell& child = cell_data[child_idx];
            float value_error = getValueError(child);
            if (value_error < -allowed_volume_error)
            { // the cell is in debt, so it should get value error from its neighbors
                // check from which neighbors we can get value
                float weighted_providing_neighbor_count = 0.0; // number of cells neighboring this cell (either 0, 1 or 2) which can provide error value weighted by their allowance
                for (uint_fast8_t dimension = 0; dimension < 2; dimension++)
                {
                    ChildSide neighbor_child_side = opposite(child_side_to_check, dimension);
                    idx_t neighbor_child_idx = parent.children[toInt(neighbor_child_side)];
                    if (neighbor_child_idx < 0)
                    {
                        continue;
                    }
                    float neighbor_power = getValueError(cell_data[neighbor_child_idx]);
                    if (neighbor_power > allowed_volume_error)
                    {
                        weighted_providing_neighbor_count += neighbor_power;
                    }
                }

                // get value from neigbors
                for (uint_fast8_t dimension = 0; dimension < 2; dimension++)
                {
                    ChildSide neighbor_child_side = opposite(child_side_to_check, dimension);
                    idx_t neighbor_child_idx = parent.children[toInt(neighbor_child_side)];
                    if (neighbor_child_idx < 0)
                    {
                        continue;
                    }
                    float neighbor_power = getValueError(cell_data[neighbor_child_idx]);
                    if (neighbor_power > allowed_volume_error || weighted_providing_neighbor_count == 0.0)
                    {
                        float value_transfer = (weighted_providing_neighbor_count == 0.0)? -value_error * 0.5 : -value_error * neighbor_power / weighted_providing_neighbor_count;
                        Direction child_to_neighbor_direction = getChildToNeighborChildDirection(child_side_to_check, dimension);
                        assert(child.adjacent_cells[toInt(child_to_neighbor_direction)].size() == 1 && "Child should only be connected to the one neighboring child on this side");
                        Link& link_to_neighbor = child.adjacent_cells[toInt(child_to_neighbor_direction)].front();
                        assert(link_to_neighbor.loan == 0.0 || iter > 0); // the loan has not been set yet
                        Link& loan_link = link_to_neighbor.getReverse();
                        loan_link.loan += value_transfer;
                    }
                }
            }
        }
    }

    // debug check:
    for (idx_t child_idx : parent.children)
    {
        if (child_idx < 0)
        {
            break;
        }
        const Cell& child = cell_data[child_idx];
        debugCheckLoans(child);
        assert(getValueError(child) > -allowed_volume_error);
    }
}

template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::transferLoans(Link& old, const std::list<Link*>& new_links)
{
    if (old.loan == 0.0)
    {
        return;
    }
    // TODO
    // this implements naive equal transfer
    int new_link_count = new_links.size();
    assert(new_link_count > 0);
    for (Link* link : new_links)
    {
        link->loan = old.loan / new_link_count;
    }
}



template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::settleLoans(Cell& from, float left_overs)
{
    if (left_overs < allowed_volume_error)
    { // nothing to distribute
        return;
    }
    std::vector<Link*> loaners;
    float total_loan = 0.0;
    for (std::list<Link>& neighbors_in_a_given_direction : from.adjacent_cells)
    {
        for (Link& link : neighbors_in_a_given_direction)
        {
            if (link.getReverse().loan > allowed_volume_error)
            {
                total_loan += link.getReverse().loan;
                loaners.push_back(&link.getReverse());
            }
        }
    }
    assert(total_loan > -allowed_volume_error && "a cell cannot be loaned a negative amount!");
    if (total_loan < allowed_volume_error)
    { // avoid division by zero
        // This cell had no error loaned
        return;
    }
    left_overs = std::min(left_overs, total_loan); // only settle loans in this function, don't introduce new loans to neighbor cells
    for (Link* loaner : loaners)
    {
        float pay_back = loaner->loan * left_overs / total_loan;
        loaner->loan -= pay_back;
    }
}

template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::settleLoans()
{
    std::vector<std::vector<Cell*>> depth_ordered = getDepthOrdered();

    for (std::vector<Cell*>& depth_nodes : depth_ordered)
    {
        for (Cell* cell : depth_nodes)
        {
            float left_overs = getValueError(*cell);
            settleLoans(*cell, left_overs);
        }
    }
}

template<typename CellGeometry>
float InfillFractal2D<CellGeometry>::getTotalActualizedVolume(const Cell& sub_tree_root)
{
    if (sub_tree_root.is_subdivided)
    {
        float ret = 0.0;
        for (idx_t child_idx : sub_tree_root.children)
        {
            if (child_idx < 0)
            {
                break;
            }
            ret += getTotalActualizedVolume(cell_data[child_idx]);
        }
        return ret;
    }
    else
    {
        return getActualizedVolume(sub_tree_root);
    }
}

/*
 * Error redistribution /\                         .
 * 
 * =======================================================
 * 
 * Output \/                  .
 */

/*
 * Output /\                         .
 * 
 * =======================================================
 * 
 * Debug \/                  .
 */

template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::debugCheckDepths() const
{
    int problems = 0;
    for (const Cell& cell : cell_data)
    {
        for (idx_t child_idx : cell.children)
        {
            if (child_idx < 0) break;
            if (cell_data[child_idx].depth != cell.depth + 1)
            {
                problems++;
                logError("Cell with depth %i has a child with depth %i!\n", cell.depth, cell_data[child_idx].depth);
            }
        }
    }
    assert(problems == 0 && "no depth difference problems");
}

template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::debugCheckVolumeStats() const
{
    int problems = 0;
    for (const Cell& cell : cell_data)
    {
        if (cell.volume <= 0)
        {
            problems++;
            logError("Cell with depth %i has incorrect volume %f!\n", cell.depth, cell.volume);
        }
        if (cell.filled_volume_allowance < 0)
        {
            problems++;
            logError("Cell with depth %i has incorrect filled_volume_allowance  %f!\n", cell.depth, cell.filled_volume_allowance );
        }
        if (cell.minimally_required_density < 0)
        {
            problems++;
            logError("Cell with depth %i has incorrect minimally_required_density %f!\n", cell.depth, cell.minimally_required_density);
        }
        float child_filled_volume_allowance = 0;
        for (idx_t child_idx : cell.children)
        {
            if (child_idx < 0) break;
            const Cell& child = cell_data[child_idx];
            child_filled_volume_allowance += child.filled_volume_allowance;
        }
        if (cell.filled_volume_allowance < child_filled_volume_allowance - 0.1)
        {
            problems++;
            logError("Cell with depth %i has a children with more volume!\n", cell.depth);
        }
    }
    assert(problems == 0 && "no depth difference problems");
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::debugCheckLoans(const Cell& cell) const
{
    for (const std::list<Link>& side_links : cell.adjacent_cells)
    {
        for (const Link& link : side_links)
        {
            assert(std::isfinite(link.loan));
            assert(std::isfinite(link.getReverse().loan));
            assert(link.loan < allowed_volume_error || link.getReverse().loan < allowed_volume_error); //  "two cells can't be loaning to each other!"
        }
    }
}

}; // namespace cura
