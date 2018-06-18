/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */

#include <cassert>
#include <sstream>  // debug TODO

#include "../utils/math.h"
#include "../utils/linearAlg2D.h"
#include "../utils/gettime.h"


namespace cura {


template<typename CellGeometry>
typename InfillFractal2D<CellGeometry>::Direction InfillFractal2D<CellGeometry>::opposite(InfillFractal2D<CellGeometry>::Direction in)
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
uint_fast8_t InfillFractal2D<CellGeometry>::opposite(uint_fast8_t in)
{
    return static_cast<uint_fast8_t>(opposite(static_cast<Direction>(in)));
}

template<typename CellGeometry>
uint_fast8_t InfillFractal2D<CellGeometry>::Cell::getChildCount() const
{
    return (children[2] < 0)? 2 : 4;
}

template<typename CellGeometry>
InfillFractal2D<CellGeometry>::InfillFractal2D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width)
: aabb(aabb)
, max_depth(max_depth)
, line_width(line_width)
, min_dist_to_cell_bound(line_width / 2)
, min_dist_to_cell_bound_diag(line_width * 0.5 * sqrt2)
, density_provider(density_provider)
{
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::initialize()
{
    TimeKeeper tk;
    createTree();
    debugCheckDepths();
    debugCheckVolumeStats();
    logError("Created InfillFractal2D tree with %i nodes and max depth %i in %5.2fs.\n", cell_data.size(), max_depth, tk.restart());
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
    // always subdivide the root, which is a bogus node!
    Cell& root = cell_data[0];
    for (idx_t child_idx : root.children)
    {
        if (child_idx >= 0 && shouldBeSubdivided(cell_data[child_idx]))
        {
            all_to_be_subdivided.push_back(child_idx);
        }
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
            constexpr bool redistribute_errors = true;
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
    bool midway_decision_boundary = false;
    createMinimalErrorPattern(midway_decision_boundary);

    std::vector<ChildSide> tree_path(max_depth, ChildSide::COUNT);
    dither(cell_data[0], tree_path);
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
        change |= subdivideAll();

        change |= bubbleUpConstraintErrors();

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
    depth_ordered.resize(max_depth);
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
        assert(sub_tree_root.depth > 0); // note: root must be subidivided
        assert(static_cast<size_t>(sub_tree_root.depth) < output.size());
        output[sub_tree_root.depth].push_back(&sub_tree_root);
    }
}

template<typename CellGeometry>
bool InfillFractal2D<CellGeometry>::subdivideAll()
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
bool InfillFractal2D<CellGeometry>::bubbleUpConstraintErrors()
{
    std::vector<std::vector<Cell*>> depth_ordered = getDepthOrdered();
    
    bool redistributed_anything = false;
    
    for (int depth = max_depth; depth >= 0; depth--)
    {
        std::vector<Cell*>& depth_nodes = depth_ordered[depth];
        for (Cell* cell : depth_nodes)
        {
            float unresolvable_error = getValueError(*cell);
            // pay back loaners proportional to the original loan
            if (unresolvable_error > allowed_volume_error)
            {
                const float total_obtained_loan = getTotalLoanObtained(*cell);
                const float rate_of_return = unresolvable_error / total_obtained_loan;
                const float ratio_loan_remaining = 1.0 - rate_of_return;

                for (const std::list<Link>& neighbors_in_a_given_direction : cell->adjacent_cells)
                {
                    for (const Link& link : neighbors_in_a_given_direction)
                    {
                        float& loan_here = link.getReverse().loan;
                        loan_here *= ratio_loan_remaining;
                        assert(std::abs(loan_here) < allowed_volume_error || std::abs(link.loan) < allowed_volume_error);
                    }
                }

                redistributed_anything = true;
            }
        }
    }
    return redistributed_anything;
}


template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::dither(Cell& parent, std::vector<ChildSide>& tree_path)
{
    if (parent.is_subdivided)
    {
        for (int path_dir = static_cast<int>(ChildSide::FIRST); path_dir < static_cast<int>(ChildSide::COUNT); path_dir++)
        {
            idx_t child_idx = parent.children[path_dir];
            if (child_idx > 0)
            {
                Cell& child = cell_data[child_idx];
                tree_path[parent.depth] = static_cast<ChildSide>(path_dir);
                dither(child, tree_path);
            }
        }
    }
    else
    {
        const float balance = getValueError(parent);
        const float parent_actualized_volume = getActualizedVolume(parent);
        const float subdivided_actualized_volume = getChildrenActualizedVolume(parent);
        const float range = subdivided_actualized_volume - parent_actualized_volume;
//         const float decision_boundary = (rand() % 100) * range / 100;
//         const float decision_boundary = (.01 * (rand() % 101) * .5 + .25) * range;
        const float decision_boundary = range / 2;
        bool do_subdivide = balance > decision_boundary && canSubdivide(parent);
        float actualized_volume = (do_subdivide)? subdivided_actualized_volume : parent_actualized_volume;

        const float left_over = balance - (actualized_volume - parent_actualized_volume); // might be negative

        // divide left_over equally per cell capacity, which is linearly related to the cell volume
        float total_weighted_forward_cell_volume = 0;
        Direction forwards[] = { Direction::RIGHT, Direction::UP };

        /* Floyd Steinberg weights:
         * 3 5 1
         *   * 7
         */

        float direction_weights[] = { 7, 5 }; // TODO: determine reasoned weights!
        float diag_weight = 0;//1;
        float backward_diag_weight = 3; //3; // TODO: error is still being propagated to already processed cells

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
        if (!canPropagateLU(parent, tree_path))
        {
            backward_diag_neighbor = nullptr;
        }
        if (backward_diag_neighbor)
        {
            /*!
             * TODO avoid error being propagated to already processed cells!!:
             *  ___ ___ ___ ___
             * | 3 | 4 | 7 | 8 |
             * |___|__↖|___|___|
             * | 1 | 2 |↖5 | 6 |
             * |___|___|___|___|
             */
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
            diag_loan_link.loan += std::abs(left_over) * diag_weight * cell_data[diag_neighbor->to_index].volume / total_weighted_forward_cell_volume;
        }
        if (backward_diag_neighbor)
        {
            Link& diag_loan_link = (left_over > 0)? *backward_diag_neighbor : backward_diag_neighbor->getReverse();
            diag_loan_link.loan += std::abs(left_over) * backward_diag_weight * cell_data[backward_diag_neighbor->to_index].volume / total_weighted_forward_cell_volume;
        }

        if (do_subdivide)
        {
            constexpr bool redistribute_errors = false;
            subdivide(parent, redistribute_errors);
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
    const float total_loan_error_balance_before = getTotalLoanError(cell);

    assert(cell.children[0] >= 0 && cell.children[1] >= 0 && "Children must be initialized for subdivision!");
    Cell& child_lb = cell_data[cell.children[0]];
    Cell& child_rb = cell_data[cell.children[1]];
    initialConnection(child_lb, child_rb, Direction::RIGHT);

//  TODO: do we even need this here?!
//     We do bubbling up of constraint erros already in a separate function!
//     if (redistribute_errors)
//     { // move left-over errors
//         logError("Not implemented yet!\n"); // TODO: move leftover errors back to neighboring cells
//     }

    if (cell.getChildCount() == 4)
    {
        Cell& child_lt = cell_data[cell.children[2]];
        Cell& child_rt = cell_data[cell.children[3]];
        initialConnection(child_lt, child_rt, Direction::RIGHT);
        initialConnection(child_lb, child_lt, Direction::UP);
        initialConnection(child_rb, child_rt, Direction::UP);
    }



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
            if (redistribute_errors)
            {
                transferLoans(neighbor.getReverse(), new_incoming_links);
                transferLoans(neighbor, new_outgoing_links);
            }
            neighboring_edge_links.erase(*neighbor.reverse);
        }
        
        cell.adjacent_cells[side].clear();
        
    }
    
    cell.is_subdivided = true;
    
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
            const std::list<Link>& up_side = cell.adjacent_cells[static_cast<size_t>(Direction::UP)];
            if (!up_side.empty())
            {
                const std::list<Link>& up_side_right_side = cell_data[up_side.back().to_index].adjacent_cells[static_cast<size_t>(left_right)];
                if (!up_side_right_side.empty())
                {
                    const Link& ur_diag_neighbor = up_side_right_side.front();
                    if (ru_diag_neighbor.to_index == ur_diag_neighbor.to_index)
                    {
                        return const_cast<Link*>(&ru_diag_neighbor);
                    }
                }
            }
        }
    }
    return nullptr;
}


template<typename CellGeometry>
bool InfillFractal2D<CellGeometry>::canPropagateLU(Cell& cell, const std::vector<ChildSide>& tree_path)
{
    if (cell.depth == 0)
    {
        return false;
    }
    if (tree_path[cell.depth - 1] == ChildSide::LEFT_BOTTOM)
    {
        return false;
    }
    if (tree_path[cell.depth - 1] != ChildSide::LEFT_TOP)
    {
        return true;
    }
    int tree_path_idx;
    for (tree_path_idx = cell.depth - 1; tree_path_idx >= 0; tree_path_idx--)
    {
        if (tree_path[tree_path_idx] == ChildSide::RIGHT_BOTTOM || tree_path[tree_path_idx] == ChildSide::RIGHT_TOP)
        {
            break;
        }
    }
    for (; tree_path_idx < cell.depth; tree_path_idx++)
    {
        if (tree_path[tree_path_idx] == ChildSide::LEFT_BOTTOM)
        {
            return false;
        }
    }
    return true;
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
            loan -= link.loan;
            loan += link.getReverse().loan;
        }
    }
    return loan;
}

template<typename CellGeometry>
float InfillFractal2D<CellGeometry>::getTotalLoanObtained(const Cell& cell) const
{
    float loan = 0.0;

    for (const std::list<Link>& neighbors_in_a_given_direction : cell.adjacent_cells)
    {
        for (const Link& link : neighbors_in_a_given_direction)
        {
            const float loan_here = link.getReverse().loan;
            loan += loan_here;
            assert(std::abs(loan_here) < allowed_volume_error || std::abs(link.loan) < allowed_volume_error);
        }
    }
    return loan;
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
    for (Link* link : new_links)
    {
        link->loan = old.loan / new_link_count;
    }
}



template<typename CellGeometry>
void InfillFractal2D<CellGeometry>::distributeLeftOvers(Cell& from, float left_overs)
{
//     from.loan_balance -= left_overs;
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
    assert(left_overs < total_loan * 1.00001 && "There cannot be more left over than what was initially loaned");
    for (Link* loaner : loaners)
    {
        float pay_back = loaner->loan * left_overs / total_loan;
        loaner->loan -= pay_back;
//         loaner->from()->loan_balance += pay_back;
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

}; // namespace cura
