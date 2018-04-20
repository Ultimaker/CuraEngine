#ifndef INFILL_SQUARE_SUBDIVISION_H
#define INFILL_SQUARE_SUBDIVISION_H

#include "../utils/AABB.h"
#include "../utils/Range.h"
#include "../utils/SVG.h"
#include "../utils/string.h" // writing loan to svg
#include "InfillFractal.h"
#include "SubdivSquare.h"

namespace cura
{



class SquareSubdivision : public InfillFractal<AABB, 4, 2>
{
    using Parent = InfillFractal<AABB, 4, 2>;
    using LinkIterator = Parent::LinkIterator;
public:
    SquareSubdivision(const DensityProvider& density_provider, const AABB aabb, const int max_depth, coord_t line_width, bool consecutivity_constraint)
    : Parent(density_provider, aabb, max_depth, line_width, consecutivity_constraint)
    {
    }
protected:
    float getActualizedArea(const Parent::Cell& node) const
    {
        const AABB& aabb = node.elem;
        Point diagonal = aabb.max - aabb.min;
        return INT2MM(Parent::line_width) * INT2MM(diagonal.X + diagonal.Y);
    }
    
    float getChildrenActualizedArea(const Cell& cell) const
    {
        // The actualized area of squares doesn't depend on surrounding cells,=
        // so we just call getActualizedArea(.)
        float actualized_area = 0;
        for (Cell* child : cell.children)
        {
            if (!child)
            {
                continue;
            }
            actualized_area += getActualizedArea(*child);
        }
        return actualized_area;
    }
    
    enum class Direction : int
    {
        LEFT = 0, // ordered on polarity and dimension: first X from less to more, then Y
        RIGHT = 1,
        DOWN = 2,
        UP = 3,
        COUNT = 4
    };
    Direction opposite(Direction in)
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
    enum class ChildSide : int
    {
        LEFT_BOTTOM = 0,
        RIGHT_BOTTOM = 1,
        LEFT_TOP = 2,
        RIGHT_TOP = 3,
        COUNT = 4,
        FIRST = LEFT_BOTTOM
    };
    
    int opposite(int in)
    {
        return static_cast<int>(opposite(static_cast<Direction>(in)));
    }
    
    float getDensity(const Cell& cell) const
    {
        AABB3D aabb3d(Point3(cell.elem.min.X, cell.elem.min.Y, 0), Point3(cell.elem.max.X, cell.elem.max.Y, 1));
        return density_provider(aabb3d);
    }
    
    int getNumberOfSides() const
    {
        return 4;
    }
    
    Cell* createRoot() const
    {
        return new Cell(aabb, 0, getNumberOfSides());
    }
    
    void createTree(Cell& sub_tree_root, int max_depth)
    {
        int parent_depth = sub_tree_root.depth;
        if (parent_depth >= max_depth)
        {
            for (int child_idx = 0; child_idx < 4; child_idx++)
            {
                sub_tree_root.children[child_idx] = nullptr;
            }
            return;
        }
        
        const AABB& parent_aabb = sub_tree_root.elem;
        Point middle = parent_aabb.getMiddle();
        Cell* child_lb = new Cell(AABB(parent_aabb.min, middle), parent_depth + 1, getNumberOfSides());
        Cell* child_rb = new Cell(AABB(Point(middle.X, parent_aabb.min.Y), Point(parent_aabb.max.X, middle.Y)), parent_depth + 1, getNumberOfSides());
        Cell* child_lt = new Cell(AABB(Point(parent_aabb.min.X, middle.Y), Point(middle.X, parent_aabb.max.Y)), parent_depth + 1, getNumberOfSides());
        Cell* child_rt = new Cell(AABB(middle, parent_aabb.max), parent_depth + 1, getNumberOfSides());
        sub_tree_root.children[0] = child_lb; // ordered on polarity and dimension: first X from less to more, then Y
        sub_tree_root.children[1] = child_rb;
        sub_tree_root.children[2] = child_lt;
        sub_tree_root.children[3] = child_rt;
        
        for (Cell* child : sub_tree_root.children)
        {
            createTree(*child, max_depth);
        }
    }
    
    void initialConnection(Cell& before, Cell& after, Direction dir)
    {
        before.adjacent_cells[static_cast<size_t>(dir)].emplace_front(after);
        after.adjacent_cells[static_cast<size_t>(opposite(dir))].emplace_front(before);
        LinkIterator before_to_after = before.adjacent_cells[static_cast<size_t>(dir)].begin();
        LinkIterator after_to_before = after.adjacent_cells[static_cast<size_t>(opposite(dir))].begin();
        before_to_after->reverse = after_to_before;
        after_to_before->reverse = before_to_after;
    }
    
    /*!
     * 
     * \param dimension The dimension to check whether they are next to each other. 0 for X if you want to check if the cells are besides each other, 1 for on top of each other.
     */
    bool isNextTo(const Cell& a, const Cell& b, int dimension) const
    {
        AABB a_square_expanded = a.elem;
        a_square_expanded.expand(10); // TODO: move allowed error to central place?
        if (!a_square_expanded.hit(b.elem))
        {
            return false; // cells are apart:  []     []
        }
        Range<coord_t> a_range(getCoord(a.elem.min, !dimension), getCoord(a.elem.max, !dimension));
        Range<coord_t> b_range(getCoord(b.elem.min, !dimension), getCoord(b.elem.max, !dimension));
        // contract to prevent
        // []
        //   []   from counting as being next to each other
        return a_range.expand(-10).overlap(b_range); // TODO: move allowed error to central place?
    }
    
    /*!
     * Transfer the loans from an old link to the new links after subdivision
     */
    void transferLoans(Link& old, const std::list<Link*>& new_links)
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
    
    void subdivide(Cell& cell)
    {
        const float total_loan_balance_before = getTotalLoanBalance(cell);
        
        assert(cell.children[0] && cell.children[1] && cell.children[2] && cell.children[3] && "Children must be initialized for subdivision!");
        Cell& child_lb = *cell.children[0];
        Cell& child_rb = *cell.children[1];
        Cell& child_lt = *cell.children[2];
        Cell& child_rt = *cell.children[3];
        
        initialConnection(child_lt, child_rt, Direction::RIGHT);
        initialConnection(child_lb, child_rb, Direction::RIGHT);
        initialConnection(child_lb, child_lt, Direction::UP);
        initialConnection(child_rb, child_rt, Direction::UP);
        
        for (int side = 0; side < getNumberOfSides(); side++)
        {
            int edge_dim = !(side / 2);
            
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
                Cell& neighboring_cell = *neighbor.to;
                std::list<Link>& neighboring_edge_links = neighboring_cell.adjacent_cells[opposite(side)];
                
                std::list<Link*> new_incoming_links;
                std::list<Link*> new_outgoing_links;
                for (Cell* child : cell.children)
                {
                    assert(child);
                    assert(neighbor.to);
                    if (isNextTo(*child, *neighbor.to, !edge_dim))
                    {
                        child->adjacent_cells[side].emplace_front(*neighbor.to);
                        LinkIterator outlink = child->adjacent_cells[side].begin();
                        
                        neighboring_edge_links.emplace(*neighbor.reverse, *child);
                        LinkIterator inlink = *neighbor.reverse;
                        inlink--;
                        
                        outlink->reverse = inlink;
                        inlink->reverse = outlink;
                        
                        new_incoming_links.push_back(&*inlink);
                        new_outgoing_links.push_back(&*outlink);
                    }
                    
                }
                transferLoans(neighbor.getReverse(), new_incoming_links);
                transferLoans(neighbor, new_outgoing_links);
                neighboring_edge_links.erase(*neighbor.reverse);
            }
            cell.adjacent_cells[side].clear();
            
        }
        
        cell.is_subdivided = true;
        
        
        float total_loan_balance_after = 0.0;
        for (const Cell* child : cell.children)
        {
            assert(child && " we just subdivided!");
            total_loan_balance_after += getTotalLoanBalance(*child);
        }
        assert(std::abs(total_loan_balance_after - total_loan_balance_before) < 0.0001);
    }
    
public:
    
    void createDitheredPattern()
    {
        bool midway_decision_boundary = false;
        createMinimalErrorPattern(midway_decision_boundary);
        
        std::vector<ChildSide> tree_path(max_depth, ChildSide::COUNT);
        if (root)
        {
            dither(*root, tree_path);
        }
    }

    /*!
     * Get the right up diagonal neighbor
     * for which the left lower corner coincides with the right uper corner of this cell
     * if any, otherwise return nullptr
     *           __ __
     * :        |__|__|             but dont get this one  or this one
     * :________|▓▓|__|                           ^             ^
     * |       ↗|     |             | X X X X X |         |     |  X
     * | from   |_____|             |___________|         |_____|  X
     * |        |     |             |from |     |         |from |  X
     * |________|_____|             |_____|_____|         |_____|____
     */
    Link* getDiagonalNeighbor(Cell& cell, Direction left_right) const
    { // implementation naming assumes left_right is right
        std::list<Link>& right_side = cell.adjacent_cells[static_cast<size_t>(left_right)];
        if (!right_side.empty())
        {
            std::list<Link>& right_side_up_side = right_side.back().to->adjacent_cells[static_cast<size_t>(Direction::UP)];
            if (!right_side_up_side.empty())
            {
                Link& ru_diag_neighbor = right_side_up_side.front();
                std::list<Link>& up_side = cell.adjacent_cells[static_cast<size_t>(Direction::UP)];
                if (!up_side.empty())
                {
                    std::list<Link>& up_side_right_side = up_side.back().to->adjacent_cells[static_cast<size_t>(left_right)];
                    if (!up_side_right_side.empty())
                    {
                        Link& ur_diag_neighbor = up_side_right_side.front();
                        if (ru_diag_neighbor.to == ur_diag_neighbor.to)
                        {
                            return &ru_diag_neighbor;
                        }
                    }
                }
            }
        }
        return nullptr;
    }
    
    /*!
     * Check whetehr we can propagate error to the cell diagonally left up of this cell.
     * This is only possible of we hadn't already processed that cell,
     * which is the case if this is a ll cell after a lb cell after the last right cell.
     */
    bool canPropagateLU(Cell& cell, const std::vector<ChildSide>& tree_path)
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
    
    void dither(Cell& parent, std::vector<ChildSide>& tree_path)
    {
        if (parent.is_subdivided)
        {
            for (int path_dir = static_cast<int>(ChildSide::FIRST); path_dir < static_cast<int>(ChildSide::COUNT); path_dir++)
            {
                Cell* child = parent.children[path_dir];;
                if (child)
                {
                    tree_path[parent.depth] = static_cast<ChildSide>(path_dir);
                    dither(*child, tree_path);
                }
            }
        }
        else
        {
            const float balance = getBalance(parent);
            const float parent_actualized_area = getActualizedArea(parent);
            const float subdivided_actualized_area = getChildrenActualizedArea(parent);
            const float range = subdivided_actualized_area - parent_actualized_area;
//             const float decision_boundary = (rand() % 100) * range / 100;
//             const float decision_boundary = (.01 * (rand() % 101) * .5 + .25) * range;
            const float decision_boundary = range / 2;
            bool do_subdivide = balance > decision_boundary && canSubdivide(parent);
            float actualized_area = (do_subdivide)? subdivided_actualized_area : parent_actualized_area;
            
            const float left_over = balance - (actualized_area - parent_actualized_area); // might be negative
            
            // divide left_over equally per cell capacity, which is linearly related to the cell area
            float total_weighted_forward_cell_area = 0;
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
                for (Link& neighbor : side)
                {
                    assert(neighbor.to->area > 0 && "area must be computed");
                    total_weighted_forward_cell_area += direction_weights[side_idx] * neighbor.to->area;
                }
            }
            Link* diag_neighbor = getDiagonalNeighbor(parent, Direction::RIGHT);
            if (diag_neighbor)
            {
                total_weighted_forward_cell_area += diag_weight * diag_neighbor->to->area;
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
                total_weighted_forward_cell_area += backward_diag_weight * backward_diag_neighbor->to->area;
            }
            
            assert(total_weighted_forward_cell_area >= 0.0);
            assert(total_weighted_forward_cell_area < 100.0);
            for (int side_idx = 0; side_idx < 2; side_idx++)
            {
                Direction direction = forwards[side_idx];
                std::list<Link>& side = parent.adjacent_cells[static_cast<size_t>(direction)];
                for (Link& neighbor : side)
                {
                    Link& loan_link = (left_over > 0)? neighbor : neighbor.getReverse();
                    loan_link.loan += std::abs(left_over) * direction_weights[side_idx] * neighbor.to->area / total_weighted_forward_cell_area;
                }
            }
            if (diag_neighbor)
            {
                Link& diag_loan_link = (left_over > 0)? *diag_neighbor : diag_neighbor->getReverse();
                diag_loan_link.loan += std::abs(left_over) * diag_weight * diag_neighbor->to->area / total_weighted_forward_cell_area;
            }
            if (backward_diag_neighbor)
            {
                Link& diag_loan_link = (left_over > 0)? *backward_diag_neighbor : backward_diag_neighbor->getReverse();
                diag_loan_link.loan += std::abs(left_over) * backward_diag_weight * backward_diag_neighbor->to->area / total_weighted_forward_cell_area;
            }
            
            if (do_subdivide)
            {
                subdivide(parent);
            }
        }
    }
    
    void testSubdivision()
    {
        subdivide(*root);
        
        subdivide(*root->children[1]);
        subdivide(*root->children[1]->children[2]);
        subdivide(*root->children[3]);
        subdivide(*root->children[3]->children[0]);
        subdivide(*root->children[3]->children[0]->children[1]);
    }
    
    void debugCheck()
    {
        float total_length = getTotalLength();
        float error = (root->filled_area_allowance - total_length) / root->filled_area_allowance * 100;
        std::cerr << "Acquired " << total_length << " for " << root->filled_area_allowance << " requested. (" << error << "% error)\n";
    }
    
    float getTotalLength(const Cell& sub_tree_root) const
    {
        float total_length = 0.0;
        if (sub_tree_root.is_subdivided)
        {
            for (Cell* child : sub_tree_root.children)
            {
                total_length += getTotalLength(*child);
            }
            return total_length;
        }
        else
        {
            return getActualizedArea(sub_tree_root);
        }
    }

    float getTotalLength() const
    {
        if (root)
        {
            return getTotalLength(*root);
        }
        else return 0.0;
    }
    
    Polygon createHilbertLine()
    {
        std::vector<Cell*> pattern = createHilbertPattern();
        Polygon ret;
        for (Cell* cell : pattern)
        {
            ret.add(cell->elem.getMiddle());
        }
        return ret;
    }
    
    std::vector<Cell*> createHilbertPattern()
    {
        std::vector<Cell*> ret;
        
        if (!root)
        {
            return ret;
        }
        
        ret.reserve(std::pow(4, max_depth));
        
        createHilbertPattern(*root, ret, 0, 1);
        
        return ret;
    }
    
    
    void createHilbertPattern(Cell& sub_tree_root, std::vector<Cell*>& pattern, uint_fast8_t starting_child, int_fast8_t winding_dir)
    {
        if (sub_tree_root.is_subdivided)
        {
            /* Basic order of Hilbert curve for first iteration:
             * 1 2
             * 0 3
             */
            constexpr ChildSide child_nr_to_child_side[] { ChildSide::LEFT_BOTTOM, ChildSide::LEFT_TOP, ChildSide::RIGHT_TOP, ChildSide::RIGHT_BOTTOM };
            /*  child_winding_dirs         child_starting_child
             *     +---+   +---+              1---2   1---2
             *     |CW |   | CW|              |   |   |   |
             *     +   +···+   +             *0   3···0*  3
             *     :           :              :           :
             *     +---+   +---+              1---2   1---2*
             *      CCW|   |CCW                   |   |
             *     +---+   +---+             *0---3   0---3
             */
            constexpr int_fast8_t child_winding_dirs[] { -1, 1, 1, -1 };
            constexpr int_fast8_t child_starting_child[] { 0, 0, 0, 2 };
            for (uint_fast8_t child_idx_offset = 0; child_idx_offset < 4; child_idx_offset++)
            {
                uint8_t child_idx = (starting_child + winding_dir * child_idx_offset + 4 ) % 4;
                Cell* child = sub_tree_root.children[static_cast<int_fast8_t>(child_nr_to_child_side[child_idx])];
                assert(child);
                createHilbertPattern(*child, pattern, (starting_child + winding_dir * child_starting_child[child_idx_offset] + 4 ) % 4, winding_dir * child_winding_dirs[child_idx_offset]);
            }
        }
        else
        {
            pattern.push_back(&sub_tree_root);
        }
    }
    
    Polygon createMooreLine()
    {
        std::vector<Cell*> pattern = createMoorePattern();
        Polygon ret;
        for (Cell* cell : pattern)
        {
            ret.add(cell->elem.getMiddle());
        }
        return ret;
    }
    
    std::vector<Cell*> createMoorePattern()
    {
        std::vector<Cell*> ret;
        
        if (!root)
        {
            return ret;
        }
        
        ret.reserve(std::pow(4, max_depth));
        
        createMoorePattern(*root, ret, 0, 1);
        
        return ret;
    }
    
    void createMoorePattern(Cell& sub_tree_root, std::vector<Cell*>& pattern, uint_fast8_t starting_child, int_fast8_t winding_dir)
    {
        if (sub_tree_root.is_subdivided)
        {
            /* Basic order of Moore curve for first iteration:
             * 1 2
             * 0 3
             */
            constexpr ChildSide child_nr_to_child_side[] { ChildSide::LEFT_BOTTOM, ChildSide::LEFT_TOP, ChildSide::RIGHT_TOP, ChildSide::RIGHT_BOTTOM };
            /*  child_winding_dirs  and child_starting_child
             *     1---2···1---2
             *     |CW     * CW|
             *     0---3*  0---3
             *         :   :
             *     1---2  *1---2
             *     | CW      CW|
             *     0---3*  0---3
             */
            constexpr int_fast8_t child_winding_dirs[] { 1, 1, 1, 1 };
            constexpr int_fast8_t child_starting_child[] { 3, 3, 1, 1 };
            for (uint_fast8_t child_idx_offset = 0; child_idx_offset < 4; child_idx_offset++)
            {
                uint8_t child_idx = (starting_child + winding_dir * child_idx_offset + 4 ) % 4;
                Cell* child = sub_tree_root.children[static_cast<int_fast8_t>(child_nr_to_child_side[child_idx])];
                assert(child);
                createHilbertPattern(*child, pattern, (starting_child + winding_dir * child_starting_child[child_idx_offset] + 4 ) % 4, winding_dir * child_winding_dirs[child_idx_offset]);
            }
        }
        else
        {
            pattern.push_back(&sub_tree_root);
        }
    }
    
    void debugOutput(SVG& svg, Cell& sub_tree_root, float drawing_line_width, bool draw_arrows)
    {
        if (sub_tree_root.is_subdivided)
        {
            for (Cell* child : sub_tree_root.children)
            {
                assert(child);
                debugOutput(svg, *child, drawing_line_width, draw_arrows);
            }
        }
        else
        {
            AABB square = sub_tree_root.elem;
            svg.writeLine(square.min, Point(square.max.X, square.min.Y), SVG::Color::BLACK, drawing_line_width);
            svg.writeLine(square.max, Point(square.max.X, square.min.Y), SVG::Color::BLACK, drawing_line_width);
            
            const Point from_middle = sub_tree_root.elem.getMiddle();
            
            // draw links
            if (draw_arrows)
            {
                for (std::list<Link>& side : sub_tree_root.adjacent_cells)
                {
                    for (Link& link : side)
                    {
                        const Point to_middle = link.to->elem.getMiddle();
                        const Point link_vector = to_middle - from_middle;
                        Point arrow_from = from_middle + link_vector / 5 * 1 - turn90CCW(link_vector / 20);
                        Point arrow_to = from_middle + link_vector / 5 * 4 - turn90CCW(link_vector / 20);
                        
                        Point arrow_head_back_l = arrow_to - link_vector / 15 + turn90CCW(link_vector / 30);
                        Point arrow_head_back_r = arrow_to - link_vector / 15 - turn90CCW(link_vector / 30);
                        svg.writeLine(arrow_from, arrow_to, SVG::Color::BLUE);
                        svg.writeLine(arrow_to, arrow_head_back_l, SVG::Color::BLUE);
                        svg.writeLine(arrow_to, arrow_head_back_r, SVG::Color::BLUE);
                        svg.writeLine(arrow_head_back_l, arrow_head_back_r, SVG::Color::BLUE);
                        
                        std::ostringstream os;
                        os << PrecisionedDouble{ 2, link.loan};
                        svg.writeText((arrow_from + arrow_to) / 2 + turn90CCW(link_vector / 10) + link_vector / 10, os.str(), SVG::Color::BLACK, 4);
                    }
                }
            }
        }
    }
    
    
    void outputHilbert(SVG& svg, float drawing_line_width)
    {
        Polygon hilbert = createHilbertLine();
        svg.writeLines(*hilbert, SVG::Color::BLACK, drawing_line_width);
    }
    
    void outputMoore(SVG& svg, float drawing_line_width)
    {
        Polygon moore = createMooreLine();
        svg.writePolygon(moore, SVG::Color::BLACK, drawing_line_width);
    }
    
    void debugOutput(SVG& svg, float drawing_line_width, bool draw_arrows)
    {
        if (root)
        {
            svg.writePolygon(root->elem.toPolygon(), SVG::Color::BLACK, drawing_line_width);
            debugOutput(svg, *root, drawing_line_width, draw_arrows);
        }
    }
};

}
#endif //INFILL_SQUARE_SUBDIVISION_H
