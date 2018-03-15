#ifndef INFILL_SQUARE_SUBDIVISION_H
#define INFILL_SQUARE_SUBDIVISION_H

#include "../utils/AABB.h"
#include "../utils/Range.h"
#include "../utils/SVG.h"
#include "InfillFractal.h"
#include "SubdivSquare.h"

namespace cura
{



class SquareSubdivision : public InfillFractal<AABB, 4, 2>
{
    using Parent = InfillFractal<AABB, 4, 2>;
    using LinkIterator = Parent::LinkIterator;
public:
    SquareSubdivision(const DensityProvider& density_provider, const AABB aabb, coord_t line_width)
    : Parent(density_provider, aabb, line_width)
    {
    }
protected:
    float getActualizedArea(const Parent::Cell& node)
    {
        const AABB& aabb = node.elem;
        Point diagonal = aabb.max - aabb.min;
        return INT2MM(Parent::line_width) * INT2MM(diagonal.X + diagonal.Y);
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
    int opposite(int in)
    {
        return static_cast<int>(opposite(static_cast<Direction>(in)));
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
        Cell* child_lb = new Cell(AABB(parent_aabb.min, middle), parent_depth + 1, number_of_sides);
        Cell* child_rb = new Cell(AABB(Point(middle.X, parent_aabb.min.Y), Point(parent_aabb.max.X, middle.Y)), parent_depth + 1, number_of_sides);
        Cell* child_lt = new Cell(AABB(Point(parent_aabb.min.X, middle.Y), Point(middle.X, parent_aabb.max.Y)), parent_depth + 1, number_of_sides);
        Cell* child_rt = new Cell(AABB(middle, parent_aabb.max), parent_depth + 1, number_of_sides);
        sub_tree_root.children[0] = child_lb; // ordered on polarity and dimension: first X from less to more, then Y
        sub_tree_root.children[1] = child_rb;
        sub_tree_root.children[2] = child_lt;
        sub_tree_root.children[3] = child_rt;
        
        for (Cell* child : sub_tree_root.children)
        {
            createTree(*child, max_depth);
        }
    }
    
    void getSpecificationAllowance(Cell& sub_tree_root)
    {
        Point area_dimensions = sub_tree_root.elem.max - sub_tree_root.elem.min;
        sub_tree_root.area = INT2MM(area_dimensions.X) * INT2MM(area_dimensions.Y);
        
        bool has_children = sub_tree_root.children[0] != nullptr;
        if (has_children)
        {
            for (Cell* child : sub_tree_root.children)
            {
                getSpecificationAllowance(*child);
                sub_tree_root.filled_area_allowance += child->filled_area_allowance;
            }
        }
        else
        {
            float requested_density = density_provider(sub_tree_root.elem);
            sub_tree_root.filled_area_allowance = sub_tree_root.area * requested_density;
        }
    }
    
    virtual void createTree()
    {
        int max_depth = 8; // TODO
        
        root = new Cell(aabb, 0, number_of_sides);
        
        createTree(*root, max_depth);
        
        getSpecificationAllowance(*root);
    }
    
    
    static constexpr char number_of_sides = 4;
    
    
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
    bool isNextTo(const Cell& a, const Cell& b, int dimension)
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
        assert(cell.children[0] && cell.children[1] && cell.children[2] && cell.children[3] && "Children must be initialized for subdivision!");
        Cell& child_lb = *cell.children[0];
        Cell& child_rb = *cell.children[1];
        Cell& child_lt = *cell.children[2];
        Cell& child_rt = *cell.children[3];
        
        initialConnection(child_lt, child_rt, Direction::RIGHT);
        initialConnection(child_lb, child_rb, Direction::RIGHT);
        initialConnection(child_lt, child_lb, Direction::UP);
        initialConnection(child_rt, child_rb, Direction::UP);
        
        for (int side = 0; side < number_of_sides; side++)
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
                transferLoans(**neighbor.reverse, new_incoming_links);
                transferLoans(neighbor, new_outgoing_links);
                neighboring_edge_links.erase(*neighbor.reverse);
            }
            cell.adjacent_cells[side].clear();
            
        }
        
        cell.is_subdivided = true;
    }
    
public:
    
    void testSubdivision()
    {
        subdivide(*root);
        
        subdivide(*root->children[1]);
        subdivide(*root->children[1]->children[2]);
    }
    
    void debugOutput(SVG& svg, Cell& sub_tree_root, int drawing_line_width)
    {
        if (sub_tree_root.is_subdivided)
        {
            for (Cell* child : sub_tree_root.children)
            {
                assert(child);
                debugOutput(svg, *child, drawing_line_width);
            }
        }
        else
        {
            svg.writePolygon(sub_tree_root.elem.toPolygon(), SVG::Color::BLACK, drawing_line_width);
            
            const Point from_middle = sub_tree_root.elem.getMiddle();
            
            // draw links
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
                }
            }
        }
    }
    
    void debugOutput(SVG& svg, int drawing_line_width)
    {
        if (root)
        {
            debugOutput(svg, *root, drawing_line_width);
        }
    }
};

}
#endif //INFILL_SQUARE_SUBDIVISION_H
