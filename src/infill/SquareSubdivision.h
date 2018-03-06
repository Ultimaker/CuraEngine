#ifndef INFILL_SQUARE_SUBDIVISION_H
#define INFILL_SQUARE_SUBDIVISION_H

#include "../utils/AABB.h"
#include "../utils/SVG.h"
#include "InfillFractal.h"
#include "SubdivSquare.h"

namespace cura
{



class SquareSubdivision : public InfillFractal<AABB, 4, 2>
{
    using Parent = InfillFractal<AABB, 4, 2>;
protected:
    float getActualizedArea(const Parent::Cell& node)
    {
        const AABB& aabb = node.elem;
        Point diagonal = aabb.max - aabb.min;
        return INT2MM(Parent::line_width) * INT2MM(diagonal.X + diagonal.Y);
    }
    
    enum class Direction : int
    {
        LEFT = 0,
        RIGHT = 1,
        DOWN = 2,
        UP = 3,
        COUNT = 4
    };
    
    static constexpr char number_of_sides = 4;
    
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
    
    void initialConnection(Cell* before, Cell* after, Direction dir)
    {
        before->adjacent_cells[static_cast<size_t>(dir)].emplace_front(after, nullptr);
        after->adjacent_cells[static_cast<size_t>(opposite(dir))].emplace_front(before, nullptr);
        Link& before_to_after = before->adjacent_cells[static_cast<size_t>(dir)].front();
        Link& after_to_before = after->adjacent_cells[static_cast<size_t>(opposite(dir))].front();
        before_to_after.reverse = &after_to_before;
        after_to_before.reverse = &before_to_after;
    }
    
    void subdivide(Cell* cell)
    {
        if (!cell)
        {
            assert(false && "Subdivide called on nullptr!");
            return;
        }
        const AABB& parent_aabb = cell->elem;
        Point middle = cell->elem.getMiddle();
        Cell* child_lb = new Cell(AABB(parent_aabb.min, middle), number_of_sides);
        Cell* child_rb = new Cell(AABB(Point(middle.X, parent_aabb.min.Y), Point(parent_aabb.max.X, middle.Y)), number_of_sides);
        Cell* child_rt = new Cell(AABB(middle, parent_aabb.max), number_of_sides);
        Cell* child_lt = new Cell(AABB(Point(parent_aabb.min.X, middle.Y), Point(middle.X, parent_aabb.max.Y)), number_of_sides);
        
        initialConnection(child_lt, child_rt, Direction::RIGHT);
        initialConnection(child_lb, child_rb, Direction::RIGHT);
        initialConnection(child_lt, child_lb, Direction::UP);
        initialConnection(child_rt, child_rb, Direction::UP);
        
        for (int side = 0; side < number_of_sides; side++)
        {
            std::list<Link>& neighbors = cell->adjacent_cells[side];
        }
    }
    
    
    
    void debugOutput(SVG& svg)
    {
        
    }
};

}
#endif //INFILL_SQUARE_SUBDIVISION_H
