#ifndef INFILL_INFILL_FRACTAL_H
#define INFILL_INFILL_FRACTAL_H

#include <list>
#include <unordered_set>
#include <cassert>

#include "../utils/IntPoint.h"

#include "../utils/FingerTree.h"

namespace cura
{

/*!
 * Generic class which encompasses subdivision fractals and space filling curves used for infill generation.
 */
template<class Elem, unsigned char subdivision_count, unsigned char dimensionality>
class InfillFractal
{
public:
    struct AreaStatistics
    {
        float area; //!< The area of the triangle or square in mm^2
        float filled_area_allowance; //!< The area to be filled corresponding to the average density requested by the volumetric density specification.
    };
    struct Cell; // forward decl
    struct Link
    {
        Cell* to;
        Link* reverse; //!< The link in the inverse direction
        float loan; //!< amount of requested_filled_area loaned from one cell to another, when subdivision of the former is prevented by the latter. This value should always be positive.
        Cell* from()
        {
            return reverse->to;
        }
        Link(Cell* to, Link* reverse)
        : to(to)
        , reverse(reverse)
        , loan(0.0)
        {}
    };
    
    struct Cell
    {
        Elem elem;
        typename FingerTree<AreaStatistics, subdivision_count>::Node* area_statistics;
        std::vector<std::list<Link>> adjacent_cells; //!< the adjecent cells for each edge/face of this cell
        
        Cell(const Elem& elem, unsigned char number_of_sides)
        : elem(elem)
        , adjacent_cells(number_of_sides)
        {
        }
    };
    
    
    
    coord_t line_width; //!< The line width of the fill lines
    
    
    Cell* origin;
    
    FingerTree<AreaStatistics, subdivision_count> statistics_tree;
    
    InfillFractal()
    : origin(nullptr)
    {
//         createStatsticsTree();
    }

    virtual ~InfillFractal()
    {
        if (!origin)
        {
            return;
        }
        std::unordered_set<Cell*> all_cells;
        std::list<Cell*> to_be_handled;
        to_be_handled.push_front(origin);

        while (!to_be_handled.empty())
        {
            Cell* handling = to_be_handled.front();
            to_be_handled.pop_front();
            if (all_cells.find(handling) != all_cells.end())
            {
                all_cells.emplace(handling);
                for (std::list<Link>& neighbors : handling->adjacent_cells)
                {
                    for (Link& link : neighbors)
                    {
                        to_be_handled.push_back(link.to);
                    }
                }
            }
        }
        
        for (Cell* cell : all_cells)
        {
            delete cell;
        }
    }

protected:
//     virtual void createStatsticsTree() = 0;
    
    virtual void subdivide(Cell* cell) = 0;
    
    virtual float getActualizedArea(const Cell& cell) = 0;
    
//     virtual bool isConstrainedBy(const Cell& constrainee, const Cell& constrainer) = 0;

    float getBalance(const Cell& cell)
    {
        float balance = cell.area_statistics->filled_area_allowance - getActualizedArea(cell);
        for (std::list<Link>& neighbors_in_a_given_direction : cell.adjacent_cells)
        {
            for (Link& link : neighbors_in_a_given_direction)
            {
                balance -= link.loan;
                balance += link.reverse->loan;
            }
        }
        return balance;
    }

    void distributeLeftOvers(Cell& from, float left_overs)
    {
//         from.loan_balance -= left_overs;
        std::vector<Link*> loaners;
        float total_loan = 0.0;
        for (std::list<Link>& neighbors_in_a_given_direction : from->adjacent_cells)
        {
            for (Link& link : neighbors_in_a_given_direction)
            {
                if (link.reverse->loan > 0.00001)
                {
                    total_loan += link.reverse->loan;
                    loaners.push_back(link.reverse);
                }
            }
        }
        assert(left_overs < total_loan * 1.00001 && "There cannot be more left over than what was initially loaned");
        for (Link* loaner : loaners)
        {
            float pay_back = loaner->loan * left_overs / total_loan;
            loaner->loan -= pay_back;
//             loaner->from()->loan_balance += pay_back;
        }
    }
};

}
#endif //INFILL_INFILL_FRACTAL_H
