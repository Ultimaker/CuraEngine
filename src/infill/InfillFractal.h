#ifndef INFILL_INFILL_FRACTAL_H
#define INFILL_INFILL_FRACTAL_H

#include <list>
#include <unordered_set>
#include <cassert>

#include "../utils/optional.h"
#include "../utils/IntPoint.h"
#include "DensityProvider.h"

namespace cura
{

/*!
 * Generic class which encompasses subdivision fractals and space filling curves used for infill generation.
 * 
 * The vocabulary of money is adopted to make explanations easier.
 * Rather than talking about the error between the requested density by the density provider and the actualized density by the pattern,
 * we talk about the liquid assets which have yet to be solidified.
 * 
 * \tparam Elem The element type to store, e.g. squares or triangles.
 * \tparam subdivision_count The amount of smaller shapes an element subdivides into
 * \tparam dimensionality The amount of directions in which an element is subdivided. A space filing curve is 1-dimensional.
 */
template<class Elem, unsigned char subdivision_count, unsigned char dimensionality>
class InfillFractal
{
public:
    struct Cell; // forward decl
    struct Link;
    using LinkIterator = typename std::list<Link>::iterator;
    struct Link
    {
        Cell* to;
        std::optional<LinkIterator> reverse; //!< The link in the inverse direction
        float loan; //!< amount of requested_filled_area loaned from one cell to another, when subdivision of the former is prevented by the latter. This value should always be positive.
        Cell* from()
        {
            assert(reverse);
            return (*reverse)->to;
        }
        Link& getReverse() const
        {
            assert(reverse);
            return **reverse;
        }
        Link(Cell& to)
        : to(&to)
        , loan(0.0)
        {}
    };
    
    struct Cell
    {
        Elem elem;
        int depth;
        float area; //!< The area of the triangle or square in mm^2
        float filled_area_allowance; //!< The area to be filled corresponding to the average density requested by the volumetric density specification.
        bool is_subdivided;
        std::vector<std::list<Link>> adjacent_cells; //!< the adjecent cells for each edge/face of this cell
        
        std::array<Cell*, subdivision_count> children;
        
        Cell(const Elem& elem, const int depth, unsigned char number_of_sides)
        : elem(elem)
        , depth(depth)
        , area(-1)
        , filled_area_allowance(0)
        , is_subdivided(false)
        , adjacent_cells(number_of_sides)
        {
        }
        
        ~Cell()
        {
            for (Cell* child : children)
            {
                if (child)
                {
                    delete child;
                }
            }
        }
    };
    
    
    
    coord_t line_width; //!< The line width of the fill lines
    
    const DensityProvider& density_provider; //!< function which determines the requested infill density of a triangle defined by two consecutive edges.
    
    Cell* root;
    
    AABB aabb;
    
    InfillFractal(const DensityProvider& density_provider, const AABB aabb, coord_t line_width)
    : line_width(line_width)
    , density_provider(density_provider)
    , root(nullptr)
    , aabb(aabb)
    {
    }

    virtual ~InfillFractal()
    {
        if (!root)
        {
            return;
        }
        delete root;
    }
    
    void initialize()
    {
        createTree();
    }

    /*!
     * Create a pattern with no dithering and no balancing.
     * 
     * \param middle_decision_boundary Whether to divide when required area is more than midway between the actualized area and the children actualized area. Set to false to create the minimal required subdivision before dithering
     */
    void createMinimalErrorPattern(bool middle_decision_boundary = true)
    {
        std::list<Cell*> to_be_checked;
        to_be_checked.push_back(root);
        
        while (!to_be_checked.empty())
        {
            Cell* checking = to_be_checked.front();
            to_be_checked.pop_front();
            
            if (!checking->children[0])
            { // cell has no children
                continue;
            }
            
            bool is_constrained = false;
            for (std::list<Link>& side : checking->adjacent_cells)
            {
                for (Link& neighbor : side)
                {
                    if (isConstrainedBy(*checking, *neighbor.to))
                    {
                        is_constrained = true;
                        break;
                    }
                }
            }
            float decision_boundary = (middle_decision_boundary)?
                                        (getActualizedArea(*checking) + getChildrenActualizedArea(*checking)) / 2
                                        : getChildrenActualizedArea(*checking);
            if (!is_constrained && checking->filled_area_allowance > decision_boundary)
            {
                subdivide(*checking);
                for (Cell* child : checking->children)
                {
                    to_be_checked.push_back(child);
                }
            }
        }
    }
    
    virtual void createDitheredPattern() = 0;

protected:
    virtual void createTree() = 0;
    
    virtual void subdivide(Cell& cell) = 0;
    
    virtual float getActualizedArea(const Cell& cell) = 0;

    virtual float getChildrenActualizedArea(const Cell& cell) = 0;
    
    virtual bool isConstrainedBy(const Cell& constrainee, const Cell& constrainer) = 0;

    float getBalance(const Cell& cell)
    {
        float balance = cell.filled_area_allowance - getActualizedArea(cell) + getTotalLoanBalance(cell);
        return balance;
    }
    
    float getTotalLoanBalance(const Cell& cell)
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

    void distributeLeftOvers(Cell& from, float left_overs)
    {
//         from.loan_balance -= left_overs;
        std::vector<Link*> loaners;
        float total_loan = 0.0;
        for (std::list<Link>& neighbors_in_a_given_direction : from->adjacent_cells)
        {
            for (Link& link : neighbors_in_a_given_direction)
            {
                if (link.getReverse().loan > 0.00001)
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
//             loaner->from()->loan_balance += pay_back;
        }
    }
};

}
#endif //INFILL_INFILL_FRACTAL_H
