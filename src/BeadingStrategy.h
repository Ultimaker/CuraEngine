//Copyright (c) 2019 Ultimaker B.V.


#ifndef BEADING_STRATEGY_H
#define BEADING_STRATEGY_H

#include <utility>

#include "utils/IntPoint.h"

namespace arachne
{

/*!
 * Pure virtual base class template.
 * 
 * Strategy for covering a given (constant) horizontal model thickness with a number of beads.
 * 
 * The beads may have different widths.
 * 
 * TODO:
 * extend with printing order?
 */
class BeadingStrategy
{
public:
    /*!
     * The beading for a given horizontal model thickness.
     */
    struct Beading
    {
        std::vector<coord_t> bead_widths; //! The line width of each bead from the outer inset inward
        std::vector<coord_t> toolpath_locations; //! The distance of the toolpath location of each bead from the outline
        coord_t left_over; //! The distance not covered by any bead; gap area.
    };

    /*!
     * Retrieve the bead widths with which to cover a given thickness.
     * 
     * Requirement: Given a constant \p bead_count the output of each bead width must change gradually along with the \p thickness.
     * 
     * \note The \p bead_count might be different from the \ref BeadingStrategy::optimal_bead_count
     */
    virtual Beading compute(coord_t thickness, coord_t bead_count) const = 0;

    /*!
     * The ideal thickness for a given 
     */
    virtual coord_t optimal_thickness(coord_t bead_count) const = 0;

    /*!
     * The model thickness at which \ref BeadingStrategy::optimal_bead_count transitions from \p lower_bead_count to \p lower_bead_count + 1
     */
    virtual coord_t transition_thickness(coord_t lower_bead_count) const = 0;

    /*!
     * The number of beads should we ideally usefor a given model thickness
     */
    virtual coord_t optimal_bead_count(coord_t thickness) const = 0;
};




} // namespace arachne
#endif // BEADING_STRATEGY_H
