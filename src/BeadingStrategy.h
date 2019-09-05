//Copyright (c) 2019 Ultimaker B.V.


#ifndef BEADING_STRATEGY_H
#define BEADING_STRATEGY_H

#include <utility>

#include "utils/IntPoint.h"
#include "utils/logoutput.h"

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
        coord_t total_thickness;
        std::vector<coord_t> bead_widths; //! The line width of each bead from the outer inset inward
        std::vector<coord_t> toolpath_locations; //! The distance of the toolpath location of each bead from the outline
        coord_t left_over; //! The distance not covered by any bead; gap area.
    };

    coord_t optimal_width; //! optimal bead width

    /*!
     * The maximum angle between outline segments smaller than which we are going to add transitions
     * Equals 180 - the "limit bisector angle" from the paper
     */
    float transitioning_angle;

    BeadingStrategy(coord_t optimal_width, float transitioning_angle = M_PI / 3)
    : optimal_width(optimal_width)
    , transitioning_angle(transitioning_angle)
    {
    }

    virtual ~BeadingStrategy()
    {}

    /*!
     * Retrieve the bead widths with which to cover a given thickness.
     * 
     * Requirement: Given a constant \p bead_count the output of each bead width must change gradually along with the \p thickness.
     * 
     * \note The \p bead_count might be different from the \ref BeadingStrategy::optimal_bead_count
     */
    virtual Beading compute(coord_t thickness, coord_t bead_count) const = 0;

    /*!
     * The ideal thickness for a given \param bead_count
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

    /*!
     * The length of the transitioning region along the marked / significant regions of the skeleton.
     * 
     * Transitions are used to smooth out the jumps in integer bead count; the jumps turn into ramps with some incline defined by their length.
     */
    virtual coord_t getTransitioningLength(coord_t lower_bead_count) const
    {
        if (lower_bead_count == 0)
        {
            return 10;
        }
        return optimal_width;
    }

    /*!
     * The fraction of the transition length to put between the lower end of the transition and the point where the unsmoothed bead count jumps.
     * 
     * Transitions are used to smooth out the jumps in integer bead count; the jumps turn into ramps which could be positioned relative to the jump location.
     */
    virtual float getTransitionAnchorPos(coord_t lower_bead_count) const
    {
        coord_t lower_optimum = optimal_thickness(lower_bead_count);
        coord_t transition_point = transition_thickness(lower_bead_count);
        coord_t upper_optimum = optimal_thickness(lower_bead_count + 1);
        return 1.0 - float(transition_point - lower_optimum) / float(upper_optimum - lower_optimum);
    }

    /*!
     * Get the locations in a bead count region where \ref BeadingStrategy::compute exhibits a bend in the widths.
     * Ordered from lower thickness to higher.
     * 
     * This is used to insert extra support bones into the skeleton, so that the resulting beads in long trapezoids don't linearly change between the two ends.
     */
    virtual std::vector<coord_t> getNonlinearThicknesses(coord_t lower_bead_count) const
    {
        return std::vector<coord_t>();
    }

    static bool checkTranisionThicknessConsistency(const BeadingStrategy* strategy);
};




} // namespace arachne
#endif // BEADING_STRATEGY_H
