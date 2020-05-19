//Copyright (c) 2020 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_EDGE_H
#define VORONOI_QUADRILATERALIZATION_EDGE_H

#include <memory> // smart pointers
#include <list>
#include <vector>

#include "utils/ExtrusionJunction.h"

namespace arachne
{
    using namespace cura;

class SkeletalTrapezoidationEdge
{
    using type_t = int_least16_t;
    using edge_t = SkeletalTrapezoidationEdge;
public:
    /*!
     * Representing the location along an edge where the anchor position of a transition should be placed.
     */
    struct TransitionMiddle
    {
        coord_t pos; // Position along edge as measure from edge.from.p
        coord_t lower_bead_count;
        TransitionMiddle(coord_t pos, coord_t lower_bead_count)
            : pos(pos), lower_bead_count(lower_bead_count)
        {}
    };

    /*!
     * Represents the location along an edge where the lower or upper end of a transition should be placed.
     */
    struct TransitionEnd
    {
        coord_t pos; // Position along edge as measure from edge.from.p, where the edge is always the half edge oriented from lower to higher R
        coord_t lower_bead_count;
        bool is_lower_end; // Whether this is the ed of the transition with lower bead count
        TransitionEnd(coord_t pos, coord_t lower_bead_count, bool is_lower_end)
            : pos(pos), lower_bead_count(lower_bead_count), is_lower_end(is_lower_end)
        {}
    };

    type_t type;
    static constexpr type_t NORMAL = 0; // from voronoi diagram
    static constexpr type_t EXTRA_VD = 1; // introduced to voronoi diagram in order to make the gMAT
    static constexpr type_t TRANSITION_END = 2; // introduced to voronoi diagram in order to make the gMAT

    SkeletalTrapezoidationEdge()
    : SkeletalTrapezoidationEdge(NORMAL)
    {}
    SkeletalTrapezoidationEdge(type_t type)
    : type(type)
    , is_marked(-1)
    {}

    bool isMarked() const
    {
        assert(is_marked != -1);
        return is_marked;
    }
    void setMarked(bool b)
    {
        is_marked = b;
    }
    bool markingIsSet() const
    {
        return is_marked >= 0;
    }

    bool hasTransitions(bool ignore_empty = false) const
    {
        return transitions_.use_count() > 0 && (ignore_empty || ! transitions_.lock()->empty());
    }
    void initTransitions(std::shared_ptr<std::list<TransitionMiddle>> storage)
    {
        transitions_ = storage;
    }
    std::shared_ptr<std::list<TransitionMiddle>> transitions()
    {
        return transitions_.lock();
    }

    bool hasTransitionEnds(bool ignore_empty = false) const
    {
        return transition_ends_.use_count() > 0 && (ignore_empty || ! transition_ends_.lock()->empty());
    }
    void initTransitionEnds(std::shared_ptr<std::list<TransitionEnd>> storage)
    {
        transition_ends_ = storage;
    }
    std::shared_ptr<std::list<TransitionEnd>> transition_ends()
    {
        return transition_ends_.lock();
    }

    bool hasExtrusionJunctions(bool ignore_empty = false) const
    {
        return extrusion_juntions_.use_count() > 0 && (ignore_empty || ! extrusion_juntions_.lock()->empty());
    }
    void initExtrusionJunctions(std::shared_ptr<std::vector<ExtrusionJunction>> storage)
    {
        extrusion_juntions_ = storage;
    }
    std::shared_ptr<std::vector<ExtrusionJunction>> extrusion_juntions()
    {
        return extrusion_juntions_.lock();
    }

private:
    int_least8_t is_marked; //! whether the edge is significant; whether the source segments have a sharp angle; -1 is unknown

    std::weak_ptr<std::list<TransitionMiddle>> transitions_;
    std::weak_ptr<std::list<TransitionEnd>> transition_ends_;
    std::weak_ptr<std::vector<ExtrusionJunction>> extrusion_juntions_;
};


} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_EDGE_H
