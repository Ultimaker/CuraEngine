// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SKELETAL_TRAPEZOIDATION_EDGE_H
#define SKELETAL_TRAPEZOIDATION_EDGE_H

#include <cassert>
#include <list>
#include <memory> // smart pointers
#include <vector>

#include "utils/ExtrusionJunction.h"

namespace cura
{

class SkeletalTrapezoidationEdge
{
private:
    enum class Central : int
    {
        UNKNOWN = -1,
        NO = 0,
        YES = 1
    };

public:
    /*!
     * Representing the location along an edge where the anchor position of a transition should be placed.
     */
    struct TransitionMiddle
    {
        coord_t pos_; // Position along edge as measure from edge.from.p
        int lower_bead_count_;
        coord_t feature_radius_; // The feature radius at which this transition is placed
        TransitionMiddle(coord_t pos, int lower_bead_count, coord_t feature_radius)
            : pos_(pos)
            , lower_bead_count_(lower_bead_count)
            , feature_radius_(feature_radius)
        {
        }
    };

    /*!
     * Represents the location along an edge where the lower or upper end of a transition should be placed.
     */
    struct TransitionEnd
    {
        coord_t pos_; // Position along edge as measure from edge.from.p, where the edge is always the half edge oriented from lower to higher R
        int lower_bead_count_;
        bool is_lower_end_; // Whether this is the ed of the transition with lower bead count
        TransitionEnd(coord_t pos, int lower_bead_count, bool is_lower_end)
            : pos_(pos)
            , lower_bead_count_(lower_bead_count)
            , is_lower_end_(is_lower_end)
        {
        }
    };

    enum class EdgeType : int
    {
        NORMAL = 0, // from voronoi diagram
        EXTRA_VD = 1, // introduced to voronoi diagram in order to make the gMAT
        TRANSITION_END = 2 // introduced to voronoi diagram in order to make the gMAT
    };
    EdgeType type_;

    SkeletalTrapezoidationEdge()
        : SkeletalTrapezoidationEdge(EdgeType::NORMAL)
    {
    }
    SkeletalTrapezoidationEdge(const EdgeType& type)
        : type_(type)
        , is_central(Central::UNKNOWN)
    {
    }

    bool isCentral() const
    {
        assert(is_central != Central::UNKNOWN);
        return is_central == Central::YES;
    }
    void setIsCentral(bool b)
    {
        is_central = b ? Central::YES : Central::NO;
    }
    bool centralIsSet() const
    {
        return is_central != Central::UNKNOWN;
    }

    bool hasTransitions(bool ignore_empty = false) const
    {
        return transitions_.use_count() > 0 && (ignore_empty || ! transitions_.lock()->empty());
    }
    void setTransitions(std::shared_ptr<std::list<TransitionMiddle>>& storage)
    {
        transitions_ = storage;
    }
    std::shared_ptr<std::list<TransitionMiddle>> getTransitions()
    {
        return transitions_.lock();
    }

    bool hasTransitionEnds(bool ignore_empty = false) const
    {
        return transition_ends_.use_count() > 0 && (ignore_empty || ! transition_ends_.lock()->empty());
    }
    void setTransitionEnds(std::shared_ptr<std::list<TransitionEnd>>& storage)
    {
        transition_ends_ = storage;
    }
    std::shared_ptr<std::list<TransitionEnd>> getTransitionEnds()
    {
        return transition_ends_.lock();
    }

    bool hasExtrusionJunctions(bool ignore_empty = false) const
    {
        return extrusion_junctions_.use_count() > 0 && (ignore_empty || ! extrusion_junctions_.lock()->empty());
    }
    void setExtrusionJunctions(std::shared_ptr<LineJunctions>& storage)
    {
        extrusion_junctions_ = storage;
    }
    std::shared_ptr<LineJunctions> getExtrusionJunctions()
    {
        return extrusion_junctions_.lock();
    }

    Central is_central; //! whether the edge is significant; whether the source segments have a sharp angle; -1 is unknown

private:
    std::weak_ptr<std::list<TransitionMiddle>> transitions_;
    std::weak_ptr<std::list<TransitionEnd>> transition_ends_;
    std::weak_ptr<LineJunctions> extrusion_junctions_;
};


} // namespace cura
#endif // SKELETAL_TRAPEZOIDATION_EDGE_H
