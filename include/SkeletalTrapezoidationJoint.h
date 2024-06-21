// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SKELETAL_TRAPEZOIDATION_JOINT_H
#define SKELETAL_TRAPEZOIDATION_JOINT_H

#include <memory> // smart pointers

#include "BeadingStrategy/BeadingStrategy.h"
#include "geometry/Point2LL.h"

namespace cura
{

class SkeletalTrapezoidationJoint
{
    using Beading = BeadingStrategy::Beading;

public:
    struct BeadingPropagation
    {
        Beading beading_;
        coord_t dist_to_bottom_source_;
        coord_t dist_from_top_source_;
        bool is_upward_propagated_only_;
        BeadingPropagation(const Beading& beading)
            : beading_(beading)
            , dist_to_bottom_source_(0)
            , dist_from_top_source_(0)
            , is_upward_propagated_only_(false)
        {
        }
    };

    coord_t distance_to_boundary_;
    coord_t bead_count_;
    double transition_ratio_; //! The distance near the skeleton to leave free because this joint is in the middle of a transition, as a fraction of the inner bead width of the
                              //! bead at the higher transition.
    SkeletalTrapezoidationJoint()
        : distance_to_boundary_(-1)
        , bead_count_(-1)
        , transition_ratio_(0)
    {
    }

    bool hasBeading() const
    {
        return beading_.use_count() > 0;
    }
    void setBeading(std::shared_ptr<BeadingPropagation>& storage)
    {
        beading_ = storage;
    }
    std::shared_ptr<BeadingPropagation> getBeading() const
    {
        return beading_.lock();
    }

private:
    std::weak_ptr<BeadingPropagation> beading_;
};

} // namespace cura
#endif // SKELETAL_TRAPEZOIDATION_JOINT_H
