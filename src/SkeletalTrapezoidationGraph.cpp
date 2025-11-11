// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SkeletalTrapezoidationGraph.h"

#include <unordered_map>

#include <spdlog/spdlog.h>

#include "utils/linearAlg2D.h"
#include "utils/macros.h"

namespace cura
{

STHalfEdge::STHalfEdge(SkeletalTrapezoidationEdge data)
    : HalfEdge(data)
{
}

bool STHalfEdge::canGoUp(bool strict) const
{
    if (to_->data_.distance_to_boundary_ > from_->data_.distance_to_boundary_)
    {
        return true;
    }
    if (to_->data_.distance_to_boundary_ < from_->data_.distance_to_boundary_ || strict)
    {
        return false;
    }

    // Edge is between equidistqant verts; recurse!
    for (edge_t* outgoing = next_; outgoing != twin_; outgoing = outgoing->twin_->next_)
    {
        if (outgoing->canGoUp())
        {
            return true;
        }
        assert(outgoing->twin_);
        if (! outgoing->twin_)
            return false;
        assert(outgoing->twin_->next_);
        if (! outgoing->twin_->next_)
            return true; // This point is on the boundary?! Should never occur
    }
    return false;
}

bool STHalfEdge::isUpward(const bool strict) const
{
    if (to_->data_.distance_to_boundary_ > from_->data_.distance_to_boundary_)
    {
        return true;
    }
    if (to_->data_.distance_to_boundary_ < from_->data_.distance_to_boundary_ || strict)
    {
        return false;
    }

    // Equidistant edge case:
    std::optional<cura::coord_t> forward_up_dist = this->distToGoUp();
    std::optional<cura::coord_t> backward_up_dist = twin_->distToGoUp();
    if (forward_up_dist && backward_up_dist)
    {
        return forward_up_dist < backward_up_dist;
    }

    if (forward_up_dist)
    {
        return true;
    }

    if (backward_up_dist)
    {
        return false;
    }
    return to_->p_ < from_->p_; // Arbitrary ordering, which returns the opposite for the twin edge
}

std::optional<cura::coord_t> STHalfEdge::distToGoUp() const
{
    if (to_->data_.distance_to_boundary_ > from_->data_.distance_to_boundary_)
    {
        return 0;
    }
    if (to_->data_.distance_to_boundary_ < from_->data_.distance_to_boundary_)
    {
        return std::optional<cura::coord_t>();
    }

    // Edge is between equidistqant verts; recurse!
    std::optional<cura::coord_t> ret;
    for (edge_t* outgoing = next_; outgoing != twin_; outgoing = outgoing->twin_->next_)
    {
        std::optional<cura::coord_t> dist_to_up = outgoing->distToGoUp();
        if (dist_to_up)
        {
            if (ret)
            {
                ret = std::min(*ret, *dist_to_up);
            }
            else
            {
                ret = dist_to_up;
            }
        }
        assert(outgoing->twin_);
        if (! outgoing->twin_)
            return std::optional<cura::coord_t>();
        assert(outgoing->twin_->next_);
        if (! outgoing->twin_->next_)
            return 0; // This point is on the boundary?! Should never occur
    }
    if (ret)
    {
        ret = *ret + cura::vSize(to_->p_ - from_->p_);
    }
    return ret;
}

STHalfEdge* STHalfEdge::getNextUnconnected()
{
    edge_t* result = static_cast<STHalfEdge*>(this);
    while (result->next_)
    {
        result = result->next_;
        if (result == this)
        {
            return nullptr;
        }
    }
    return result->twin_;
}

STHalfEdgeNode::STHalfEdgeNode(SkeletalTrapezoidationJoint data, Point2LL p)
    : HalfEdgeNode(data, p)
{
}

bool STHalfEdgeNode::isMultiIntersection()
{
    int odd_path_count = 0;
    edge_t* outgoing = incident_edge_;
    do
    {
        if (! outgoing)
        { // This is a node on the outside
            return false;
        }
        if (outgoing->data_.isCentral())
        {
            odd_path_count++;
        }
    } while (outgoing = outgoing->twin_->next_, outgoing != incident_edge_);
    return odd_path_count > 2;
}

bool STHalfEdgeNode::isCentral() const
{
    edge_t* edge = incident_edge_;
    do
    {
        if (edge->data_.isCentral())
        {
            return true;
        }
        assert(edge->twin_);
        if (! edge->twin_)
            return false;
    } while (edge = edge->twin_->next_, edge != incident_edge_);
    return false;
}

bool STHalfEdgeNode::isLocalMaximum(bool strict) const
{
    if (data_.distance_to_boundary_ == 0)
    {
        return false;
    }

    edge_t* edge = incident_edge_;
    do
    {
        if (edge->canGoUp(strict))
        {
            return false;
        }
        assert(edge->twin_);
        if (! edge->twin_)
            return false;

        if (! edge->twin_->next_)
        { // This point is on the boundary
            return false;
        }
    } while (edge = edge->twin_->next_, edge != incident_edge_);
    return true;
}

void SkeletalTrapezoidationGraph::collapseSmallEdges(coord_t snap_dist)
{
    std::unordered_map<edge_t*, std::list<edge_t>::iterator> edge_locator;
    std::unordered_map<node_t*, std::list<node_t>::iterator> node_locator;

    for (auto edge_it = edges.begin(); edge_it != edges.end(); ++edge_it)
    {
        edge_locator.emplace(&*edge_it, edge_it);
    }

    for (auto node_it = nodes.begin(); node_it != nodes.end(); ++node_it)
    {
        node_locator.emplace(&*node_it, node_it);
    }

    auto safelyRemoveEdge = [this, &edge_locator](edge_t* to_be_removed, std::list<edge_t>::iterator& current_edge_it, bool& edge_it_is_updated)
    {
        if (current_edge_it != edges.end() && to_be_removed == &*current_edge_it)
        {
            current_edge_it = edges.erase(current_edge_it);
            edge_it_is_updated = true;
        }
        else
        {
            edges.erase(edge_locator[to_be_removed]);
        }
    };

    auto should_collapse = [snap_dist](node_t* a, node_t* b)
    {
        return shorterThen(a->p_ - b->p_, snap_dist);
    };

    for (auto edge_it = edges.begin(); edge_it != edges.end();)
    {
        if (edge_it->prev_)
        {
            edge_it++;
            continue;
        }

        edge_t* quad_start = &*edge_it;
        edge_t* quad_end = quad_start;
        while (quad_end->next_)
            quad_end = quad_end->next_;
        edge_t* quad_mid = (quad_start->next_ == quad_end) ? nullptr : quad_start->next_;

        bool edge_it_is_updated = false;
        if (quad_mid && should_collapse(quad_mid->from_, quad_mid->to_))
        {
            assert(quad_mid->twin_);
            if (! quad_mid->twin_)
            {
                RUN_ONCE(spdlog::warn("Encountered quad edge without a twin."));
                continue; // Prevent accessing unallocated memory.
            }
            int count = 0;
            for (edge_t* edge_from_3 = quad_end; edge_from_3 && edge_from_3 != quad_mid->twin_; edge_from_3 = edge_from_3->twin_->next_)
            {
                edge_from_3->from_ = quad_mid->from_;
                edge_from_3->twin_->to_ = quad_mid->from_;
                if (++count > 1000)
                {
                    break;
                }
            }

            // o-o > collapse top
            // | |
            // | |
            // | |
            // o o
            if (quad_mid->from_->incident_edge_ == quad_mid)
            {
                if (quad_mid->twin_->next_)
                {
                    quad_mid->from_->incident_edge_ = quad_mid->twin_->next_;
                }
                else
                {
                    quad_mid->from_->incident_edge_ = quad_mid->prev_->twin_;
                }
            }

            nodes.erase(node_locator[quad_mid->to_]);

            quad_mid->prev_->next_ = quad_mid->next_;
            quad_mid->next_->prev_ = quad_mid->prev_;
            quad_mid->twin_->next_->prev_ = quad_mid->twin_->prev_;
            quad_mid->twin_->prev_->next_ = quad_mid->twin_->next_;

            safelyRemoveEdge(quad_mid->twin_, edge_it, edge_it_is_updated);
            safelyRemoveEdge(quad_mid, edge_it, edge_it_is_updated);
        }

        //  o-o
        //  | | > collapse sides
        //  o o
        if (should_collapse(quad_start->from_, quad_end->to_) && should_collapse(quad_start->to_, quad_end->from_))
        { // Collapse start and end edges and remove whole cell

            quad_start->twin_->to_ = quad_end->to_;
            quad_end->to_->incident_edge_ = quad_end->twin_;
            if (quad_end->from_->incident_edge_ == quad_end)
            {
                if (quad_end->twin_->next_)
                {
                    quad_end->from_->incident_edge_ = quad_end->twin_->next_;
                }
                else
                {
                    quad_end->from_->incident_edge_ = quad_end->prev_->twin_;
                }
            }
            nodes.erase(node_locator[quad_start->from_]);

            quad_start->twin_->twin_ = quad_end->twin_;
            quad_end->twin_->twin_ = quad_start->twin_;
            safelyRemoveEdge(quad_start, edge_it, edge_it_is_updated);
            safelyRemoveEdge(quad_end, edge_it, edge_it_is_updated);
        }
        // If only one side had collapsable length then the cell on the other side of that edge has to collapse
        // if we would collapse that one edge then that would change the quad_start and/or quad_end of neighboring cells
        // this is to do with the constraint that !prev == !twin.next

        if (! edge_it_is_updated)
        {
            edge_it++;
        }
    }
}

void SkeletalTrapezoidationGraph::makeRib(edge_t*& prev_edge, Point2LL start_source_point, Point2LL end_source_point)
{
    Point2LL p = LinearAlg2D::getClosestOnLine(prev_edge->to_->p_, start_source_point, end_source_point);
    coord_t dist = vSize(prev_edge->to_->p_ - p);
    prev_edge->to_->data_.distance_to_boundary_ = dist;
    assert(dist >= 0);

    nodes.emplace_front(SkeletalTrapezoidationJoint(), p);
    node_t* node = &nodes.front();
    node->data_.distance_to_boundary_ = 0;

    edges.emplace_front(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::EXTRA_VD));
    edge_t* forth_edge = &edges.front();
    edges.emplace_front(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::EXTRA_VD));
    edge_t* back_edge = &edges.front();

    prev_edge->next_ = forth_edge;
    forth_edge->prev_ = prev_edge;
    forth_edge->from_ = prev_edge->to_;
    forth_edge->to_ = node;
    forth_edge->twin_ = back_edge;
    back_edge->twin_ = forth_edge;
    back_edge->from_ = node;
    back_edge->to_ = prev_edge->to_;
    node->incident_edge_ = back_edge;

    prev_edge = back_edge;
}

std::pair<SkeletalTrapezoidationGraph::edge_t*, SkeletalTrapezoidationGraph::edge_t*> SkeletalTrapezoidationGraph::insertRib(edge_t& edge, node_t* mid_node)
{
    edge_t* edge_before = edge.prev_;
    edge_t* edge_after = edge.next_;
    node_t* node_before = edge.from_;
    node_t* node_after = edge.to_;

    Point2LL p = mid_node->p_;

    std::pair<Point2LL, Point2LL> source_segment = getSource(edge);
    Point2LL px = LinearAlg2D::getClosestOnLineSegment(p, source_segment.first, source_segment.second);
    coord_t dist = vSize(p - px);
    assert(dist > 0);
    mid_node->data_.distance_to_boundary_ = dist;
    mid_node->data_.transition_ratio_ = 0; // Both transition end should have rest = 0, because at the ends a whole number of beads fits without rest

    nodes.emplace_back(SkeletalTrapezoidationJoint(), px);
    node_t* source_node = &nodes.back();
    source_node->data_.distance_to_boundary_ = 0;

    edge_t* first = &edge;
    edges.emplace_back(SkeletalTrapezoidationEdge());
    edge_t* second = &edges.back();
    edges.emplace_back(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::TRANSITION_END));
    edge_t* outward_edge = &edges.back();
    edges.emplace_back(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EdgeType::TRANSITION_END));
    edge_t* inward_edge = &edges.back();

    if (edge_before)
    {
        edge_before->next_ = first;
    }
    first->next_ = outward_edge;
    outward_edge->next_ = nullptr;
    inward_edge->next_ = second;
    second->next_ = edge_after;

    if (edge_after)
    {
        edge_after->prev_ = second;
    }
    second->prev_ = inward_edge;
    inward_edge->prev_ = nullptr;
    outward_edge->prev_ = first;
    first->prev_ = edge_before;

    first->to_ = mid_node;
    outward_edge->to_ = source_node;
    inward_edge->to_ = mid_node;
    second->to_ = node_after;

    first->from_ = node_before;
    outward_edge->from_ = mid_node;
    inward_edge->from_ = source_node;
    second->from_ = mid_node;

    node_before->incident_edge_ = first;
    mid_node->incident_edge_ = outward_edge;
    source_node->incident_edge_ = inward_edge;
    if (edge_after)
    {
        node_after->incident_edge_ = edge_after;
    }

    first->data_.setIsCentral(true);
    outward_edge->data_.setIsCentral(false); // TODO verify this is always the case.
    inward_edge->data_.setIsCentral(false);
    second->data_.setIsCentral(true);

    outward_edge->twin_ = inward_edge;
    inward_edge->twin_ = outward_edge;

    first->twin_ = nullptr; // we don't know these yet!
    second->twin_ = nullptr;

    assert(second->prev_->from_->data_.distance_to_boundary_ == 0);

    return std::make_pair(first, second);
}

SkeletalTrapezoidationGraph::edge_t* SkeletalTrapezoidationGraph::insertNode(edge_t* edge, Point2LL mid, coord_t mide_node_bead_count)
{
    edge_t* last_edge_replacing_input = edge;

    nodes.emplace_back(SkeletalTrapezoidationJoint(), mid);
    node_t* mid_node = &nodes.back();

    edge_t* twin = last_edge_replacing_input->twin_;
    last_edge_replacing_input->twin_ = nullptr;
    twin->twin_ = nullptr;
    std::pair<edge_t*, edge_t*> left_pair = insertRib(*last_edge_replacing_input, mid_node);
    std::pair<edge_t*, edge_t*> right_pair = insertRib(*twin, mid_node);
    edge_t* first_edge_replacing_input = left_pair.first;
    last_edge_replacing_input = left_pair.second;
    edge_t* first_edge_replacing_twin = right_pair.first;
    edge_t* last_edge_replacing_twin = right_pair.second;

    first_edge_replacing_input->twin_ = last_edge_replacing_twin;
    last_edge_replacing_twin->twin_ = first_edge_replacing_input;
    last_edge_replacing_input->twin_ = first_edge_replacing_twin;
    first_edge_replacing_twin->twin_ = last_edge_replacing_input;

    mid_node->data_.bead_count_ = mide_node_bead_count;

    return last_edge_replacing_input;
}

std::pair<Point2LL, Point2LL> SkeletalTrapezoidationGraph::getSource(const edge_t& edge)
{
    const edge_t* from_edge = &edge;
    while (from_edge->prev_)
    {
        from_edge = from_edge->prev_;
    }

    const edge_t* to_edge = &edge;
    while (to_edge->next_)
    {
        to_edge = to_edge->next_;
    }

    return std::make_pair(from_edge->from_->p_, to_edge->to_->p_);
}

} // namespace cura
