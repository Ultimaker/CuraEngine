// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/SierpinskiFill.h"

#include <algorithm> // swap
#include <assert.h>
#include <functional> // function
#include <iterator> // next, prev

#include <spdlog/spdlog.h>

#include "geometry/Polygon.h"
#include "infill/ImageBasedDensityProvider.h"
#include "infill/UniformDensityProvider.h"
#include "utils/AABB3D.h"
#include "utils/SVG.h"

namespace cura
{

static constexpr bool diagonal = true;
static constexpr bool straight = false;

static constexpr double allowed_length_error = 0.01;

static constexpr bool deep_debug_checking = false;

SierpinskiFill::SierpinskiFill(const DensityProvider& density_provider, const AABB aabb, int max_depth, const coord_t line_width, bool dithering)
    : dithering_(dithering)
    , constraint_error_diffusion_(dithering)
    , density_provider_(density_provider)
    , aabb_(aabb)
    , line_width_(line_width)
    , max_depth_(max_depth)
{
    createTree();

    createLowerBoundSequence();

    for (SierpinskiTriangle* node : sequence_)
    {
        if (node->getValueError() < -allowed_length_error)
        {
            spdlog::error(
                "Node is subdivided without the appropriate value! value_error: {} from base {} el: {} er: {}, while the realized_length = {}",
                node->getValueError(),
                node->requested_length_,
                node->error_left_,
                node->error_right_,
                node->realized_length_);
            assert(false);
        }
    }
    debugCheck(true);

    settleErrors();

    diffuseError();
}

SierpinskiFill::~SierpinskiFill()
{
}


void SierpinskiFill::createTree()
{
    Point2LL lt = Point2LL(aabb_.min_.X, aabb_.max_.Y);
    Point2LL rb = Point2LL(aabb_.max_.X, aabb_.min_.Y);

    bool root_straight_corner_is_left = false;
    int root_depth = 1;
    root_.children.emplace_back(rb, aabb_.min_, aabb_.max_, SierpinskiTriangle::SierpinskiDirection::AC_TO_AB, root_straight_corner_is_left, root_depth);
    root_.children.emplace_back(lt, aabb_.max_, aabb_.min_, SierpinskiTriangle::SierpinskiDirection::AC_TO_AB, root_straight_corner_is_left, root_depth);
    for (SierpinskiTriangle& triangle : root_.children)
    {
        createTree(triangle);
    }

    // calculate node statistics
    createTreeStatistics(root_);

    createTreeRequestedLengths(root_);
}

void SierpinskiFill::createTree(SierpinskiTriangle& sub_root)
{
    if (sub_root.depth_ < max_depth_) // We need to subdivide.
    {
        SierpinskiTriangle& t = sub_root;
        Point2LL middle = (t.a_ + t.b_) / 2;
        // At each subdivision we divide the triangle in two.
        // Figure out which sort of triangle each child will be:
        SierpinskiTriangle::SierpinskiDirection first_dir, second_dir;
        switch (t.dir_)
        {
        default:
        case SierpinskiTriangle::SierpinskiDirection::AB_TO_BC:
            first_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_BC;
            second_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_AB;
            break;
        case SierpinskiTriangle::SierpinskiDirection::AC_TO_AB:
            first_dir = SierpinskiTriangle::SierpinskiDirection::AB_TO_BC;
            second_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_BC;
            break;
        case SierpinskiTriangle::SierpinskiDirection::AC_TO_BC:
            first_dir = SierpinskiTriangle::SierpinskiDirection::AB_TO_BC;
            second_dir = SierpinskiTriangle::SierpinskiDirection::AC_TO_AB;
            break;
        }
        sub_root.children.emplace_back(middle, t.a_, t.straight_corner_, first_dir, ! t.straight_corner_is_left_, t.depth_ + 1);
        sub_root.children.emplace_back(middle, t.straight_corner_, t.b_, second_dir, ! t.straight_corner_is_left_, t.depth_ + 1);
        for (SierpinskiTriangle& child : t.children)
        {
            createTree(child);
        }
    }
}
void SierpinskiFill::createTreeStatistics(SierpinskiTriangle& triangle)
{
    Point2LL ac = triangle.straight_corner_ - triangle.a_;
    double area = 0.5 * INT2MM2(vSize2(ac));
    double short_length = .5 * vSizeMM(ac);
    double long_length = .5 * vSizeMM(triangle.b_ - triangle.a_);
    triangle.area_ = area;
    triangle.realized_length_ = (triangle.dir_ == SierpinskiTriangle::SierpinskiDirection::AC_TO_BC) ? long_length : short_length;
    for (SierpinskiTriangle& child : triangle.children)
    {
        createTreeStatistics(child);
    }
}


void SierpinskiFill::createTreeRequestedLengths(SierpinskiTriangle& triangle)
{
    if (triangle.children.empty())
    { // set requested_length of leaves
        AABB triangle_aabb;
        triangle_aabb.include(triangle.a_);
        triangle_aabb.include(triangle.b_);
        triangle_aabb.include(triangle.straight_corner_);
        AABB3D triangle_aabb3d(Point3LL(triangle_aabb.min_.X, triangle_aabb.min_.Y, 0), Point3LL(triangle_aabb.max_.X, triangle_aabb.max_.Y, 1));
        double density = density_provider_(triangle_aabb3d); // The density of the square around the triangle is a rough estimate of the density of the triangle.
        triangle.requested_length_ = density * triangle.area_ / INT2MM(line_width_);
    }
    else
    { // bubble total up requested_length and total_child_realized_length
        for (SierpinskiTriangle& child : triangle.children)
        {
            createTreeRequestedLengths(child);

            triangle.requested_length_ += child.requested_length_;
            triangle.total_child_realized_length_ += child.realized_length_;
        }
    }
}

void SierpinskiFill::createLowerBoundSequence()
{
    sequence_.emplace_front(&root_);

    if (deep_debug_checking)
        debugCheck();
    for (int iteration = 0; iteration < 999; iteration++)
    {
        bool change = false;
        change |= subdivideAll();
        if (deep_debug_checking)
            debugCheck();

        if (constraint_error_diffusion_)
        {
            change |= bubbleUpConstraintErrors();
            if (deep_debug_checking)
                debugCheck();
        }

        if (! change)
        {
            spdlog::debug("Finished after {} iterations, with a max depth of {}.", iteration + 1, max_depth_);
            break;
        }
    }
}

std::vector<std::vector<std::list<SierpinskiFill::SierpinskiTriangle*>::iterator>> SierpinskiFill::getDepthOrdered()
{
    std::vector<std::vector<std::list<SierpinskiTriangle*>::iterator>> depth_ordered(max_depth_ + 1);
    depth_ordered.resize(max_depth_);
    for (std::list<SierpinskiTriangle*>::iterator it = sequence_.begin(); it != sequence_.end(); ++it)
    {
        SierpinskiTriangle* node = *it;
        depth_ordered[node->depth_].emplace_back(it);
    }
    return depth_ordered;
}

bool SierpinskiFill::subdivideAll()
{
    std::vector<std::vector<std::list<SierpinskiTriangle*>::iterator>> depth_ordered = getDepthOrdered();

    bool change = false;
    for (std::vector<std::list<SierpinskiTriangle*>::iterator>& depth_nodes : depth_ordered)
        for (std::list<SierpinskiTriangle*>::iterator it : depth_nodes)
        {
            SierpinskiTriangle* node = *it;
            SierpinskiTriangle& triangle = *node;

            // The range of consecutive triangles to consider for subdivision simultaneously.
            // Two triangles connected to each other via the long edge must be subdivided simultaneously,
            // so then the range will be two long rather than one.
            std::list<SierpinskiTriangle*>::iterator begin = it;
            std::list<SierpinskiTriangle*>::iterator end = std::next(it);
            if (triangle.dir_ == SierpinskiTriangle::SierpinskiDirection::AC_TO_AB && end != sequence_.end())
            {
                continue; // don't subdivide these two triangles just yet, wait till next iteration
            }
            if (triangle.dir_ == SierpinskiTriangle::SierpinskiDirection::AB_TO_BC && begin != sequence_.begin())
            {
                begin = std::prev(it);
                assert((*begin)->depth_ == triangle.depth_ || isConstrainedBackward(it));
            }
            else
            {
                assert(begin == std::prev(end));
            }
            bool is_constrained = isConstrainedBackward(begin) || isConstrainedForward(std::prev(end));
            // Don't check for constraining in between the cells in the range;
            // the range is defined as the range of triangles which are constraining each other simultaneously.

            if (node->depth_ == max_depth_) // Never subdivide beyond maximum depth.
                continue;
            double total_subdiv_error = getSubdivisionError(begin, end);
            if (! node->children.empty() && total_subdiv_error >= 0 && ! is_constrained)
            {
                bool redistribute_errors = true;
                subdivide(begin, end, redistribute_errors);
                change = true;
            }
        }
    return change;
}

bool SierpinskiFill::bubbleUpConstraintErrors()
{
    std::vector<std::vector<std::list<SierpinskiTriangle*>::iterator>> depth_ordered = getDepthOrdered();

    bool redistributed_anything = false;

    for (int depth = max_depth_; depth >= 0; depth--)
    {
        std::vector<std::list<SierpinskiTriangle*>::iterator>& depth_nodes = depth_ordered[depth];
        for (std::list<SierpinskiTriangle*>::iterator it : depth_nodes)
        {
            SierpinskiTriangle* node = *it;
            SierpinskiTriangle& triangle = *node;

            double unresolvable_error = triangle.getValueError();

            // If constrained in one direction, resolve the error in the other direction only.
            // If constrained in both directions, divide the error equally over both directions.
            bool is_constrained_forward = isConstrainedForward(it);
            bool is_constrained_backward = isConstrainedBackward(it);
            if (unresolvable_error > allowed_length_error && (is_constrained_forward || is_constrained_backward))
            {
                if (is_constrained_forward && is_constrained_backward)
                {
                    // when constrained in both directions, then divide error equally
                    unresolvable_error *= .5;
                    // disperse half of the error forward and the other half backward
                }
                if (deep_debug_checking)
                    debugCheck();
                if (is_constrained_forward)
                {
                    SierpinskiTriangle* next = *std::next(it);
                    node->error_right_ -= unresolvable_error;
                    next->error_left_ += unresolvable_error;
                }
                if (is_constrained_backward)
                {
                    SierpinskiTriangle* prev = *std::prev(it);
                    node->error_left_ -= unresolvable_error;
                    prev->error_right_ += unresolvable_error;
                }
                if (std::abs(unresolvable_error) > allowed_length_error)
                {
                    redistributed_anything = true;
                }
                if (deep_debug_checking)
                    debugCheck();
            }
        }
    }
    return redistributed_anything;
}


std::list<SierpinskiFill::SierpinskiTriangle*>::iterator
    SierpinskiFill::subdivide(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end, bool redistribute_errors)
{
    if (redistribute_errors && deep_debug_checking)
        debugCheck();
    if (redistribute_errors)
    { // move left-over errors
        bool distribute_subdivision_errors = true;
        redistributeLeftoverErrors(begin, end, distribute_subdivision_errors);

        SierpinskiTriangle* first = *begin;
        SierpinskiTriangle* last = *std::prev(end);
        (*begin)->children.front().error_left_ += first->error_left_;
        (*std::prev(end))->children.back().error_right_ += last->error_right_;
    }
    if (redistribute_errors && deep_debug_checking)
        debugCheck(false);

    std::list<SierpinskiFill::SierpinskiTriangle*>::iterator first_child_it = std::prev(begin);
    // the actual subdivision
    for (std::list<SierpinskiTriangle*>::iterator it = begin; it != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        assert(! node->children.empty() && "cannot subdivide node with no children!");
        for (SierpinskiTriangle& child : node->children)
        {
            sequence_.insert(begin, &child);
        }
    }
    first_child_it++;
    // removal of parents
    std::list<SierpinskiTriangle*>::iterator last_child_it = std::prev(begin);
    sequence_.erase(begin, end);

    if (redistribute_errors && deep_debug_checking)
        debugCheck(false);

    if (redistribute_errors)
    { // make positive errors in children well balanced
        // Pass along error from parent
        balanceErrors(first_child_it, std::next(last_child_it));
    }

    if (redistribute_errors && deep_debug_checking)
        debugCheck();

    return last_child_it;
}

void SierpinskiFill::redistributeLeftoverErrors(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end, bool distribute_subdivision_errors)
{
    SierpinskiTriangle* prev = *std::prev(begin);
    SierpinskiTriangle* next = *end;
    SierpinskiTriangle* first = *begin;
    SierpinskiTriangle* last = *std::prev(end);

    // exchange intermediate errors
    for (auto it = begin; it != end && std::next(it) != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        SierpinskiTriangle* other = *std::next(it);
        if (std::abs(node->error_right_ + other->error_left_) > allowed_length_error)
        {
            spdlog::warn("Nodes aren't balanced! er: {} other el: {}", node->error_right_, other->error_left_);
            assert(false);
        }
        double exchange = node->error_right_;
        if (node->error_right_ < other->error_left_)
        {
            exchange *= -1;
        }
        node->error_right_ -= exchange;
        other->error_left_ += exchange;
    }

    double total_superfluous_error = 0;
    for (auto it = begin; it != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        total_superfluous_error += (distribute_subdivision_errors) ? node->getSubdivisionError() : node->getValueError();
    }
    if (total_superfluous_error < allowed_length_error)
    { // there is no significant left-over error
        if (distribute_subdivision_errors && total_superfluous_error < -allowed_length_error)
        {
            spdlog::warn("RedistributeLeftoverErrors shouldn't be called if the node isn't to be subdivided. Total error: {}", total_superfluous_error);
            assert(false);
        }
        return;
    }
    if (begin != sequence_.begin() && end != sequence_.end() && first->error_left_ > allowed_length_error && last->error_right_ > allowed_length_error)
    {
        double total_error_input = first->error_left_ + last->error_right_;
        total_superfluous_error = std::min(total_superfluous_error, total_error_input); // total superfluous error cannot be more than the influx of error
        double left_spillover = total_superfluous_error * first->error_left_ / total_error_input;
        double right_spillover = total_superfluous_error * last->error_right_ / total_error_input;
        (*begin)->error_left_ -= left_spillover;
        prev->error_right_ += left_spillover;
        (*std::prev(end))->error_right_ -= right_spillover;
        next->error_left_ += right_spillover;
    }
    else if (begin != sequence_.begin() && first->error_left_ > allowed_length_error)
    {
        total_superfluous_error = std::min(total_superfluous_error, first->error_left_); // total superfluous error cannot be more than the influx of error
        (*begin)->error_left_ -= total_superfluous_error;
        prev->error_right_ += total_superfluous_error;
        assert(first->error_left_ > -allowed_length_error);
    }
    else if (end != sequence_.end() && last->error_right_ > allowed_length_error)
    {
        total_superfluous_error = std::min(total_superfluous_error, last->error_right_); // total superfluous error cannot be more than the influx of error
        last->error_right_ -= total_superfluous_error;
        next->error_left_ += total_superfluous_error;
        assert(last->error_right_ > -allowed_length_error);
    }
}

void SierpinskiFill::balanceErrors(std::list<SierpinskiFill::SierpinskiTriangle*>::iterator begin, std::list<SierpinskiFill::SierpinskiTriangle*>::iterator end)
{
    // copy sublist to array
    std::vector<SierpinskiFill::SierpinskiTriangle*> nodes;
    for (std::list<SierpinskiFill::SierpinskiTriangle*>::iterator it = begin; it != end; ++it)
    {
        nodes.emplace_back(*it);
    }

    std::vector<double> node_error_compensation(nodes.size());

    // sort children on value_error, i.e. sort on total_value
    std::vector<int> order;
    for (unsigned int node_idx = 0; node_idx < nodes.size(); node_idx++)
    {
        order.emplace_back(node_idx);
    }
    std::stable_sort(
        order.begin(),
        order.end(),
        [&nodes](int a, int b)
        {
            return nodes[a]->getValueError() < nodes[b]->getValueError();
        });

    // add error to children with too low value
    double added = 0;
    unsigned int node_order_idx;
    for (node_order_idx = 0; node_order_idx < nodes.size(); node_order_idx++)
    {
        int node_idx = order[node_order_idx];
        SierpinskiTriangle* node = nodes[node_idx];
        double value_error = node->getValueError();
        if (value_error < 0)
        {
            added -= value_error;
            node_error_compensation[node_idx] = -value_error;
        }
        else
            break;
    }
    if (added == 0)
    {
        return;
    }

    // subtract the added value from remaining children
    // divide acquired negative balancing error among remaining nodes with positive value error
    double subtracted = 0;
    // divide up added among remaining children in ratio to their value error
    double total_remaining_value_error = 0;
    for (unsigned int remaining_node_order_idx = node_order_idx; remaining_node_order_idx < nodes.size(); remaining_node_order_idx++)
    {
        int node_idx = order[remaining_node_order_idx];
        SierpinskiTriangle* node = nodes[node_idx];
        double val_err = node->getValueError();
        assert(val_err > -allowed_length_error);
        total_remaining_value_error += val_err;
    }

    if (total_remaining_value_error < added - allowed_length_error)
    {
        spdlog::warn("total_remaining: {} should be > {}", total_remaining_value_error, added);
        assert(false);
    }

    if (std::abs(total_remaining_value_error) < .0001) // Error is insignificant.
    {
        return;
    }

    for (unsigned int remaining_node_order_idx = node_order_idx; remaining_node_order_idx < nodes.size(); remaining_node_order_idx++)
    {
        int node_idx = order[remaining_node_order_idx];
        SierpinskiTriangle* node = nodes[node_idx];
        double val_error = node->getValueError();
        assert(val_error > -allowed_length_error);

        double diff = added * val_error / total_remaining_value_error;
        subtracted += diff;
        node_error_compensation[node_idx] = -diff;
    }
    if (std::abs(subtracted - added) > allowed_length_error)
    {
        spdlog::warn("Redistribution didn't distribute well!! added {} subtracted {}", added, subtracted);
        assert(false);
    }

    double energy = 0;
    for (unsigned int node_idx = 0; node_idx < nodes.size(); node_idx++)
    {
        nodes[node_idx]->error_left_ -= energy;
        energy += node_error_compensation[node_idx];
        nodes[node_idx]->error_right_ += energy;
    }
    assert(energy < allowed_length_error);
}

void SierpinskiFill::settleErrors()
{
    std::vector<std::vector<std::list<SierpinskiTriangle*>::iterator>> depth_ordered = getDepthOrdered();

    for (std::vector<std::list<SierpinskiTriangle*>::iterator>& depth_nodes : depth_ordered)
    {
        for (std::list<SierpinskiTriangle*>::iterator it : depth_nodes)
        {
            redistributeLeftoverErrors(it, std::next(it), false);
        }
    }
}

void SierpinskiFill::diffuseError()
{
    int pair_constrained_nodes = 0;
    int constrained_nodes = 0;
    int unconstrained_nodes = 0;
    int subdivided_nodes = 0;
    double error = 0;
    for (std::list<SierpinskiTriangle*>::iterator it = sequence_.begin(); it != sequence_.end(); ++it)
    {
        SierpinskiTriangle& triangle = *(*it);

        double boundary = (triangle.realized_length_ + triangle.total_child_realized_length_) * 0.5;

        double nodal_value = ((use_errors_in_dithering_) ? triangle.getErroredValue() : triangle.requested_length_);

        double boundary_error = nodal_value - boundary + error;

        std::list<SierpinskiTriangle*>::iterator begin = it;
        std::list<SierpinskiTriangle*>::iterator end = std::next(it);
        if (triangle.dir_ == SierpinskiTriangle::SierpinskiDirection::AC_TO_AB && end != sequence_.end())
        {
            pair_constrained_nodes++;
            continue; // don't subdivide these two triangles just yet, wait till next iteration
        }
        if (triangle.dir_ == SierpinskiTriangle::SierpinskiDirection::AB_TO_BC && begin != sequence_.begin())
        {
            begin = std::prev(it);
            assert((*begin)->depth_ == triangle.depth_ || isConstrainedBackward(it));
        }


        bool is_constrained = false;
        for (auto nested_it = begin; nested_it != end; ++nested_it)
        {
            if (isConstrainedBackward(nested_it) || isConstrainedForward(nested_it))
            {
                is_constrained = true;
                constrained_nodes++;
                break;
            }
        }
        if (! is_constrained)
            unconstrained_nodes++;
        if (! is_constrained && boundary_error >= 0 && ! triangle.children.empty())
        {
            subdivided_nodes++;
            it = subdivide(begin, end, false);
            if (dithering_)
                error += nodal_value - triangle.total_child_realized_length_;
        }
        else
        {
            if (dithering_)
                error += nodal_value - triangle.realized_length_;
        }
    }
    spdlog::debug(
        "pair_constrained_nodes: {}, constrained_nodes: {}, unconstrained_nodes: {}, subdivided_nodes: {}",
        pair_constrained_nodes,
        constrained_nodes,
        unconstrained_nodes,
        subdivided_nodes);
}

bool SierpinskiFill::isConstrainedBackward(std::list<SierpinskiTriangle*>::iterator it)
{
    SierpinskiTriangle* node = *it;
    SierpinskiTriangle* prev = *std::prev(it);
    if (it != sequence_.begin() && node->dir_ == SierpinskiTriangle::SierpinskiDirection::AB_TO_BC && prev->depth_ < node->depth_)
        return true;
    return false;
}
bool SierpinskiFill::isConstrainedForward(std::list<SierpinskiTriangle*>::iterator it)
{
    SierpinskiTriangle* node = *it;
    SierpinskiTriangle* next = *std::next(it);
    if (std::next(it) != sequence_.end() && node->dir_ == SierpinskiTriangle::SierpinskiDirection::AC_TO_AB && next->depth_ < node->depth_)
        return true;
    return false;
}

double SierpinskiFill::getSubdivisionError(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end)
{
    double ret = 0;
    for (auto it = begin; it != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        ret += node->getSubdivisionError();
    }
    return ret;
}


void SierpinskiFill::debugOutput(SVG& svg)
{
    svg.writePolygon(aabb_.toPolygon(), SVG::Color::RED);

    // draw triangles
    for (SierpinskiTriangle* node : sequence_)
    {
        SierpinskiTriangle& triangle = *node;
        svg.writeLine(triangle.a_, triangle.b_, SVG::Color::GRAY);
        svg.writeLine(triangle.a_, triangle.straight_corner_, SVG::Color::GRAY);
        svg.writeLine(triangle.b_, triangle.straight_corner_, SVG::Color::GRAY);
    }
}


SierpinskiFill::Edge SierpinskiFill::SierpinskiTriangle::getFromEdge()
{
    Edge ret;
    switch (dir_)
    {
    case SierpinskiDirection::AB_TO_BC:
        ret = Edge{ a_, b_ };
        break;
    case SierpinskiDirection::AC_TO_AB:
        ret = Edge{ straight_corner_, a_ };
        break;
    case SierpinskiDirection::AC_TO_BC:
        ret = Edge{ straight_corner_, a_ };
        break;
    }
    if (! straight_corner_is_left_)
    {
        std::swap(ret.l, ret.r);
    }
    return ret;
}

SierpinskiFill::Edge SierpinskiFill::SierpinskiTriangle::getToEdge()
{
    Edge ret;
    switch (dir_)
    {
    case SierpinskiDirection::AB_TO_BC:
        ret = Edge{ straight_corner_, b_ };
        break;
    case SierpinskiDirection::AC_TO_AB:
        ret = Edge{ b_, a_ };
        break;
    case SierpinskiDirection::AC_TO_BC:
        ret = Edge{ straight_corner_, b_ };
        break;
    }
    if (! straight_corner_is_left_)
    {
        std::swap(ret.l, ret.r);
    }
    return ret;
}

double SierpinskiFill::SierpinskiTriangle::getTotalError()
{
    return error_left_ + error_right_;
}

double SierpinskiFill::SierpinskiTriangle::getErroredValue()
{
    return requested_length_ + getTotalError();
}

double SierpinskiFill::SierpinskiTriangle::getSubdivisionError()
{
    return getErroredValue() - total_child_realized_length_;
}

double SierpinskiFill::SierpinskiTriangle::getValueError()
{
    return getErroredValue() - realized_length_;
}


Polygon SierpinskiFill::generateCross() const
{
    Polygon ret;

    for (SierpinskiTriangle* max_level_it : sequence_)
    {
        SierpinskiTriangle& triangle = *max_level_it;
        Point2LL edge_middle = triangle.a_ + triangle.b_ + triangle.straight_corner_;
        switch (triangle.dir_)
        {
        case SierpinskiTriangle::SierpinskiDirection::AB_TO_BC:
            edge_middle -= triangle.a_;
            break;
        case SierpinskiTriangle::SierpinskiDirection::AC_TO_AB:
            edge_middle -= triangle.straight_corner_;
            break;
        case SierpinskiTriangle::SierpinskiDirection::AC_TO_BC:
            edge_middle -= triangle.a_;
            break;
        }
        ret.push_back(edge_middle / 2);
    }

    double realized_length = INT2MM(ret.length());
    double requested_length = root_.requested_length_;
    double error = (realized_length - requested_length) / requested_length;
    spdlog::debug("realized_length: {}, requested_length: {}  :: {}% error", realized_length, requested_length, 0.01 * static_cast<int>(10000 * error));
    return ret;
}

Polygon SierpinskiFill::generateCross(coord_t z, coord_t min_dist_to_side, coord_t pocket_size) const
{
    Polygon ret;

    std::function<Point2LL(int, Edge)> get_edge_crossing_location = [z, min_dist_to_side](const coord_t period, const Edge e)
    {
        coord_t from_l = z % (period * 2);
        if (from_l > period)
        {
            from_l = period * 2 - from_l;
        }
        from_l = from_l * vSize(e.l - e.r) / period;
        from_l = std::max(min_dist_to_side, from_l);
        from_l = std::min(vSize(e.l - e.r) - min_dist_to_side, from_l);
        return e.l + normal(e.r - e.l, from_l);
    };

    SierpinskiTriangle* last_triangle = nullptr;
    for (SierpinskiTriangle* node : sequence_)
    {
        SierpinskiTriangle& triangle = *node;

        /* The length of a side of the triangle is used as the period of
        repetition. That way the edges overhang by not more than 45 degrees.

        While there is a vertex is moving back and forth on the diagonal between
        A and B, this doesn't cause a steeper angle of parallel lines in the
        Cross pattern because the other side is sliding along the straight
        sides. The steeper overhang is then only in the corner, which is deemed
        acceptable since the corners are never too sharp. */
        const coord_t period = vSize(triangle.straight_corner_ - triangle.a_);
        ret.push_back(get_edge_crossing_location(period, triangle.getFromEdge()));

        last_triangle = &triangle;
    }
    assert(last_triangle);
    const coord_t period = vSize(last_triangle->straight_corner_ - last_triangle->a_);
    ret.push_back(get_edge_crossing_location(period, last_triangle->getToEdge()));

    if (pocket_size > 10)
    {
        // round off corners by half square root 2 of the pocket size so that the whole hole will be sqrt_pocket_size wide
        // \      /     \      /
        //  \    /  ==>  \____/
        //   \  /}\       ^^^^--pocket_size / 2
        //    \/} / pocket_size_side
        coord_t pocket_size_side = pocket_size * std::numbers::sqrt2 / 2;

        Polygon pocketed;
        pocketed.reserve(ret.size() * 3 / 2);

        Point2LL p0 = ret.back();
        for (size_t poly_idx = 0; poly_idx < ret.size(); poly_idx++)
        {
            Point2LL p1 = ret[poly_idx];
            Point2LL p2 = ret[(poly_idx + 1) % ret.size()];
            Point2LL v0 = p0 - p1;
            Point2LL v1 = p2 - p1;

            coord_t prod = std::abs(dot(v0, v1));
            bool is_straight_corner = prod < sqrt(vSize(v0) * vSize(v1)) * min_dist_to_side; // allow for rounding errors of up to min_dist_to_side
            if (is_straight_corner)
            {
                coord_t pocket_rounding
                    = std::min(std::min(pocket_size_side, vSize(v0) / 3), vSize(v1) / 3); // a third so that if a line segment is shortened on both sides the middle remains
                pocketed.push_back(p1 + normal(v0, pocket_rounding));
                pocketed.push_back(p1 + normal(v1, pocket_rounding));
            }
            else
            {
                pocketed.push_back(p1);
            }
            p0 = p1;
        }

        return pocketed;
    }
    else
    {
        return ret;
    }
}

void SierpinskiFill::debugCheck(bool check_subdivision)
{
    if (std::abs(sequence_.front()->error_left_) > allowed_length_error)
    {
        spdlog::warn("First node has error left!");
        assert(false);
    }
    if (std::abs(sequence_.back()->error_right_) > allowed_length_error)
    {
        spdlog::warn("Last node has error right!");
        assert(false);
    }

    for (auto it = sequence_.begin(); it != sequence_.end(); ++it)
    {
        if (std::next(it) == sequence_.end())
        {
            break;
        }
        SierpinskiTriangle* node = *it;
        SierpinskiTriangle* next = *std::next(it);

        if (std::abs(node->error_right_ + next->error_left_) > allowed_length_error)
        {
            spdlog::warn("Consecutive nodes in fractal don't have the same error! er: {} , nel: {}", node->error_right_, next->error_left_);
            assert(false);
        }
        if (check_subdivision && node->getValueError() < -allowed_length_error)
        {
            spdlog::warn("Fractal node shouldn't have been subdivided!");
            assert(false);
        }
    }
}

} // namespace cura
