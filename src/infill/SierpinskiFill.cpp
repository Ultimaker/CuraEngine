/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SierpinskiFill.h"

#include <algorithm> // swap 
#include <functional> // function
#include <iterator> // next, prev

#include "../utils/linearAlg2D.h" // rotateAround
#include "../utils/optional.h" // rotateAround

#include "ImageBasedDensityProvider.h"
#include "UniformDensityProvider.h"

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

static constexpr float sqrt2 = 1.41421356237;

static constexpr float allowed_length_error = .01;

static constexpr bool deep_debug_checking = false;

SierpinskiFill::SierpinskiFill(const DensityProvider& density_provider, const AABB aabb, int max_depth, const coord_t line_width, bool dithering)
: dithering(dithering)
, constraint_error_diffusion(dithering)
, density_provider(density_provider)
, aabb(aabb)
, line_width(line_width)
, max_depth(max_depth)
{
    createTree();
    
    sequence.emplace_front(&root);
    
    createLowerBoundSequence();
    
    diffuseError();
}

SierpinskiFill::~SierpinskiFill()
{
}


void SierpinskiFill::createTree()
{
    Point m = aabb.min;
    Point lt = Point(m.X, aabb.max.Y);
    Point rb = Point(aabb.max.X, m.Y);
    
    root.children.emplace_back(rb, m, aabb.max, SierpinskiTriangle::SierpinskiDirection::AC_TO_AB, false, 1);
    root.children.emplace_back(lt, aabb.max, m, SierpinskiTriangle::SierpinskiDirection::AC_TO_AB, false, 1);
    for (SierpinskiTriangle& triangle : root.children)
        createTree(triangle);

    // calculate node statistics
    createTreeStatistics(root);
    
    createTreeRequestedLengths(root);
}

void SierpinskiFill::createTree(SierpinskiTriangle& sub_root)
{
    if (sub_root.depth < max_depth)
    {
        SierpinskiTriangle& t = sub_root;
        Point middle = (t.a + t.b) / 2;
        SierpinskiTriangle::SierpinskiDirection first_dir, second_dir;
        switch(t.dir)
        {
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
        sub_root.children.emplace_back(middle, t.a, t.straight_corner, first_dir, !t.straight_corner_is_left, t.depth + 1);
        sub_root.children.emplace_back(middle, t.straight_corner, t.b, second_dir, !t.straight_corner_is_left, t.depth + 1);
        for (SierpinskiTriangle& child : t.children)
        {
            createTree(child);
        }
    }
}
void SierpinskiFill::createTreeStatistics(SierpinskiTriangle& triangle)
{
    Point ac = triangle.straight_corner - triangle.a;
    float area = 0.5 * INT2MM2(vSize2(ac));
    float short_length = .5 * vSizeMM(ac);
    float long_length = .5 * vSizeMM(triangle.b - triangle.a);
    triangle.area = area;
    triangle.realized_length = (triangle.dir == SierpinskiTriangle::SierpinskiDirection::AC_TO_BC)? long_length : short_length;
    for (SierpinskiTriangle& child : triangle.children)
    {
        createTreeStatistics(child);
    }
}


void SierpinskiFill::createTreeRequestedLengths(SierpinskiTriangle& triangle)
{
    if (triangle.children.empty())
    {
    // set requested_length of leaves
        float density = density_provider(triangle.a, triangle.b, triangle.straight_corner, triangle.straight_corner);
        triangle.requested_length = density * triangle.area / INT2MM(line_width);
    }
    else
    {
    // bubble total up requested_length and total_child_realized_length
        for (SierpinskiTriangle& child : triangle.children)
        {
            createTreeRequestedLengths(child);
            
            triangle.requested_length += child.requested_length;
            triangle.total_child_realized_length += child.realized_length;
        }
    }
}

void SierpinskiFill::createLowerBoundSequence()
{
    for (int iteration = 0; iteration < 999; iteration++)
    {
        bool change = false;
        change |= subdivideAll();
        
        if (constraint_error_diffusion)
        {
            change |= bubbleUpConstraintErrors();
        }
        
        
        if (!change)
        {
            std::cerr << "Finished after "<<(iteration + 1)<<" iterations, with max_depth = "<<max_depth << '\n';
            break;
        }
    }
}

bool SierpinskiFill::subdivideAll()
{
    std::vector<std::list<std::list<SierpinskiTriangle*>::iterator>> depth_ordered;
    { // compute depth_ordered
        for (int i = 0; i < max_depth + 1; i++)
        {
            depth_ordered.emplace_back();
        }
        for (std::list<SierpinskiTriangle*>::iterator it = sequence.begin(); it != sequence.end(); ++it)
        {
            SierpinskiTriangle*& node = *it;
            depth_ordered[node->depth].emplace_back(it);
        }
    }
    
    bool change = false;
    for (std::list<std::list<SierpinskiTriangle*>::iterator>& depth_nodes : depth_ordered)
        for (std::list<SierpinskiTriangle*>::iterator it : depth_nodes)
        {
            SierpinskiTriangle*& node = *it;
            SierpinskiTriangle& triangle = *node;
            
            std::list<SierpinskiTriangle*>::iterator begin = it;
            std::list<SierpinskiTriangle*>::iterator end = std::next(it);
            if (
                triangle.dir == SierpinskiTriangle::SierpinskiDirection::AC_TO_AB
                && end != sequence.end()
            )
            {
                continue; // don't subdivide these two tringles just yet, wait till next iteration
            }
            if (triangle.dir == SierpinskiTriangle::SierpinskiDirection::AB_TO_BC
                && begin != sequence.begin()
            )
            {
                begin = std::prev(it);
                assert((*begin)->depth == triangle.depth || isConstrainedBackward(it));
            }
            else
            {
                assert(begin == std::prev(end));
            }
            bool is_constrained = false;
            if (is_constrained)
            {
                for (auto nested_it = begin; nested_it != end; ++nested_it)
                {
                    if (isConstrainedBackward(nested_it) || isConstrainedForward(nested_it))
                    {
                        is_constrained = true;
                    }
                }
            }
            if (node->depth == max_depth)
                continue;
            float total_subdiv_error = getSubdivisionError(begin, end);
            if (
                !node->children.empty()
                && total_subdiv_error >= 0
                && !is_constrained
                )
            {
                subdivide(begin, end, true);
                change = true;
            }
        }
    return change;
}

bool SierpinskiFill::bubbleUpConstraintErrors()
{
    std::vector<std::list<std::list<SierpinskiTriangle*>::iterator>> depth_ordered(max_depth + 1);
    { // compute depth_ordered
        for (int i = 0; i < max_depth + 1; i++)
        {
            depth_ordered.emplace_back();
        }
        for (std::list<SierpinskiTriangle*>::iterator it = sequence.begin(); it != sequence.end(); ++it)
        {
            SierpinskiTriangle* node = *it;
            depth_ordered[node->depth].emplace_back(it);
        }
    }
    
    bool redistributed_anything = false;
    
    for (int depth = max_depth; depth >= 0; depth--)
    {
        std::list<std::list<SierpinskiTriangle*>::iterator>& depth_nodes = depth_ordered[depth];
        for (std::list<SierpinskiTriangle*>::iterator it : depth_nodes)
        {
            SierpinskiTriangle* node = *it;
            SierpinskiTriangle& triangle = *node;
            
            float unresolvable_error = triangle.getValueError(); // node.getSubdivisionError();
            
            bool is_constrained_forward = isConstrainedForward(it);
            bool is_constrained_backward = isConstrainedBackward(it);
            if (
                unresolvable_error > allowed_length_error &&
                (is_constrained_forward || is_constrained_backward)
                )
            {
                if (is_constrained_forward && is_constrained_backward)
                {
                    // when constrained in both directions, then divide error equally
                    unresolvable_error *= .5;
                    // disperse half of the error forward and the other half backward
                }
                if (deep_debug_checking) debugCheck();
                if (is_constrained_forward)
                {
                    SierpinskiTriangle* next = *std::next(it);
                    node->error_right -= unresolvable_error;
                    next->error_left += unresolvable_error;
                }
                if (is_constrained_backward)
                {
                    SierpinskiTriangle* prev = *std::prev(it);
                    node->error_left -= unresolvable_error;
                    prev->error_right += unresolvable_error;
                }
                if (std::abs(unresolvable_error) > allowed_length_error)
                {
                    redistributed_anything = true;
                }
                if (deep_debug_checking) debugCheck();
            }
        }
    }
    return redistributed_anything;
}


std::list<SierpinskiFill::SierpinskiTriangle*>::iterator SierpinskiFill::subdivide(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end, bool redistribute_errors)
{
    if (redistribute_errors && deep_debug_checking) debugCheck();
    if (redistribute_errors)
    { // move left-over errors
        redistributeLeftoverErrors(begin, end);

        SierpinskiTriangle* first = *begin;
        SierpinskiTriangle* last = *std::prev(end);
        (*begin)->children.front().error_left += first->error_left;
        (*std::prev(end))->children.back().error_right += last->error_right;

    }
    if (redistribute_errors && deep_debug_checking) debugCheck(false);
    
    std::list<SierpinskiFill::SierpinskiTriangle*>::iterator first_child_it = std::prev(begin);
    // the actual subdivision
    for (std::list<SierpinskiTriangle*>::iterator it = begin; it != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        assert(!node->children.empty() && "cannot subdivide node with no children!");
        for (SierpinskiTriangle& child : node->children)
        {
            sequence.insert(begin, &child);
        }
    }
    first_child_it++;
    // removal of parents
    std::list<SierpinskiTriangle*>::iterator last_child_it = std::prev(begin);
    sequence.erase(begin, end);

    if (redistribute_errors && deep_debug_checking) debugCheck(false);
    
    if (redistribute_errors)
    { // make positive errors in children well balanced
        // Pass along error from parent
        balanceErrors(first_child_it, std::next(last_child_it));
    }
    
    if (redistribute_errors && deep_debug_checking) debugCheck();

    return last_child_it;
}

void SierpinskiFill::redistributeLeftoverErrors(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end)
{
    SierpinskiTriangle* prev = *std::prev(begin);
    SierpinskiTriangle* next = *end;
    SierpinskiTriangle* first = *begin;
    SierpinskiTriangle* last = *std::prev(end);

    // exchange intermediate errors
    for (auto it = begin; it != end; ++it)
    {
        if (std::next(it) == end)
        {
            break;
        }
        SierpinskiTriangle* node = *it;
        SierpinskiTriangle* next = *std::next(it);
        if (std::abs(node->error_right + next->error_left) > allowed_length_error)
        {
            std::cerr << "Nodes aren't balanced! er: " << node->error_right << " next el: " << next->error_left << '\n';
            assert(false);
        }
        float exchange = node->error_right;
        if (node->error_right < next->error_left)
        {
            exchange *= -1;
        }
        node->error_right -= exchange;
        next->error_left += exchange;
    }

    float total_subdiv_error = 0;
    for (auto it = begin; it != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        total_subdiv_error += node->getSubdivisionError();
    }
    if (total_subdiv_error < allowed_length_error)
    { // there is no left-over error
        if (total_subdiv_error < -allowed_length_error)
        {
            std::cerr << "redistributeLeftoverErrors shouldn't be called if the node isn't to be subdivided. Total error: " << total_subdiv_error << "\n";
            assert(false);
        }
        return;
    }
    if (begin != sequence.begin() && end != sequence.end() && first->error_left > allowed_length_error && last->error_right > allowed_length_error)
    {
        float total_error = first->error_left + last->error_right;
        float overflow = std::min(total_subdiv_error, total_error);
        float left_spillover = overflow * first->error_left / total_error;
        float right_spillover = overflow * last->error_right / total_error;
        (*begin)->error_left -= left_spillover;
        prev->error_right += left_spillover;
        (*std::prev(end))->error_right -= right_spillover;
        next->error_left += right_spillover;
    }
    else if (begin != sequence.begin() && first->error_left > allowed_length_error)
    {
        float overflow = std::min(total_subdiv_error, first->error_left);
        (*begin)->error_left -= overflow;
        prev->error_right += overflow;
        assert(first->error_left > -allowed_length_error);
    }
    else if (end != sequence.end() && last->error_right > allowed_length_error)
    {
        float overflow = std::min(total_subdiv_error, last->error_right);
        last->error_right -= overflow;
        next->error_left += overflow;
        assert(last->error_right > -allowed_length_error);
    }

    // check whether all left over errors are dispersed
    for (auto it = begin; it != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        if (std::abs(node->getSubdivisionError()) > allowed_length_error && node->getTotalError() > allowed_length_error)
        {
            std::cerr << "spillover error for subdivision not handled well! subdiv_err:" << node->getSubdivisionError() << " el:" << node->error_left << " er:" << node->error_right << '\n';
            assert(false);
        }
    }
}

/*!
 * Balance child values such that they account for the minimum value of their recursion level.
 * 
 * Account for errors caused by unbalanced children.
 * Plain subdivision can lead to one child having a smaller value than the density_value associated with the recursion depth
 * if another child has a high enough value such that the parent value will cause subdivision.
 * 
 * In order to compensate for the error incurred, we more error value from the high child to the low child
 * such that the low child has an erroredValue of at least the density_value associated with the recusion depth.
 * 
 * \param node The parent node of the children to balance
 */
void SierpinskiFill::balanceErrors(std::list<SierpinskiFill::SierpinskiTriangle*>::iterator begin, std::list<SierpinskiFill::SierpinskiTriangle*>::iterator end)
{
    // copy sublist to array
    std::vector<SierpinskiFill::SierpinskiTriangle*> nodes;
    for (std::list<SierpinskiFill::SierpinskiTriangle*>::iterator it = begin; it != end; ++it)
    {
        nodes.emplace_back(*it);
    }
    
    std::vector<float> node_error_compensation(nodes.size());

    // sort children on value_error, i.e. sort on total_value
    std::vector<int> order;
    for (int node_idx = 0; node_idx < nodes.size(); node_idx++)
    {
        order.emplace_back(node_idx);
    }
    std::sort(order.begin(), order.end(), [&nodes](int a, int b)
            {
                return nodes[a]->getValueError() < nodes[b]->getValueError();
            });
    
    // add error to children with too low value
    float added = 0;
    int node_order_idx;
    for (node_order_idx = 0; node_order_idx < nodes.size(); node_order_idx++)
    {
        int node_idx = order[node_order_idx];
        SierpinskiTriangle*& node = nodes[node_idx];
        float value_error = node->getValueError();
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
    float subtracted = 0;
    // divide up added among remaining children in ratio to their value error
    float total_remaining_value_error = 0;
    for (int remaining_node_order_idx = node_order_idx; remaining_node_order_idx < nodes.size(); remaining_node_order_idx++)
    {
        int node_idx = order[remaining_node_order_idx];
        SierpinskiTriangle* node = nodes[node_idx];
        float val_err = node->getValueError();
        assert(val_err > -allowed_length_error);
        total_remaining_value_error += val_err;
    }
    
    if (total_remaining_value_error < added - allowed_length_error)
    {
        std::cerr << "total_remaining:" << total_remaining_value_error << " should be > " << added << '\n';
        assert(false);
    }
    
    for (int remaining_node_order_idx = node_order_idx; remaining_node_order_idx < nodes.size(); remaining_node_order_idx++)
    {
        int node_idx = order[remaining_node_order_idx];
        SierpinskiTriangle* node = nodes[node_idx];
        float val_error = node->getValueError();
        assert(val_error > -allowed_length_error);
        
        float diff = added * val_error / total_remaining_value_error;
        subtracted += diff;
        node_error_compensation[node_idx] = -diff;
    }
    if (std::abs(subtracted - added) > allowed_length_error)
    {
        std::cerr << "Redistribution didn't distribute well!! added " << added << " subtracted" << subtracted << '\n';
        assert(false);
    }

    float energy = 0;
    for (int node_idx = 0; node_idx < nodes.size(); node_idx++)
    {
        nodes[node_idx]->error_left -= energy;
        energy += node_error_compensation[node_idx];
        nodes[node_idx]->error_right += energy;
    }
    assert(energy < allowed_length_error);
}


void SierpinskiFill::diffuseError()
{
    int pair_constrained_nodes = 0;
    int constrained_nodes = 0;
    int unconstrained_nodes = 0;
    int subdivided_nodes = 0;
    float error = 0;
    for (std::list<SierpinskiTriangle*>::iterator it = sequence.begin(); it != sequence.end(); ++it)
    {
        SierpinskiTriangle* node = *it;
        SierpinskiTriangle& triangle = *node;

        float boundary = (triangle.realized_length + triangle.total_child_realized_length) * .5f;
        
        float nodal_value = ((use_errors_in_dithering)? node->getErroredValue() : node->requested_length);
        
        float boundary_error = nodal_value - boundary + error;

        std::list<SierpinskiTriangle*>::iterator begin = it;
        std::list<SierpinskiTriangle*>::iterator end = std::next(it);
        if (
            triangle.dir == SierpinskiTriangle::SierpinskiDirection::AC_TO_AB
            && end != sequence.end()
        )
        {
            pair_constrained_nodes++;
            continue; // don't subdivide these two triangles just yet, wait till next iteration
        }
        if (triangle.dir == SierpinskiTriangle::SierpinskiDirection::AB_TO_BC
            && begin != sequence.begin()
        )
        {
            begin = std::prev(it);
            assert((*begin)->depth == triangle.depth || isConstrainedBackward(it));
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
        if (!is_constrained)
            unconstrained_nodes++;
        if (!is_constrained
            && boundary_error >= 0
            && !node->children.empty()
            )
        {
            subdivided_nodes++;
            it = subdivide(begin, end, false);
            if (dithering)
                error += nodal_value - triangle.total_child_realized_length;
        }
        else
        {
            if (dithering)
                error += nodal_value - triangle.realized_length;
        }
    }
    std::cerr << "pair_constrained_nodes: " << pair_constrained_nodes << ", constrained_nodes: " << constrained_nodes << ", unconstrained_nodes: " << unconstrained_nodes << ", subdivided_nodes: " << subdivided_nodes << '\n';
}

bool SierpinskiFill::isConstrainedBackward(std::list<SierpinskiTriangle*>::iterator it)
{
    SierpinskiTriangle* node = *it;
    SierpinskiTriangle* prev = *std::prev(it);
    if (it != sequence.begin() && node->dir == SierpinskiTriangle::SierpinskiDirection::AB_TO_BC && prev->depth < node->depth)
        return true;
    return false;
}
bool SierpinskiFill::isConstrainedForward(std::list<SierpinskiTriangle*>::iterator it)
{
    SierpinskiTriangle* node = *it;
    SierpinskiTriangle* next = *std::next(it);
    if (std::next(it) != sequence.end() && node->dir == SierpinskiTriangle::SierpinskiDirection::AC_TO_AB && next->depth < node->depth)
        return true;
    return false;
}

float SierpinskiFill::getSubdivisionError(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end)
{
    float ret = 0;
    for (auto it = begin; it != end; ++it)
    {
        SierpinskiTriangle* node = *it;
        ret += node->getSubdivisionError();
    }
    return ret;
}




void SierpinskiFill::debugOutput(SVG& svg)
{
    svg.writePolygon(aabb.toPolygon(), SVG::Color::RED);

    // draw triangles
    for (SierpinskiTriangle* node : sequence)
    {
        SierpinskiTriangle& triangle = *node;
        svg.writeLine(triangle.a, triangle.b, SVG::Color::GRAY);
        svg.writeLine(triangle.a, triangle.straight_corner, SVG::Color::GRAY);
        svg.writeLine(triangle.b, triangle.straight_corner, SVG::Color::GRAY);
    }
}


Polygon SierpinskiFill::generateCross() const
{
    Polygon ret;

    for (SierpinskiTriangle* max_level_it : sequence)
    {
        SierpinskiTriangle& triangle = *max_level_it; 
        Point edge_middle = triangle.a + triangle.b + triangle.straight_corner;
        switch(triangle.dir)
        {
            case SierpinskiTriangle::SierpinskiDirection::AB_TO_BC:
                edge_middle -= triangle.a;
                break;
            case SierpinskiTriangle::SierpinskiDirection::AC_TO_AB:
                edge_middle -= triangle.straight_corner;
                break;
            case SierpinskiTriangle::SierpinskiDirection::AC_TO_BC:
                edge_middle -= triangle.a;
                break;
        }
        ret.add(edge_middle / 2);
    }
    
    return ret;
}
Polygon SierpinskiFill::generateSierpinski() const
{
    Polygon ret;

    for (SierpinskiTriangle* max_level_it : sequence)
    {
        SierpinskiTriangle& triangle = *max_level_it; 
        ret.add((triangle.a + triangle.b + triangle.straight_corner) / 3);
    }
    
    return ret;
}


Polygon SierpinskiFill::generateCross(coord_t z, coord_t min_dist_to_side) const
{
    
    Polygon ret;
    
    std::function<Point (int, Edge)> get_edge_crossing_location = [&ret, z, min_dist_to_side](int depth, Edge e)
    {
        coord_t period =  8 << (14 - depth / 2);
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
    for (SierpinskiTriangle* node : sequence)
    {
        SierpinskiTriangle& triangle = *node;
        
        ret.add(get_edge_crossing_location(triangle.depth, triangle.getFromEdge()));
        
        last_triangle = &triangle;
    }
    assert(last_triangle);
    ret.add(get_edge_crossing_location(last_triangle->depth, last_triangle->getToEdge()));
    
    return ret;
}

void SierpinskiFill::debugCheck(bool check_subdivision)
{
    if (sequence.front()->error_left != 0)
    {
        std::cerr << "First node has error left!\n";
        assert(false);
    }
    if (sequence.back()->error_right != 0)
    {
        std::cerr << "Last node has error right!\n";
        assert(false);
    }

    for (auto it = sequence.begin(); it != sequence.end(); ++it)
    {
        if (std::next(it) == sequence.end())
        {
            break;
        }
        SierpinskiTriangle* node = *it;
        SierpinskiTriangle* next = *std::next(it);

        if (std::abs(node->error_right + next->error_left) > allowed_length_error)
        {
            std::cerr << "Consecutive nodes don't have the same error! er: "<< node->error_right << ", nel: "<< next->error_left << "\n";
            assert(false);
        }
        if (check_subdivision && node->getValueError() < -allowed_length_error)
        {
            std::cerr << "Node shouldn't have been subdivided!\n";
            assert(false);
        }
    }
}

}; // namespace cura
