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
            
            if (node->depth == max_depth)
                continue;
            if (
                triangle.getSubdivisionError() >= 0
                && (!isConstrainedForward(it) && !isConstrainedBackward(it))
                )
            {
                subdivide(it, true);
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
            SierpinskiTriangle*& node = *it;
            depth_ordered[node->depth].emplace_back(it);
        }
    }
    
    bool redistributed_anything = false;
    
    for (int depth = max_depth; depth >= 0; depth--)
    {
        std::list<std::list<SierpinskiTriangle*>::iterator>& depth_nodes = depth_ordered[depth];
        for (std::list<SierpinskiTriangle*>::iterator it : depth_nodes)
        {
                
            SierpinskiTriangle*& node = *it;
            SierpinskiTriangle& triangle = *node;
            
            float unresolvable_error = triangle.getValueError(); // node.getSubdivisionError();
            
            
            if (
                unresolvable_error > .5 &&
                (isConstrainedForward(it) || isConstrainedBackward(it))
                )
            {
                if (isConstrainedForward(it) && isConstrainedBackward(it))
                {
                    // when constrained in both directions, then divide error equally
                    unresolvable_error *= .5;
                    // disperse half of the error forward and the other half backward
                }
                if (isConstrainedForward(it))
                {
                    SierpinskiTriangle*& next = *std::next(it);
                    node->error_right -= unresolvable_error;
                    next->error_left += unresolvable_error;
                }
                if (isConstrainedBackward(it))
                {
                    SierpinskiTriangle*& prev = *std::prev(it);
                    node->error_left -= unresolvable_error;
                    prev->error_right += unresolvable_error;
                }
                redistributed_anything = true;
            }
        }
    }
    return redistributed_anything;
}


/*!
 * Subdivide a node into its children.
 * Redistribute leftover errors needed for this subdivision and account for errors needed to keep the children balanced.
 * 
 * \param it iterator to the node to subdivide
 * \param redistribute_errors Whether to redistribute the accumulated errors to neighboring nodes and/or among children
 * \return The last child, so that we can iterate further through the sequence on the input iterator.
 */
std::list<SierpinskiFill::SierpinskiTriangle*>::iterator SierpinskiFill::subdivide(std::list<SierpinskiTriangle*>::iterator it, bool redistribute_errors)
{
    SierpinskiTriangle*& node = *it;
    
    assert(!node->children.empty() && "cannot subdivide node with no children!");
    
    if (redistribute_errors)
    { // move left-over errors
        redistributeLeftoverErrors(it);
    }
    
    std::list<SierpinskiFill::SierpinskiTriangle*>::iterator first_child_it = std::prev(it);
    // the actual subdivision
    for (SierpinskiTriangle& child : node->children)
    {
        sequence.insert(it, &child);
    }
    first_child_it++;
    
    std::list<SierpinskiTriangle*>::iterator last_child_it = std::prev(it);
    sequence.erase(it);

    
    if (redistribute_errors)
    { // make positive errors in children well balanced
        // Pass along error from parent
        node->children.front().error_left += node->error_left;
        node->children.back().error_right += node->error_right;
        
        balanceErrors(first_child_it, std::next(last_child_it));
    }
    
    return last_child_it;
}


/*!
 * Redistribute positive errors in as much as they aren't needed to subdivide this node.
 * If this node has received too much positive error then it will subdivide
 * and pass along the error from whence it came.
 * 
 * This is called just before performing a subdivision.
 */
void SierpinskiFill::redistributeLeftoverErrors(std::list<SierpinskiTriangle*>::iterator it)
{
    SierpinskiTriangle*& node = *it;
    SierpinskiTriangle*& prev = *std::prev(it);
    SierpinskiTriangle*& next = *std::next(it);
    
    float subdiv_error = node->getSubdivisionError();
    if (subdiv_error < .5)
    { // there is no left-over error
        if (subdiv_error < -.5)
        {
            std::cerr << "redistributeLeftoverErrors shouldn't be called if the node isn't to be subdivided.\n";
            assert(false);
        }
        return;
    }
    if (it != sequence.begin() && std::next(it) != sequence.end() && node->error_left > .5 && node->error_right > .5)
    {
        float total_error = node->error_left + node->error_right;
        float overflow = std::min(subdiv_error, total_error);
        float left_spillover = overflow * node->error_left / total_error;
        float right_spillover = overflow * node->error_right / total_error;
        node->error_left -= left_spillover;
        prev->error_right += left_spillover;
        node->error_right -= right_spillover;
        next->error_left += right_spillover;
        if (std::abs(node->getSubdivisionError()) > .1 && node->getTotalError() > .1)
        {
            std::cerr << "spillover error for subdivision not handled well! subdiv_err:" << node->getSubdivisionError() << " el:" << node->error_left << " er:" << node->error_right <<'\n';
            assert(false);
        }
    }
    else if (it != sequence.begin() && node->error_left > .5)
    {
        float overflow = std::min(subdiv_error, node->error_left);
        node->error_left -= overflow;
        prev->error_right += overflow;
        assert(node->error_left > -.1);
        if (std::abs(node->getSubdivisionError()) > .5 && node->getTotalError() > .5)
        {
            std::cerr << "spillover error for subdivision not handled well! subdiv_err:" << node->getSubdivisionError() << " el:" << node->error_left << " er:" << node->error_right << '\n';
            assert(false);
        }
    }
    else if (std::next(it) != sequence.end() && node->error_right > .5)
    {				
        float overflow = std::min(subdiv_error, node->error_right);
        node->error_right -= overflow;
        next->error_left += overflow;
        assert(node->error_right > -.1);
        if (std::abs(node->getSubdivisionError()) > .5 && node->getTotalError() > .5)
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
            if (node_idx >= nodes.size() / 2)
                node->error_left -= value_error;
            else
                node->error_right -= value_error;
            // it doesn't matter whether this diff gets added to left or right,
            //   because when we subdivide this child again the error is used up
            //   in order to reach teh minimum value of each cell
            // However, we keep the error on the most logical side to maintain equality and so that we can have more assertions
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
        SierpinskiTriangle*& node = nodes[node_idx];
        float val_err = node->getValueError();
        assert(val_err > -.1);
        total_remaining_value_error += val_err;
    }
    
    if (total_remaining_value_error < added - .5)
    {
        std::cerr << "total_remaining:" << total_remaining_value_error << " should be > " << added << '\n';
        assert(false);
    }
    
    for (int remaining_node_order_idx = node_order_idx; remaining_node_order_idx < nodes.size(); remaining_node_order_idx++)
    {
        int node_idx = order[remaining_node_order_idx];
        SierpinskiTriangle*& node = nodes[node_idx];
        float val_error = node->getValueError();
        assert(val_error > -.1);
        
        float diff = added * val_error / total_remaining_value_error;
        subtracted += diff;
        if (node_idx >= nodes.size() / 2)
            node->error_left -= diff;
        else
            node->error_right -= diff;
    }
    if (std::abs(subtracted - added) > .5)
    {
        std::cerr << "Redistribution didn't distribute well!! added " << added << " subtracted" << subtracted << '\n';
        assert(false);
    }
}


void SierpinskiFill::diffuseError()
{
    float error = 0;
    for (std::list<SierpinskiTriangle*>::iterator it = sequence.begin(); it != sequence.end(); ++it)
    {
        SierpinskiTriangle*& node = *it;
        SierpinskiTriangle& triangle = *node;
        
        float boundary = (triangle.realized_length + triangle.total_child_realized_length) * .5f;
        
        float nodal_value = ((use_errors_in_dithering)? node->getErroredValue() : node->requested_length);
        
        float boundary_error = boundary - nodal_value + error;

        bool do_subdivision = boundary_error <= 0;
        if (do_subdivision)
        {
            SierpinskiTriangle* prev = *std::prev(it);
            if (it != sequence.begin() && prev->depth < node->depth)
            {
                do_subdivision = false;
            }
            SierpinskiTriangle* next = *std::next(it);
            if (std::next(it) != sequence.end() && next->depth < node->depth)
            {
                do_subdivision = false;
            }
        }
        if (do_subdivision
//             && ! node.children.isEmpty()
            
            )
        {
            it = subdivide(it, false);
            if (dithering)
                error += triangle.total_child_realized_length - nodal_value;
        }
        else
        {
            if (dithering)
                error += triangle.realized_length - nodal_value;
        }
    }
}

bool SierpinskiFill::isConstrainedBackward(std::list<SierpinskiTriangle*>::iterator it)
{
    SierpinskiTriangle* node = *it;
    if (it != sequence.begin() && (*std::prev(it))->depth < node->depth)
        return true;
    return false;
}
bool SierpinskiFill::isConstrainedForward(std::list<SierpinskiTriangle*>::iterator it)
{
    SierpinskiTriangle* node = *it;
    if (std::next(it) != sequence.end() && (*std::next(it))->depth < node->depth)
        return true;
    return false;
}






void SierpinskiFill::debugOutput(SVG& svg)
{
    svg.writePolygon(aabb.toPolygon(), SVG::Color::RED);
    std::cerr << "SierpinskiFill::debugOutput hasn't been reimplementen fully yet \n"; // TODO
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
//     for (SierpinskiTriangle* max_level_it = pre_division_tree.begin(max_depth); max_level_it != pre_division_tree.end(max_depth); ++max_level_it)
    for (SierpinskiTriangle* max_level_it : sequence)
    {
        SierpinskiTriangle& triangle = *max_level_it; 
        
        ret.add(get_edge_crossing_location(triangle.depth, triangle.getFromEdge()));
        
        last_triangle = &triangle;
    }
    assert(last_triangle);
    ret.add(get_edge_crossing_location(last_triangle->depth, last_triangle->getToEdge()));
    
    return ret;
}


}; // namespace cura
