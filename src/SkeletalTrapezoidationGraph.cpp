#include "SkeletalTrapezoidationGraph.h"
#include <unordered_map>

#include "utils/linearAlg2D.h"

namespace arachne
{

void SkeletalTrapezoidationGraph::fixNodeDuplication()
{
    for (auto node_it = nodes.begin(); node_it != nodes.end();)
    {
        node_t* replacing_node = nullptr;
        for (edge_t* outgoing = node_it->some_edge; outgoing != node_it->some_edge; outgoing = outgoing->twin->next)
        {
            assert(outgoing);
            if (outgoing->from != &*node_it)
            {
                replacing_node = outgoing->from;
            }
            if (outgoing->twin->to != &*node_it)
            {
                replacing_node = outgoing->twin->to;
            }
        }
        if (replacing_node)
        {
            for (edge_t* outgoing = node_it->some_edge; outgoing != node_it->some_edge; outgoing = outgoing->twin->next)
            {
                outgoing->twin->to = replacing_node;
                outgoing->from = replacing_node;
            }
            node_it = nodes.erase(node_it);
        }
        else
        {
            ++node_it;
        }
    }
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
        if (current_edge_it != edges.end()
            && to_be_removed == &*current_edge_it)
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
        return shorterThen(a->p - b->p, snap_dist); 
    };
        
    for (auto edge_it = edges.begin(); edge_it != edges.end();)
    {
        if (edge_it->prev)
        {
            edge_it++;
            continue;
        }
        
        edge_t* quad_start = &*edge_it;
        edge_t* quad_end = quad_start; while (quad_end->next) quad_end = quad_end->next;
        edge_t* quad_mid = (quad_start->next == quad_end)? nullptr : quad_start->next;

        bool edge_it_is_updated = false;
        bool quad_mid_is_removed = false;
        if (quad_mid && should_collapse(quad_mid->from, quad_mid->to))
        {
            assert(quad_mid->twin);
            int count = 0;
            for (edge_t* edge_from_3 = quad_end; edge_from_3 && edge_from_3 != quad_mid->twin; edge_from_3 = edge_from_3->twin->next)
            {
                edge_from_3->from = quad_mid->from;
                edge_from_3->twin->to = quad_mid->from;
                if (count > 50)
                {
                    std::cerr << edge_from_3->from->p << " - " << edge_from_3->to->p << '\n';
                }
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
            if (quad_mid->from->some_edge == quad_mid)
            {
                if (quad_mid->twin->next)
                {
                    quad_mid->from->some_edge = quad_mid->twin->next;
                }
                else
                {
                    quad_mid->from->some_edge = quad_mid->prev->twin;
                }
            }
            
            nodes.erase(node_locator[quad_mid->to]);

            quad_mid->prev->next = quad_mid->next;
            quad_mid->next->prev = quad_mid->prev;
            quad_mid->twin->next->prev = quad_mid->twin->prev;
            quad_mid->twin->prev->next = quad_mid->twin->next;

            safelyRemoveEdge(quad_mid->twin, edge_it, edge_it_is_updated);
            safelyRemoveEdge(quad_mid, edge_it, edge_it_is_updated);
            quad_mid_is_removed = true;
        }

        //  o-o
        //  | | > collapse sides
        //  o o
        if ( should_collapse(quad_start->from, quad_end->to) && should_collapse(quad_start->to, quad_end->from))
        { // Collapse start and end edges and remove whole cell
            assert(!quad_mid || quad_mid_is_removed);

            quad_start->twin->to = quad_end->to;
            quad_end->to->some_edge = quad_end->twin;
            if (quad_end->from->some_edge == quad_end)
            {
                if (quad_end->twin->next)
                {
                    quad_end->from->some_edge = quad_end->twin->next;
                }
                else
                {
                    quad_end->from->some_edge = quad_end->prev->twin;
                }
            }
            nodes.erase(node_locator[quad_start->from]);

            quad_start->twin->twin = quad_end->twin;
            quad_end->twin->twin = quad_start->twin;
            safelyRemoveEdge(quad_start, edge_it, edge_it_is_updated);
            safelyRemoveEdge(quad_end, edge_it, edge_it_is_updated);
        }
        // If only one side had collapsable length then the cell on the other side of that edge has to collapse
        // if we would collapse that one edge then that would change the quad_start and/or quad_end of neighboring cells
        // this is to do with the constraint that !prev == !twin.next

        if (!edge_it_is_updated)
        {
            edge_it++;
        }
    }
}

void SkeletalTrapezoidationGraph::makeRib(edge_t*& prev_edge, Point start_source_point, Point end_source_point, bool is_next_to_start_or_end)
{
    Point p = LinearAlg2D::getClosestOnLineSegment(prev_edge->to->p, start_source_point, end_source_point);
    coord_t dist = vSize(prev_edge->to->p - p);
    prev_edge->to->data.distance_to_boundary = dist;
    assert(dist >= 0);

    nodes.emplace_front(SkeletalTrapezoidationJoint(), p);
    node_t* node = &nodes.front();
    node->data.distance_to_boundary = 0;
    
    edges.emplace_front(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EXTRA_VD));
    edge_t* forth_edge = &edges.front();
    edges.emplace_front(SkeletalTrapezoidationEdge(SkeletalTrapezoidationEdge::EXTRA_VD));
    edge_t* back_edge = &edges.front();
    
    prev_edge->next = forth_edge;
    forth_edge->prev = prev_edge;
    forth_edge->from = prev_edge->to;
    forth_edge->to = node;
    forth_edge->twin = back_edge;
    back_edge->twin = forth_edge;
    back_edge->from = node;
    back_edge->to = prev_edge->to;
    node->some_edge = back_edge;
    
    prev_edge = back_edge;
}

}
