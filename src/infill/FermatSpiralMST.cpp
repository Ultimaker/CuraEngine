/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#include <cassert>
#include <cmath>
#include <cstdlib>

#include <limits>
#include <algorithm>
#include <vector>

#ifndef NDEBUG
# include <iostream>
#endif // NDEBUG

#include "FermatSpiralMST.h"
#include "math.h"

using namespace cura;

#define DEFAULT_INFILL_WIDTH    300

//
// local data structures and ata
//
struct ConnectionSorter
{
    inline bool operator() (const struct SpiralContourNodeConnection * conn1, const struct SpiralContourNodeConnection * conn2)
    {
        return (conn1->weight < conn2->weight);
    }
};

struct PathIndex {
    uint64_t path_idx;
    uint64_t start_point_idx;
    uint64_t end_point_idx;
};


//
// local functions
//
static bool shouldIncludeCij(const ClipperLib::IntPoint& cij,
                             struct SpiralContourNode *cip1j,
                             struct SpiralContourNode *cip1k);

static void reverseArcList(uint64_t path_count,
                           std::vector<struct Arc *>& result_list,
                           const std::vector<struct Arc *>& original_list);

static void chopOffPathOnPoints(const ClipperLib::Path& original_path,
                                const ClipperLib::IntPoint& new_p1,
                                const ClipperLib::IntPoint& new_p2,
                                int64_t chopoff_p1_idx,
                                int64_t chopoff_p2_idx,
                                ClipperLib::Path& result_path);

static void get_inward_vector(
    double& vec_x,
    double& vec_y,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    const ClipperLib::IntPoint& p3,
    int32_t direction);

static bool get_intersection_on_contour_node(
    ClipperLib::IntPoint& intersection_point,
    uint64_t& path_index,
    uint64_t& start_index,
    uint64_t& end_index,
    double& distance,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    const SpiralContourNode& node);

static bool get_intersection_on_path(
    ClipperLib::IntPoint& intersection_point,
    uint64_t& start_index,
    uint64_t& end_index,
    double& distance,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    const ClipperLib::Path& path,
    bool is_closed_path);

static bool get_shortest_intersection_on_node(
    ClipperLib::IntPoint& result_point,
    struct PathIndex& path_index,
    double& distance,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    const struct SpiralContourNode *node);

static bool travel_along_path_from_point_for_length(
    ClipperLib::IntPoint& result_point,
    uint64_t& result_point_start_index,
    uint64_t& result_point_end_index,
    const ClipperLib::Path& path,
    const ClipperLib::IntPoint& start_point,
    uint64_t start_index,
    double distance,
    bool is_closed_path);

static void chop_child_node(
    struct SpiralContourNode *node,
    uint64_t pin_path_index,
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_path_index,
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout);

static void chop_child_path_open(
    ClipperLib::Path& result_path_former,
    ClipperLib::Path& result_path_later,
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout,
    const ClipperLib::Path& path);

static void chop_child_path_closed(
    ClipperLib::Path& result_path_former,
    ClipperLib::Path& result_path_later,
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout,
    const ClipperLib::Path& path);



SpiralContourTree::SpiralContourTree()
    : m_tree_root(nullptr)
    , m_infill_width(DEFAULT_INFILL_WIDTH)
{
    this->m_contour_node_list.clear();
    this->m_all_node_connection_list.clear();
    this->m_all_contour_node_list.clear();
    this->m_all_arc_list.clear();
}


SpiralContourTree::~SpiralContourTree()
{
    // safely clear everything
    clear();
}

/*
 * Safely clears everything and releases all the allocated memories.
 */
void SpiralContourTree::clear()
{
    m_tree_root = nullptr;

    // clear all connection nodes
    for (auto itr_conn = m_all_node_connection_list.begin(); itr_conn != m_all_node_connection_list.end(); ++itr_conn)
    {
        // delete all arcs
        for (auto itr_arc = (*itr_conn)->arc_list.begin(); itr_arc != (*itr_conn)->arc_list.end(); ++itr_arc)
        {
            delete (*itr_arc);
        }
        delete (*itr_conn);
    }
    m_all_node_connection_list.clear();

    // clear all tree nodes
    for (uint32_t i = 0; i < m_all_contour_node_list.size(); ++i)
    {
        delete m_all_contour_node_list[i];
    }
    m_all_contour_node_list.clear();

    m_all_arc_list.clear();
    m_contour_node_list.clear();
}


void SpiralContourTree::connectContours(struct SpiralContourNode *node)
{
    if (node == nullptr)
    {
        node = this->m_tree_root;
    }

    // if this is not a parent node, handle the children nodes first and then connect
    bool did_connect = false;
    for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
    {
        this->connectContours((*itr_conn)->child_node);
        did_connect = true;
    }

    if (node != this->m_tree_root)
    {
        // connect this node with its parent
        this->formPath(node);
    }
}


static bool get_intersection_on_contour_node(
    ClipperLib::IntPoint& intersection_point,
    uint64_t& path_index,
    uint64_t& start_index,
    uint64_t& end_index,
    double& distance,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    const struct SpiralContourNode *node)
{
    bool found_intersection = false;
    double closest_distance = std::numeric_limits<double>::max();
    ClipperLib::IntPoint closest_point;
    uint64_t closest_path_index = 0;
    uint64_t closest_start_index = 0;
    uint64_t closest_end_index = 0;

    bool is_closed_path = !node->has_been_chopped;
    for (auto itr_path = node->paths.begin(); itr_path != node->paths.end(); ++itr_path)
    {
        double this_distance;
        ClipperLib::IntPoint this_point;
        uint64_t this_start_index = 0;
        uint64_t this_end_index = 0;

        found_intersection |= get_intersection_on_path(
            this_point, this_start_index, this_end_index, this_distance,
            p1, p2, *itr_path, is_closed_path);
        if (found_intersection and this_distance < closest_distance)
        {
            closest_distance = this_distance;
            closest_point = this_point;
            closest_path_index = path_index;
            closest_start_index = this_start_index;
            closest_end_index = this_end_index;
        }
        ++path_index;
    }
}


static bool get_intersection_on_path(
    ClipperLib::IntPoint& intersection_point,
    uint64_t& start_index,
    uint64_t& end_index,
    double& distance,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    const ClipperLib::Path& path,
    bool is_closed_path)
{
    bool found_intersection = false;
    ClipperLib::IntPoint point;
    double closest_distance = std::numeric_limits<double>::max();

    for (uint64_t i = 0; i < path.size(); ++i)
    {
        uint64_t j = i + 1;
        if (j >= path.size())
        {
            j = 0;
        }

        const ClipperLib::IntPoint& p3 = path[i];
        const ClipperLib::IntPoint& p4 = path[j];

        bool intersect = doIntersect(p1, p2, p3, p4);
        if (intersect)
        {
            compute_line_intersection(point, p1, p2, p3, p4);
            const double line_distance = p2p_dist(p1, point);

            if (line_distance < closest_distance)
            {
                closest_distance = line_distance;
                distance = closest_distance;
                intersection_point = point;
                start_index = i;
                end_index = j;
                found_intersection = true;
                break;
            }
        }
    }

    return found_intersection;
}


static void chop_child_node(
    struct SpiralContourNode *node,
    uint64_t pin_path_index,  // information of the inward point
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_path_index,  // information of the outward point
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout)
{
    // the new path will start from the inward point to the outward point
    ClipperLib::Paths new_paths;

    // the inward and outward points will be on a continuous path, so
    // their path indices are the same.
    const ClipperLib::Path& path = node->paths[pin_path_index];

    ClipperLib::Path sub_path_former;
    ClipperLib::Path sub_path_later;

    // chop the path
    if (node->has_been_chopped)
    {
        chop_child_path_open(sub_path_former, sub_path_later,
            pin_start_index, pin_end_index, pin,
            pout_start_index, pout_end_index, pout,
            node->paths[pin_path_index]);
    }
    else
    {
        chop_child_path_closed(sub_path_former, sub_path_later,
            pin_start_index, pin_end_index, pin,
            pout_start_index, pout_end_index, pout,
            node->paths[pin_path_index]);
    }

    // reconstruct the path list
    node->paths.erase(node->paths.begin() + pin_path_index);
    if (!sub_path_later.empty())
    {
        node->paths.insert(node->paths.begin() + pin_path_index, sub_path_later);
    }
    if (!sub_path_former.empty())
    {
        node->paths.insert(node->paths.begin() + pin_path_index, sub_path_former);
    }
    node->has_been_chopped = true;
}

static void chop_child_path_open(
    ClipperLib::Path& result_path_former,
    ClipperLib::Path& result_path_later,
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout,
    const ClipperLib::Path& path)
{
    uint64_t current_index = 0;
    for (current_index = 0; current_index <= pout_start_index; ++current_index)
    {
        result_path_former << path[current_index];
    }
    if (pout != path[pout_start_index])
    {
        result_path_former << pout;
    }

    if (pin != path[pin_end_index])
    {
        result_path_later << pin;
    }
    for (current_index = pin_end_index; current_index < path.size(); ++current_index)
    {
        result_path_later << path[current_index];
    }

    if (result_path_former.size() <= 1)
    {
        result_path_former.clear();
    }
    if (result_path_later.size() <= 1)
    {
        result_path_later.clear();
    }
}


static void chop_child_path_closed(
    ClipperLib::Path& result_path_former,
    ClipperLib::Path& result_path_later,
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout,
    const ClipperLib::Path& path)
{
    // take into account the last ones
    // add pin
    result_path_later << pin;
    if (pin != path[pin_end_index])
    {
        result_path_later << path[pin_end_index];
    }

    // add points in the middle: pin_end_index -> pout_start_index
    if (pin_end_index < pout_start_index)
    {
        for (uint64_t i = pin_end_index + 1; i < pout_start_index; ++i)
        {
            result_path_later << path[i];
        }
    }
    else
    {
        for (uint64_t i = pin_end_index + 1; i < path.size(); ++i)
        {
            result_path_later << path[i];
        }
        for (uint64_t i = 0; i < pout_start_index; ++i)
        {
            result_path_later << path[i];
        }
    }

    // add pout
    if (pout != path[pout_start_index])
    {
        result_path_later << path[pout_start_index];
    }
    result_path_later << pout;
}


void SpiralContourTree::cutParentNodeClosed(
    struct SpiralContourNode *node,
    uint64_t pin_path_index,  // information of the inward point
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_path_index,  // information of the outward point
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout)
{
    ClipperLib::Path new_path;
    const ClipperLib::Path& path = node->paths[pin_path_index];

    new_path << pin;
    if (pin != path[pin_end_index])
    {
        new_path << path[pin_end_index];
    }

    // add points in the middle: pin_end_index -> pout_start_index
    if (pin_end_index < pout_start_index)
    {
        for (uint64_t i = pin_end_index + 1; i < pout_start_index; ++i)
        {
            new_path << path[i];
        }
    }
    else
    {
        for (uint64_t i = pin_end_index + 1; i < path.size(); ++i)
        {
            new_path << path[i];
        }
        for (uint64_t i = 0; i < pout_start_index; ++i)
        {
            new_path << path[i];
        }
    }

    // add pout
    if (pout != path[pout_start_index])
    {
        new_path << path[pout_start_index];
    }
    new_path << pout;

    // set paths
    node->paths.clear();
    node->paths << new_path;
    node->has_been_chopped = true;

    // handle arcs
    bool need_to_remove_arcs = false;
    for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
    {
        struct SpiralContourNodeConnection *connection = *itr_conn;
        for (auto itr_arc = connection->arc_list.begin(); itr_arc != connection->arc_list.end(); ++itr_arc)
        {
            struct Arc *arc = *itr_arc;

            if (arc->is_closed_path)
            {
                // this only happens when the node has not been chopped before
                // so the new arc path will be the same as the new chopped paths
                arc->path_index = 0;
                arc->p1_index = 0;
                arc->p2_index = new_path.size() - 1;
                arc->p1 = new_path[0];
                arc->p2 = new_path[new_path.size() - 1];
                arc->point_count = new_path.size();
                arc->length = compute_path_length(new_path, 0, new_path.size() - 1);
                arc->is_closed_path = false;
            }
            else
            {
                // this is the first time the parent path is being chopped,
                // but this arc is not a closed path. so we need to compare
                // the indices and set the new arc path correctly.
                uint64_t new_p1_idx;
                uint64_t new_p2_idx;
                bool found_new_p1, found_new_p2;
                found_new_p1 = get_point_idx_in_path(new_p1_idx, arc->p1, new_path);
                found_new_p2 = get_point_idx_in_path(new_p2_idx, arc->p2, new_path);

                arc->path_index = 0;

                // if we cannot find p1 on the new path, it means p1 is on the chopped-off path,
                // so we need to move p1 to pout (index 0)
                arc->p1_index = found_new_p1 ? new_p1_idx : 0;
                arc->p1 = new_path[arc->p1_index];

                // if we cannot find p2 on the new path, it means p2 is on the chopped-off path,
                // so we need to move p2 to pin (index size - 1)
                arc->p2_index = found_new_p2 ? new_p2_idx : (new_path.size() - 1);
                arc->p2 = new_path[arc->p2_index];

                if (arc->p1_index >= arc->p2_index or (!found_new_p1 and !found_new_p2))
                {
                    arc->need_to_be_removed = true;
                    connection->arc_list.erase(itr_arc);
                    --itr_arc;
                    need_to_remove_arcs = true;
                    continue;
                }

                // update point_count and length
                arc->point_count = arc->p2_index - arc->p1_index + 1;
                arc->length = compute_path_length(new_path, arc->p1_index, arc->p2_index);
            }
        }
    }

    // remove arcs
    for (auto itr_arc = this->m_all_arc_list.begin(); itr_arc != this->m_all_arc_list.end(); ++itr_arc)
    {
        if ((*itr_arc)->need_to_be_removed)
        {
            this->m_all_arc_list.erase(itr_arc);
            delete *itr_arc;
            --itr_arc;
        }
    }
}


void SpiralContourTree::cutParentNodeOpen(
    struct SpiralContourNode *node,
    uint64_t pin_path_index,  // information of the inward point
    uint64_t pin_start_index,
    uint64_t pin_end_index,
    const ClipperLib::IntPoint& pin,
    uint64_t pout_path_index,  // information of the outward point
    uint64_t pout_start_index,
    uint64_t pout_end_index,
    const ClipperLib::IntPoint& pout)
{
    ClipperLib::Path result_path_former;
    ClipperLib::Path result_path_later;
    uint64_t current_index = 0;

    const ClipperLib::Path path = node->paths[pin_path_index];

    for (current_index = 0; current_index <= pout_start_index; ++current_index)
    {
        result_path_former << path[current_index];
    }
    if (pout != path[pout_start_index])
    {
        result_path_former << pout;
    }

    if (pin != path[pin_end_index])
    {
        result_path_later << pin;
    }
    for (current_index = pin_end_index; current_index < path.size(); ++current_index)
    {
        result_path_later << path[current_index];
    }

    if (result_path_former.size() <= 1)
    {
        result_path_former.clear();
    }
    if (result_path_later.size() <= 1)
    {
        result_path_later.clear();
    }

    // set paths
    node->paths.erase(node->paths.begin() + pin_path_index);
    if (!sub_path_later.empty())
    {
        node->paths.insert(node->paths.begin() + pin_path_index, sub_path_later);
    }
    if (!sub_path_former.empty())
    {
        node->paths.insert(node->paths.begin() + pin_path_index, sub_path_former);
    }
    node->has_been_chopped = true;

    // for convenience. the path indices for the former and later sub-paths.
    const uint64_t former_path_idx = pin_path_index;
    const uint64_t later_path_idx = former_path_idx + 1;

    // handle arcs
    bool need_to_remove_arcs = false;
    for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
    {
        struct SpiralContourNodeConnection *connection = *itr_conn;
        for (auto itr_arc = connection->arc_list.begin(); itr_arc != connection->arc_list.end(); ++itr_arc)
        {
            struct Arc *arc = *itr_arc;
            assert(!arc->is_closed_path);

            // this is the first time the parent path is being chopped,
            // but this arc is not a closed path. so we need to compare
            // the indices and set the new arc path correctly.
            uint64_t new_p1_idx_on_former;
            uint64_t new_p2_idx_on_former;
            bool found_new_p1_on_former, found_new_p2_on_former;
            found_new_p1_on_former = get_point_idx_in_path(new_p1_idx_on_former, arc->p1, result_path_former);
            found_new_p2_on_former = get_point_idx_in_path(new_p2_idx_on_former, arc->p2, result_path_former);

            uint64_t new_p1_idx_on_later;
            uint64_t new_p2_idx_on_later;
            bool found_new_p1_on_later, found_new_p2_on_later;
            found_new_p1_on_later = get_point_idx_in_path(new_p1_idx_on_later, arc->p1, result_path_later);
            found_new_p2_on_later = get_point_idx_in_path(new_p2_idx_on_later, arc->p2, result_path_later);

            // a point must not be found on both former and later paths
            assert(not (found_new_p1_on_former and found_new_p1_on_later));
            assert(not (found_new_p2_on_former and found_new_p2_on_later));
            // a point must a least be found on one side
            assert(found_new_p1_on_former or found_new_p1_on_later);
            assert(found_new_p2_on_former or found_new_p2_on_later);

            if ((found_new_p1_on_former and found_new_p2_on_former)
                or (found_new_p1_on_later and found_new_p2_on_later))
            {
                // both points are on the same side
                uint64_t new_path_idx = 0;
                uint64_t new_p1_idx = 0;
                uint64_t new_p2_idx = 0;
                ClipperLib::Path& arc_on_path;
                if (found_new_p2_on_former)
                {
                    // both on former path
                    new_path_idx = former_path_idx;
                    new_p1_idx = new_p1_idx_on_former;
                    new_p2_idx = new_p2_idx_on_former;
                    arc_on_path = result_path_former;
                }
                else
                {
                    // both on later path
                    new_path_idx = later_path_idx;
                    new_p1_idx = new_p1_idx_on_later;
                    new_p2_idx = new_p2_idx_on_later;
                    arc_on_path = result_path_later;
                }

                assert(new_p1_idx <= new_p2_idx);

                if (new_p1_idx == new_p2_idx)
                {
                    // this arc doesn't exist any more, remove it
                    arc->need_to_be_removed = true;
                    connection->arc_list.erase(itr_arc);
                    --itr_arc;
                    need_to_remove_arcs = true;
                    continue;
                }

                // update arc info
                arc->path_index = new_path_idx;
                arc->p1_index = new_p1_idx;
                arc->p2_index = new_p2_idx;
                arc->point_count = new_p2_idx - new_p1_idx + 1;
                arc->length = compute_path_length(arc_on_path, new_p1_idx, new_p2_idx);
            }
            else
            {
                // p1 must be in the front of p2
                assert(not (found_new_p2_on_former and found_new_p1_on_later));

                // p1 and p2 are on different parts, we need to chop off the arc
                // arc #1: p1 -> pin
                // arc #2: pout -> p2
                // this is parent, so pin comes before pout
                arc->path_index = former_path_idx;
                arc->p1_index = new_p1_idx_on_former;
                arc->p2_index = result_path_former.size() - 1;
                
                // create arc #2
                
            }
        }
    }

    // remove arcs
}


void SpiralContourTree::getInwardOutwardPoints(struct SpiralContourNodeConnection *connection)
{
    const double step_size = 50;  // TODO: make this configurable

    ClipperLib::IntPoint parent_inward_point;
    ClipperLib::IntPoint parent_outward_point;
    uint64_t parent_inward_point_start_idx = 0;
    uint64_t parent_inward_point_end_idx = 0;
    uint64_t parent_outward_point_start_idx = 0;
    uint64_t parent_outward_point_end_idx = 0;

    ClipperLib::IntPoint child_inward_point;
    ClipperLib::IntPoint child_outward_point;
    uint64_t child_inward_point_start_idx = 0;
    uint64_t child_inward_point_end_idx = 0;
    uint64_t child_outward_point_start_idx = 0;
    uint64_t child_outward_point_end_idx = 0;

    struct SpiralContourNode *parent_node = connection->parent_node;
    struct SpiralContourNode *child_node = connection->child_node;
    std::vector<struct Arc *> &arc_list = connection->arc_list;

    bool got_inward_outward_point = false;

    uint64_t arc_idx = 0;
    while (arc_idx < arc_list.size())
    {
        struct Arc *arc = arc_list[arc_idx];

        if (arc->length < m_infill_width)
        {
#ifndef NDEBUG
            std::cout << "arc length is smaller than infill width, skipping this arc."
                << " arc_length = " << arc->length << "; infill_width = " << m_infill_width << std::endl;
#endif // NDEBUG
            continue;
        }

        assert(arc->p1_index != arc->p2_index);

        const ClipperLib::Path& parent_path = parent_node->paths[arc->path_index];

        double remaining_arc_length = arc->length;
        // find the suitable inward and outward points
        for (uint64_t parent_p1_idx = arc->p1_index; parent_p1_idx <= arc->p2_index; ++parent_p1_idx)
        {
            // start from the first point on this arc, travel along the path,
            // and try to find inward and outward points that work 
            ClipperLib::IntPoint current_inward_point = parent_path[parent_p1_idx];

            bool got_inward_outward_point = false;
            while (true)
            {
                uint64_t outward_point_start_index;
                uint64_t outward_point_end_index;
                ClipperLib::IntPoint outward_point;

                // get the outward point on parent
                bool found_point = travel_along_path_from_point_for_length(
                    outward_point, outward_point_start_index, outward_point_end_index,
                    parent_path, current_inward_point, parent_p1_idx, m_infill_width, arc->is_closed_path);
                if (!found_point)
                {
                    // move on to the next arc if we cannot get any more points on this arc to try
                    break;
                }

                // compute inward vector from parent
                double inward_vector_x;
                double inward_vector_y;
                const ClipperLib::IntPoint& p1 = parent_path[(parent_p1_idx == 0) ? (parent_path.size() - 1) : (parent_p1_idx - 1)];
                const ClipperLib::IntPoint& p2 = parent_path[parent_p1_idx];
                const ClipperLib::IntPoint& p3 = parent_path[parent_p1_idx + 1];
                get_inward_vector(inward_vector_x, inward_vector_y, p1, p2, p3, parent_node->direction);

                // get inner point using the inward vector
                ClipperLib::IntPoint inward_p2;
                inward_p2.X = current_inward_point.X + inward_vector_x * 10 * m_infill_width;
                inward_p2.Y = current_inward_point.Y + inward_vector_y * 10 * m_infill_width;

                // get intersection
                double inward_length;
                ClipperLib::IntPoint inward_intersection_point;
                uint64_t inward_intersection_point_path_index;
                uint64_t inward_intersection_point_start_index;
                uint64_t inward_intersection_point_end_index;
                found_point = get_intersection_on_contour_node(
                    inward_intersection_point,
                    inward_intersection_point_path_index, inward_intersection_point_start_index, inward_intersection_point_end_index,
                    inward_length,
                    current_inward_point, inward_p2, child_node);
                if (!found_point)
                {
                    // TODO: move the inward point and continue
                    continue;
                }

                // use the same inward point to compute intersection point as the outward point on child
                ClipperLib::IntPoint outward_p2;
                outward_p2.X = outward_point.X + inward_vector_x * 10 * m_infill_width;
                outward_p2.Y = outward_point.Y + inward_vector_y * 10 * m_infill_width;

                double outward_length;
                ClipperLib::IntPoint outward_intersection_point;
                uint64_t outward_intersection_point_path_index;
                uint64_t outward_intersection_point_start_index;
                uint64_t outward_intersection_point_end_index;
                found_point = get_intersection_on_contour_node(
                    outward_intersection_point,
                    outward_intersection_point_path_index, outward_intersection_point_start_index, outward_intersection_point_end_index,
                    outward_length,
                    outward_point, outward_p2, child_node);
                if (!found_point)
                {
                    // TODO: move the inward point and continue
                    continue;
                }

                // make sure that the points are reasonable
                double longer_link_length = inward_length > outward_length ? inward_length : outward_length;
                double shorter_link_length = inward_length <= outward_length ? inward_length : outward_length;
                if (shorter_link_length * 1.10 >= longer_link_length)
                {
                    std::cout << "link distance difference is too much, skipping" << std::endl;
                    continue;
                }

                // we have found a path, now start connecting
                // TODO: chop off on child path

                // TODO: chop off on parent path (complex)

                connection->processed = true;
            }
        }

        if (connection->processed)
        {
            break;
        }
    }

    assert(connection->processed);
    if (!connection->processed)
    {
        // TODO: show error.
    }
}


void SpiralContourTree::generateFullPath(ClipperLib::Path& full_path, struct SpiralContourNode *node)
{
    // TODO
}


/*
 * Forms a path from the given child node to its parent node.
 */
void SpiralContourTree::formPath(struct SpiralContourNode *child_node)
{
    // TODO
}


void chopOffPathOnPoints(
    const ClipperLib::Path& original_path,
    const ClipperLib::IntPoint& new_p1,
    const ClipperLib::IntPoint& new_p2,
    int64_t chopoff_p1_idx,
    int64_t chopoff_p2_idx,
    ClipperLib::Path& result_path)
{
    ClipperLib::Path new_path;
    int64_t i;

    const ClipperLib::IntPoint& chopoff_p1 = original_path[chopoff_p1_idx];
    const ClipperLib::IntPoint& chopoff_p2 = original_path[chopoff_p2_idx];

    // if the chop off point is on the last edge, we need to create points
    if (chopoff_p1_idx >= chopoff_p2_idx)
    {
        result_path << new_p1;
        if (chopoff_p1 != new_p1)
        {
            result_path << chopoff_p1;
        }
        for (i = chopoff_p1_idx + 1; i < original_path.size(); ++i)
        {
            if (original_path[i].X == new_p2.X and original_path[i].Y == new_p2.Y)
                continue;
            if (original_path[i].X == new_p1.X and original_path[i].Y == new_p1.Y)
                continue;
            result_path << original_path[i];
        }
        for (i = 0; i < chopoff_p2_idx; ++i)
        {
            if (original_path[i].X == new_p2.X and original_path[i].Y == new_p2.Y)
                continue;
            if (original_path[i].X == new_p1.X and original_path[i].Y == new_p1.Y)
                continue;
            result_path << original_path[i];
        }
        if (chopoff_p2 != new_p2)
        {
            result_path << chopoff_p2;
        }
        result_path << new_p2;
    }
    else
    {
        result_path << new_p1;
        if (chopoff_p1 != new_p1)
        {
            result_path << chopoff_p1;
        }

        for (i = chopoff_p1_idx + 1; i < chopoff_p2_idx; ++i)
        {
            result_path << original_path[i];
        }

        if (chopoff_p2 != new_p2)
        {
            result_path << chopoff_p2;
        }
        result_path << new_p2;
    }
}


void SpiralContourTree::constructTree()
{
    if (m_contour_node_list.size() == 0)
    {
#ifdef NDEBUG
        std::cout << "no contour node to process." << std::endl;
#endif // NDEBUG
        return;
    }

#ifdef NDEBUG
    std::cout << "start constructing MST" << std::endl;
#endif // NDEBUG

    // generate connections between nodes
    uint32_t i;
    uint32_t j;
    uint32_t jp;
    uint32_t k;
    for (i = 0; i < m_contour_node_list.size(); ++i)
    {
        // if there is no next level, do nothing
        if (i + 1 >= m_contour_node_list.size())
        {
            break;
        }

        for (j = 0; j < m_contour_node_list[i].size(); ++j)
        {
            struct SpiralContourNode *c_ij = m_contour_node_list[i][j];

            // if there is only one contour, then there will only be one connection
            if (m_contour_node_list[i + 1].size() == 1)
            {
                struct SpiralContourNode *c_ip1_jp = m_contour_node_list[i + 1][0];
                this->computeConnections(c_ij, c_ip1_jp, nullptr);
                continue;
            }

            // get c[i+1,j'] and c[i+1,k]
            for (jp = 0; jp < m_contour_node_list[i + 1].size(); ++jp)
            {
                struct SpiralContourNode *c_ip1_jp = m_contour_node_list[i + 1][jp];
                for (k = 0; k < m_contour_node_list[i + 1].size(); ++k)
                {
                    if (k == jp)
                    {
                        continue;
                    }
                    struct SpiralContourNode *c_ip1_k = m_contour_node_list[i + 1][k];
                    this->computeConnections(c_ij, c_ip1_jp, c_ip1_k);
                }
            }
        }
    }

    // sort the connections based on weight
    std::vector<struct SpiralContourNodeConnection *> sorted_connection_list = m_all_node_connection_list;
    std::sort(sorted_connection_list.begin(), sorted_connection_list.end(), ConnectionSorter());

    // create a minimum spanning tree (MST)
    std::vector<struct SpiralContourNode *> already_connected_node_list;
    already_connected_node_list.reserve(this->m_all_contour_node_list.size());
    uint32_t created_connection_count = 0;

    for (auto itr_conn = sorted_connection_list.begin(); itr_conn != sorted_connection_list.end(); ++itr_conn)
    {
        // if all nodes have been connected, no need to continue
        if (created_connection_count == this->m_all_contour_node_list.size() - 1)
        {
            break;
        }

        struct SpiralContourNode *parent_node = (*itr_conn)->parent_node;
        struct SpiralContourNode *child_node = (*itr_conn)->child_node;

        // make sure we don't create a cyclic link
        bool found_parent = false;
        bool found_child = false;
        for (auto itr_connected_node = already_connected_node_list.begin(); itr_connected_node != already_connected_node_list.end(); ++itr_connected_node)
        {
            if ((*itr_connected_node) == parent_node)
            {
                found_parent = true;
            }
            else if ((*itr_connected_node) == child_node)
            {
                found_child = true;
            }
            if (found_parent and found_child)
            {
                break;
            }
        }
        // make sure we don't create a cyclic link
        if (found_parent and found_child and child_node->parent != nullptr)
        {
            continue;
        }

        // set this connection
        parent_node->to_child_connection_list.push_back(*itr_conn);
        this->updateNodeType(parent_node);
        child_node->to_parent_connection_list.push_back(*itr_conn);
        this->updateNodeType(child_node);
        child_node->parent = parent_node;
        ++created_connection_count;

        if (!found_parent)
        {
            already_connected_node_list.push_back(parent_node);
        }
        if (!found_child)
        {
            already_connected_node_list.push_back(child_node);
        }
    }

    // determines and sets the path directions of all contours
    this->determineContourDirections(this->m_tree_root, 0, false);

#ifndef NDEBUG
    //std::cout << ">>>>>>>>> MST:" << std::endl;
    //this->printMST(this->m_tree_root, 0);
#endif // NDEBUG
}


/*
 * This function is called after the tree is constructed. It uses depth-first search to
 * traverse through all the contours and determines the direction of all contours.
 */
int32_t SpiralContourTree::determineContourDirections(
    struct SpiralContourNode *node,
    int32_t parent_direction,
    bool is_parent_direction_set)
{
    bool direction_need_to_change = false; // whether the direction of this contour needs to be changed
    int32_t new_node_direction = 0; // the new direction if the direction of this contour needs to be changed
    bool is_node_direction_known = false; // whether the direction of this contour is known

    // if the parent's direction is known, we simple use the parent's direction to determine the child's direction.
    if (is_parent_direction_set)
    {
        new_node_direction = -parent_direction;
        is_node_direction_known = true;
        direction_need_to_change = node->direction != new_node_direction;
    }

    struct SpiralContourNodeConnection *connection = nullptr;
    for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
    {
        connection = *itr_conn;
        int32_t child_direction = this->determineContourDirections(connection->child_node, new_node_direction, is_node_direction_known);

        // if the parent's direction has not been determined yet, we determine the direction of this node using the first child's direction
        if (!is_node_direction_known)
        {
            new_node_direction = -child_direction;
            is_node_direction_known = true;
            direction_need_to_change = node->direction != new_node_direction;
        }
    }

    if (!is_node_direction_known)
    {
        new_node_direction = node->direction;
        direction_need_to_change = false;
    }
#ifndef NDEBUG
    assert(new_node_direction != 0);
    if (direction_need_to_change)
    {
        assert(new_node_direction != node->direction);
    }
#endif // NDEBUG

    // reverse the paths and arcs if needed
    if (direction_need_to_change)
    {
        // reverse all paths and set direction
        ClipperLib::Paths reversed_path_list;
        reversed_path_list.reserve(node->paths.size());
        for (auto itr_path = node->paths.begin(); itr_path != node->paths.end(); ++itr_path)
        {
            // reverse all points in each path
            ClipperLib::Path reversed_path;
            reverse_path_direction(reversed_path, *itr_path);
            reversed_path_list << reversed_path;
        }
        node->paths = reversed_path_list;
        node->direction = new_node_direction;

        // we by now have reversed the contour path direction of this node.
        // because the arcs in the connections are ordered according to the parent node's direction,
        // in this case, we need to make sure that all child connections this node has need to be reversed too.
        for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
        {
            connection = *itr_conn;

            std::vector<struct Arc *> reversed_arc_list;
            reverseArcList(node->paths.size(), reversed_arc_list, connection->arc_list);
            connection->arc_list = reversed_arc_list;
        }
    }

    assert(node->direction != 0);
    return node->direction;
}


void SpiralContourTree::addConnectionArc(struct SpiralContourNode *parent_node, struct SpiralContourNode *child_node, struct Arc *arc)
{
    struct SpiralContourNodeConnection *connection = nullptr;

    // find an existing connection object
    for (auto itr_conn = m_all_node_connection_list.begin(); itr_conn != m_all_node_connection_list.end(); ++itr_conn)
    {
        // TODO: check and optimise those checks
        if ((*itr_conn)->parent_node != parent_node)
        {
            continue;
        }
        if ((*itr_conn)->child_node != child_node)
        {
            continue;
        }

        connection = *itr_conn;
        break;
    }

    // create a new connection object if no existing can be found
    if (connection == nullptr)
    {
        connection = new struct SpiralContourNodeConnection;
        connection->parent_node = parent_node;
        connection->child_node = child_node;
        connection->weight = 0;
        connection->processed = false;
        connection->arc_list = std::vector<struct Arc *>();
        this->m_all_node_connection_list.push_back(connection);
    }

    connection->arc_list.push_back(arc);
    connection->weight += arc->point_count;  // weight is the number of points
}


void SpiralContourTree::addNode(struct SpiralContourNode *node, uint32_t level)
{
    // make sure that the node list for this level exists
    while (level >= m_contour_node_list.size())
    {
        m_contour_node_list.push_back(std::vector<struct SpiralContourNode *>());
    }

    // assign level number and node id and add to list
    node->level = level;
    node->index = m_contour_node_list[level].size();

    this->m_contour_node_list[level].push_back(node);
    this->m_all_contour_node_list.push_back(node);
}


void SpiralContourTree::setPolygons(const ClipperLib::Paths& paths)
{
    if (paths.size() == 0)
    {
        return;
    }

    // the first path is the outline, so we can directly feed the whole paths to the
    // handling function and it will generate spiral contour c[0,0] for the outline
    // path automatically.
    this->createNodes(0, paths);

    // set root node
    this->m_tree_root = this->m_contour_node_list[0][0];
}


void SpiralContourTree::updateNodeType(struct SpiralContourNode *node)
{
    uint32_t connection_count = node->to_parent_connection_list.size() + node->to_child_connection_list.size();
    uint32_t type = connection_count <= 2 ? 1 : 2;
    node->type = type;
}


/*
 * Creates child nodes of the given parent node and all child nodes for the child nodes this function has created.
 */
void SpiralContourTree::createNodes(uint32_t current_level, const ClipperLib::Paths& paths)
{
    for (auto itr_path = paths.begin(); itr_path != paths.end(); ++itr_path)
    {
        assert((*itr_path).size() > 1);

        // create a child node for each Polygon Path and continue
        struct SpiralContourNode *child_node = new struct SpiralContourNode;
        memset(child_node, 0, sizeof(struct SpiralContourNode));

        // create the child node
        child_node->paths = ClipperLib::Paths();
        child_node->paths << *itr_path;
        child_node->has_been_chopped = false;
        child_node->direction = compute_path_direction(*itr_path);
        child_node->parent = nullptr;
        child_node->to_child_connection_list = std::vector<struct SpiralContourNodeConnection *>();
        child_node->to_parent_connection_list = std::vector<struct SpiralContourNodeConnection *>();

        this->addNode(child_node, current_level);

        // create child nodes for this node (if any)
        ClipperLib::Paths child_node_paths;

        ClipperLib::ClipperOffset clipper(1.2, 10.0);
        clipper.AddPath(*itr_path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        clipper.MiterLimit = 1.2;
        clipper.Execute(child_node_paths, -this->m_infill_width);

        // create spiral contour tree nodes for this child node
        createNodes(current_level + 1, child_node_paths);
    }
}


struct Arc *SpiralContourTree::createArc(const ClipperLib::Path& path, uint64_t start_index, uint64_t end_index)
{
    assert(start_index != end_index);
    assert(end_index < path.size());

    struct Arc *arc = new struct Arc();
    arc->p1 = path[start_index];
    arc->p2 = path[end_index];
    arc->path_index = 0;
    arc->p1_index = start_index;
    arc->p2_index = end_index;

    arc->point_count = end_index - start_index + 1;
    arc->length = compute_path_length(path, start_index, end_index);

    arc->is_closed_path = end_index == path.size() - 1;
    arc->need_to_be_removed = false;

    assert(arc->p1 != arc->p2);
    assert(arc->point_count >= 2);
    assert(arc->length > 0);
    assert(arc->p1.X >= 0);
    assert(arc->p1.Y >= 0);
    assert(arc->p2.X >= 0);
    assert(arc->p2.Y >= 0);

    this->m_all_arc_list.push_back(arc);
    return arc;
}


void SpiralContourTree::computeConnections(
    struct SpiralContourNode *node_cij,
    struct SpiralContourNode *node_cip1j,
    struct SpiralContourNode *node_cip1k)
{
    bool is_in_nearest_area = false;
    uint64_t created_connection_count = 0;
    uint64_t nearest_area_start_point_index = 0;

    // if there is only one contour on the lower level, just take the whole contour as the nearest area
    if (node_cip1k == nullptr)
    {
        // add arc to the tree
        struct Arc *arc = createArc(node_cij->paths[0], 0, node_cij->paths[0].size() - 1);
        this->addConnectionArc(node_cij, node_cip1j, arc);
        return;
    }

    // take points on this node and compute distance towards the other
    uint64_t current_point_index = 0;
    for (auto itr_pt_cij = node_cij->paths[0].begin(); itr_pt_cij != node_cij->paths[0].end(); ++itr_pt_cij)
    {
        bool has_smallest_dj_prime = shouldIncludeCij(*itr_pt_cij, node_cip1j, node_cip1k);

        if (has_smallest_dj_prime)
        {
            if (!is_in_nearest_area)
            {
                // mark this point as the starting point of the current nearest area.
                is_in_nearest_area = true;
                nearest_area_start_point_index = current_point_index;
            }
        }
        else if (!has_smallest_dj_prime && is_in_nearest_area)
        {
            if (nearest_area_start_point_index < current_point_index - 1)
            {
                // add arc to the tree
                struct Arc *arc = createArc(node_cij->paths[0], nearest_area_start_point_index, current_point_index - 1);
                this->addConnectionArc(node_cij, node_cip1j, arc);
                ++created_connection_count;
            }

            is_in_nearest_area = false;
        }

        ++current_point_index;
    }

    // if there is still an open nearest area, we need to conclude it.
    if (is_in_nearest_area)
    {
        if (nearest_area_start_point_index < current_point_index - 1)
        {
            // add arc to the tree
            struct Arc *arc = createArc(node_cij->paths[0], nearest_area_start_point_index, current_point_index - 1);
            this->addConnectionArc(node_cij, node_cip1j, arc);
            ++created_connection_count;
        }
    }

    assert(created_connection_count > 0);
}


static bool shouldIncludeCij(
    const ClipperLib::IntPoint& cij,
    struct SpiralContourNode *cip1j,
    struct SpiralContourNode *cip1k)
{
    // no other contours on the same level as c[i+1,j].
    if (cip1k == nullptr)
    {
        return true;
    }

    double cip1j_closest_distance;
    double cip1k_closest_distance;

    // FIXME: use different way to calculate this
    bool found_cip1j = calculate_closest_distance_on_path_from_point(cip1j_closest_distance, cij, cip1j->paths[0]);
    bool found_cip1k = calculate_closest_distance_on_path_from_point(cip1k_closest_distance, cij, cip1k->paths[0]);
    if (!found_cip1j)
    {
        return false;
    }
    if (!found_cip1k)
    {
        return true;
    }
    bool has_smallest_dj_prime = cip1j_closest_distance < cip1k_closest_distance;

    return has_smallest_dj_prime;
}


static void reverseArcList(uint64_t path_count, std::vector<struct Arc *>& result_list, const std::vector<struct Arc *>& original_list)
{
    result_list.reserve(original_list.size());
    for (auto itr_arc = original_list.rbegin(); itr_arc != original_list.rend(); ++itr_arc)
    {
        struct Arc *arc = *itr_arc;

        // exchange points
        arc->p1.X ^= arc->p2.X;
        arc->p2.X ^= arc->p1.X;
        arc->p1.X ^= arc->p2.X;

        arc->p1.Y ^= arc->p2.Y;
        arc->p2.Y ^= arc->p1.Y;
        arc->p1.Y ^= arc->p2.Y;

        // exchange point indices
        arc->p1_index ^= arc->p2_index;
        arc->p2_index ^= arc->p1_index;
        arc->p1_index ^= arc->p2_index;

        // correct path index
        arc->path_index = path_count - arc->path_index;

        result_list.push_back(arc);
    }
}


// =======================================================
//     Util functions
// =======================================================

static void get_inward_vector(
    double& vec_x,
    double& vec_y,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    const ClipperLib::IntPoint& p3,
    int32_t direction)
{
    vec_x = ((p2.X - p1.X) + (p3.X - p2.X)) / 2.0;
    vec_y = ((p2.Y - p1.Y) + (p3.Y - p2.Y)) / 2.0;

    const double tmp = vec_x;
    vec_x = vec_y;
    vec_y = tmp;

    vec_x *= direction;
    vec_y *= -direction;
}


static bool get_shortest_intersection_on_node(
    ClipperLib::IntPoint& result_point,
    struct PathIndex& path_index,
    double& distance,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    struct SpiralContourNode *node)
{
    uint64_t path_idx = 0;
    uint64_t start_point_idx = 0;
    uint64_t end_point_idx = 0;
    uint64_t i = 0;
    uint64_t j = 0;
    ClipperLib::IntPoint p3;
    ClipperLib::IntPoint p4;

    double shortest_distance = std::numeric_limits<double>::max();
    bool has_intersection = false;
    for (auto itr_path = node->paths.begin(); itr_path != node->paths.end(); ++itr_path)
    {
        const ClipperLib::Path& current_path = *itr_path;
        i = 0;
        j = 1;
        for (auto itr_pt = current_path.begin(); itr_pt != current_path.end(); ++itr_pt)
        {
            if (j >= current_path.size())
            {
                // only count the last edge if this is a closed path
                if (node->has_been_chopped)
                {
                    break;
                }
                j = 0;
                p3 = *itr_pt;
                p4 = current_path[0];
            }
            else
            {
                p3 = *itr_pt;
                p4 = *(itr_pt + 1);
            }

            // get intersection point
            bool intersect = doIntersect(p1, p2, p3, p4);
            if (!intersect)
            {
                ++i;
                ++j;
                continue;
            }

            // get intersection point and length
            ClipperLib::IntPoint intersection_point;
            compute_line_intersection(intersection_point, p1, p2, p3, p4);
            const double line_distance = p2p_dist(p1, intersection_point);
            if (line_distance < shortest_distance)
            {
                shortest_distance = line_distance;
                start_point_idx = i;
                end_point_idx = j;

                distance = shortest_distance;
                result_point = intersection_point;
                path_index.path_idx = path_idx;
                path_index.start_point_idx = start_point_idx;
                path_index.end_point_idx = end_point_idx;
            }

            ++i;
            ++j;
            has_intersection = true;
        }

        ++path_idx;
    }
    return has_intersection;
}


static bool travel_along_path_from_point_for_length(
    ClipperLib::IntPoint& result_point,
    uint64_t& result_point_start_index,
    uint64_t& result_point_end_index,
    const ClipperLib::Path& path,
    const ClipperLib::IntPoint& start_point,
    uint64_t start_index,
    double distance,
    bool is_closed_path)
{
    double distance_to_travel = distance;
    struct ClipperLib::IntPoint p1 = start_point;
    struct ClipperLib::IntPoint p2 = path[start_index + 1];
    uint64_t current_index = start_index;

    bool found_point = false;
    bool last_try = false;
    while (true)
    {
        double remaining_distance_on_edge = p2p_dist(p1, p2);
        if (remaining_distance_on_edge >= distance_to_travel)
        {
            // get the point
            struct ClipperLib::IntPoint sp1 = path[current_index];
            struct ClipperLib::IntPoint sp2 = path[current_index + 1];
            double vec_x, vec_y;
            compute_unit_vector(vec_x, vec_y, sp1, sp2);

            double p_x = sp1.X + vec_x * remaining_distance_on_edge;
            double p_y = sp1.Y + vec_y * remaining_distance_on_edge;

            result_point.X = std::round(p_x);
            result_point.Y = std::round(p_y);

            result_point_start_index = current_index;
            result_point_start_index = current_index + 1;

            found_point = true;
            break;
        }

        if (last_try)
        {
            break;
        }

        distance_to_travel -= remaining_distance_on_edge;
        ++current_index;

        p1 = path[current_index];
        p2 = path[current_index + 1];
        if (current_index + 1 == path.size())
        {
            if (is_closed_path)
            {
                p2 = path[0];
                last_try = true;
            }
            else
            {
                break;
            }
        }
    }

    return found_point;
}
