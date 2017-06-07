/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#ifndef CURA_INFILL_FREMATSPIRAL_MST_H
#define CURA_INFILL_FREMATSPIRAL_MST_H

#include <cstdint>

#include "../../libs/clipper/clipper.hpp"
#include "../utils/polygon.h"

/*
 * This file contains the minimum spanning tree logic for the Fermat Spiral infll.
 */
namespace cura {

/*
 * Data structure for the sprial-contour tree.
 * This class represents a node in that tree.
 */
struct SpiralContourNode
{
    uint32_t level;  // i - distance from the outmost boundary
    uint32_t index;  // j - the index number of the contour with the same distance
    ClipperLib::Paths paths;
    bool has_been_chopped;
    int32_t direction;  // positive for clockwise, negative for counter-clockwise
    struct SpiralContourNode *parent;  // parent node of this node
    int32_t type;   // connection type: I or II
    std::vector<struct SpiralContourNodeConnection *> to_child_connection_list; // a list of candidate connections towards a child contour
    std::vector<struct SpiralContourNodeConnection *> to_parent_connection_list; // a list of candidate connections towards a parent contour
};

struct SpiralPointIndex {
    uint64_t path_idx;
    uint64_t point_idx;
};


/*
 * Represents an arc from p1 to p2.
 * Note that the edge is undirected.
 */
struct Arc
{
    ClipperLib::IntPoint p1;
    ClipperLib::IntPoint p2;
    uint64_t path_index;
    uint64_t p1_index;
    uint64_t p2_index;
    int64_t point_count;
    double length;
    bool is_closed_path;
    bool need_to_be_removed;
};


/*
 * Represents all edges between the parent node and the child node.
 * Note that the edge is undirected. 'parent' and 'child' are merely for better understanding.
 */
struct SpiralContourNodeConnection
{
    struct SpiralContourNode *parent_node;
    struct SpiralContourNode *child_node;
    int64_t weight;
    std::vector<struct Arc *> arc_list;
    bool processed;

    // the following represent the established connection points
    ClipperLib::IntPoint inward_point_on_parent;
    ClipperLib::IntPoint outward_point_on_parent;
    ClipperLib::IntPoint inward_point_on_child;
    ClipperLib::IntPoint outward_point_on_child;
};


/*
 * This class represents a sprial-contour tree. It takes a polygon object and generates a
 * minimum spanning tree (MST) based on 
 */
class SpiralContourTree
{
public:
    SpiralContourTree();
    ~SpiralContourTree();

    /*
     * Safely clears everything and releases all the allocated memories.
     */
    void clear();

    void setInfillWidth(int64_t infill_width);

    /*
     * Initialises the spiral-contour tree to process the given polygon paths.
     * Note that the given polygon paths must belong to a single object.
     */
    void setPolygons(const ClipperLib::Paths& paths);

    /*
     * Starts constructing a spiral-contour tree using all the added nodes and edges.
     * This function creates a minimum spanning tree (MST).
     */
    void constructTree();

    /*
     * Connects all the contours by generating inward and outward points.
     * This must be called after the MST is constructed because it uses the MST to
     * create connections.
     */
    void connectContours(struct SpiralContourNode *node = nullptr);

    void generateFullPath(ClipperLib::Path& full_path, struct SpiralContourNode *node = nullptr);

    const std::vector<struct SpiralContourNode *>& getAllContourNodeList() const {
        return this->m_all_contour_node_list;
    }

private:
    void cutParentNodeClosed(
        struct SpiralContourNode *node,
        uint64_t pin_path_index,  // information of the inward point
        uint64_t pin_start_index,
        uint64_t pin_end_index,
        const ClipperLib::IntPoint& pin,
        uint64_t pout_path_index,  // information of the outward point
        uint64_t pout_start_index,
        uint64_t pout_end_index,
        const ClipperLib::IntPoint& pout);

    void cutParentNodeOpen(
        struct SpiralContourNode *node,
        uint64_t pin_path_index,  // information of the inward point
        uint64_t pin_start_index,
        uint64_t pin_end_index,
        const ClipperLib::IntPoint& pin,
        uint64_t pout_path_index,  // information of the outward point
        uint64_t pout_start_index,
        uint64_t pout_end_index,
        const ClipperLib::IntPoint& pout);

    void getInwardOutwardPoints(struct SpiralContourNodeConnection *connection);

    /*
     * generate a path from this parent to all its child nodes.
     */
    void formPath(struct SpiralContourNode *parent_node);

    /*
     * This function is called after the tree is constructed. It uses depth-first search to
     * traverse through all the contours and determines the direction of all contours.
     */
    int32_t determineContourDirections(struct SpiralContourNode *node,
                                       int32_t parent_direction,
                                       bool is_parent_direction_set);

    struct Arc *createArc(const ClipperLib::Path& path, uint64_t start_index, uint64_t end_index);

    /*
     * Adds the given SpiralContourNode into the tree node list and assigns
     * the given level number and an auto-generated index number for it.
     * 
     * param node  - The SpiralContourNode to be added.
     * param level - The level number this node belongs to.
     */
    void addNode(struct SpiralContourNode *node, uint32_t level);

    /*
     * Creates child nodes of the given parent node and all child nodes for the child nodes this function has created.
     */
    void createNodes(uint32_t current_level, const ClipperLib::Paths& paths);

    void updateNodeType(struct SpiralContourNode *node);

    void addConnectionArc(struct SpiralContourNode *parent_node,
                          struct SpiralContourNode *child_node,
                          struct Arc *arc);

    void computeConnections(struct SpiralContourNode *node_cij,
                            struct SpiralContourNode *node_cip1j,
                            struct SpiralContourNode *node_cip1k);

private:
    std::vector<std::vector<struct SpiralContourNode *>> m_contour_node_list;  // indexed by [level, index]
    std::vector<struct SpiralContourNode *>           m_all_contour_node_list; // a flat list of all spiral contour nodes
    std::vector<struct SpiralContourNodeConnection *> m_all_node_connection_list;  // a flat list of all connections between all spiral contour nodes
    std::vector<struct Arc *>                         m_all_arc_list;
    struct SpiralContourNode *m_tree_root;
    int64_t m_infill_width;
};

void inline cura::SpiralContourTree::setInfillWidth(int64_t infill_width)
{
    this->m_infill_width = infill_width;
}


} // namespace cura

#endif // CURA_INFILL_FREMATSPIRAL_MST_H
