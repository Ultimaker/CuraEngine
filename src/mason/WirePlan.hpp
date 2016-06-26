/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_WIRE_PLAN_HPP
#define INCLUDED_MASON_WIRE_PLAN_HPP

#include <vector>

#include "../utils/intpoint.h"

namespace cura {
namespace mason {

/** \brief A wire represents a volume that should be filled.
 *
 * A wire represents a volume that should be filled by the tool head.
 * It covers the volume described by the line from \ref pt0 to \ref pt1
 * extruded in the z direction by \ref height and enlarged in the x/y
 * directions by \ref width.  Currently, \ref pt0 and \ref pt1 will have
 * the same z value in which case the shape of the wire is a box with a
 * cylindrical cap on each end.  The order of pt0 and pt1 does not imply
 * a fill direction.
 */
struct Wire {
public:
    Point3 pt0;
    Point3 pt1;
    int_coord_t width;
    int_coord_t height;
};

struct WireLayer {
public:
    /** \brief z of top of layer. */
    int_coord_t z;
    std::vector<Wire> wires;
};

/** \brief Representation of printing plan as wires that should be filled.
 */
class WirePlan {
public:
    void addWire(const Wire &wire);
    
    size_t numLayers() const;
    const WireLayer &getLayer(size_t layer_idx) const;
    
private:
    /** \brief Layers indexed by top z coordinate.
     * 
     * The vector index corresponds to the fine slice level in the VolumeStore.
     */
    std::vector<WireLayer> m_layers;
};

}
}

#endif
