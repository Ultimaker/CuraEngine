// CuraEngine is released under the terms of the AGPLv3 or higher
//
// This file contains community contributions implementing adaptive layer height algorithms

#ifndef SLICINGADAPTIVE_H
#define SLICINGADAPTIVE_H

#include <utility>
#include <vector>

#include "MeshGroup.h"
#include "utils/Coord_t.h"

namespace cura
{

struct SlicingParameters
{
    coord_t min_layer_height;
    coord_t max_layer_height;
    coord_t layer_height;
    coord_t object_height;
};

class SlicingAdaptive
{
public:
    struct FaceZ
    {
        std::pair<double, double> z_span; // min_z, max_z of the face
        double n_cos; // abs(normal.z)
        double n_sin; // sqrt(normal.x^2 + normal.y^2)
    };

    void clear();
    void set_slicing_parameters(const SlicingParameters& params)
    {
        m_slicing_params = params;
    }
    void prepare(const MeshGroup* meshgroup);

    /*!
     * Calculate next layer height based on quality factor
     * \param print_z The Z position of the top of the previous layer
     * \param quality_factor Quality factor from 0 (best quality) to 1 (fastest speed)
     * \param current_facet In/out parameter tracking the current face index
     * \return The optimal layer height for the next layer
     */
    coord_t next_layer_height(const double print_z, double quality_factor, size_t& current_facet);

    /*!
     * Get distance to next horizontal facet for detecting features
     * \param z Current Z position
     * \return Distance to next horizontal feature
     */
    coord_t horizontal_facet_distance(double z);

private:
    SlicingParameters m_slicing_params;
    std::vector<FaceZ> m_faces;

    /*!
     * Calculate layer height from face slope and surface deviation
     * \param face The face to calculate for
     * \param max_surface_deviation Maximum allowed surface deviation
     * \return Calculated layer height
     */
    static coord_t layer_height_from_slope(const FaceZ& face, double max_surface_deviation);

    /*!
     * Linear interpolation helper
     */
    static double lerp(double a, double b, double t)
    {
        return a + t * (b - a);
    }

    static constexpr double EPSILON = 1e-6;
};

} // namespace cura

#endif // SLICINGADAPTIVE_H