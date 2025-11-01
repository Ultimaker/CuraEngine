// CuraEngine is released under the terms of the AGPLv3 or higher
// 
// This file contains community contributions implementing adaptive layer height algorithms

#include "settings/SlicingAdaptive.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Application.h"
#include "utils/Point3D.h"

// Based on the work of Florens Waserfall (@platch on github)
// and his paper:
// Florens Wasserfall, Norman Hendrich, Jianwei Zhang:
// Adaptive Slicing for the FDM Process Revisited
// 13th IEEE Conference on Automation Science and Engineering (CASE-2017), August 20-23, Xi'an, China. 
// DOI: 10.1109/COASE.2017.8256074

namespace cura
{

coord_t SlicingAdaptive::layer_height_from_slope(const FaceZ& face, double max_surface_deviation)
{
    // Constant error measured as an area of the surface error triangle, with clamping to roughness at 90 degrees.
    // This uses an advanced triangle area error metric approach.
    const double roughness_limit = max_surface_deviation / 0.184;
    const coord_t calculated_height = (face.n_cos > 1e-5) ? 
        coord_t(1.44 * max_surface_deviation * std::sqrt(face.n_sin / face.n_cos) * 1000.0) : // Convert to microns
        std::numeric_limits<coord_t>::max();
    
    return std::min(coord_t(roughness_limit * 1000.0), calculated_height); // Convert to microns
}

void SlicingAdaptive::clear()
{
    m_faces.clear();
}

void SlicingAdaptive::prepare(const MeshGroup* meshgroup)
{
    clear();
    
    // Collect faces from all meshes in the group
    for (const Mesh& mesh : meshgroup->meshes)
    {
        // Skip non-printable meshes
        if (mesh.settings_.get<bool>("infill_mesh") || 
            mesh.settings_.get<bool>("cutting_mesh") || 
            mesh.settings_.get<bool>("anti_overhang_mesh"))
        {
            continue;
        }
        
        for (const MeshFace& face : mesh.faces_)
        {
            const MeshVertex& v0 = mesh.vertices_[face.vertex_index_[0]];
            const MeshVertex& v1 = mesh.vertices_[face.vertex_index_[1]];
            const MeshVertex& v2 = mesh.vertices_[face.vertex_index_[2]];

            const Point3D p0(v0.p_);
            const Point3D p1(v1.p_);
            const Point3D p2(v2.p_);

            // Calculate face normal
            const Point3D n = (p1 - p0).cross(p2 - p0);
            const Point3D normal = n.normalized();
            
            // Calculate z span
            double min_z = std::min({p0.z_, p1.z_, p2.z_});
            double max_z = std::max({p0.z_, p1.z_, p2.z_});
            
            // Store face data
            FaceZ face_data;
            face_data.z_span = {min_z / 1000.0, max_z / 1000.0}; // Convert to mm
            face_data.n_cos = std::abs(normal.z_);
            face_data.n_sin = std::sqrt(normal.x_ * normal.x_ + normal.y_ * normal.y_);
            
            m_faces.push_back(face_data);
        }
    }
    
    // Sort faces by Z span for efficient processing
    std::sort(m_faces.begin(), m_faces.end(), 
        [](const FaceZ& f1, const FaceZ& f2) { 
            return f1.z_span < f2.z_span; 
        });
}

coord_t SlicingAdaptive::next_layer_height(const double print_z, double quality_factor, size_t& current_facet)
{
    coord_t height = m_slicing_params.max_layer_height;
    
    // Calculate max surface deviation based on quality factor
    double max_surface_deviation;
    {
        // Convert layer heights from microns to mm for calculation
        double delta_min = m_slicing_params.min_layer_height / 1000.0;
        double delta_mid = m_slicing_params.layer_height / 1000.0;
        double delta_max = m_slicing_params.max_layer_height / 1000.0;
        
        max_surface_deviation = (quality_factor < 0.5) ?
            lerp(delta_min, delta_mid, 2.0 * quality_factor) :
            lerp(delta_max, delta_mid, 2.0 * (1.0 - quality_factor));
    }
    
    // Find all facets intersecting the slice layer
    size_t ordered_id = current_facet;
    bool first_hit = false;
    
    for (; ordered_id < m_faces.size(); ++ordered_id)
    {
        const std::pair<double, double>& zspan = m_faces[ordered_id].z_span;
        
        // Facet's minimum is higher than slice_z -> end loop
        if (zspan.first >= print_z)
            break;
            
        // Facet's maximum is higher than slice_z -> process this facet
        if (zspan.second > print_z)
        {
            // Mark first event for next iteration
            if (!first_hit)
            {
                first_hit = true;
                current_facet = ordered_id;
            }
            
            // Skip touching facets which could cause small cusp values
            if (zspan.second < print_z + EPSILON)
                continue;
                
            // Compute cusp height for this facet and store minimum
            coord_t facet_height = layer_height_from_slope(m_faces[ordered_id], max_surface_deviation);
            height = std::min(height, facet_height);
        }
    }
    
    // Apply printer capability limits
    height = std::max(height, m_slicing_params.min_layer_height);
    
    // Check for sloped facets inside the determined layer and correct height if necessary
    if (height > m_slicing_params.min_layer_height)
    {
        for (; ordered_id < m_faces.size(); ++ordered_id)
        {
            const std::pair<double, double>& zspan = m_faces[ordered_id].z_span;
            
            // Facet's minimum is higher than slice_z + height -> end loop
            if (zspan.first >= print_z + (height / 1000.0))
                break;
                
            // Skip touching facets
            if (zspan.second < print_z + EPSILON)
                continue;
                
            // Compute cusp height for this facet and check against height
            coord_t reduced_height = layer_height_from_slope(m_faces[ordered_id], max_surface_deviation);
            
            double z_diff = zspan.first - print_z;
            coord_t z_diff_microns = coord_t(z_diff * 1000.0);
            
            if (reduced_height < z_diff_microns)
            {
                // Limit layer height so the offending triangle is just above the new layer
                height = z_diff_microns;
            }
            else if (reduced_height < height)
            {
                height = reduced_height;
            }
        }
        
        // Apply limits again
        height = std::max(height, m_slicing_params.min_layer_height);
    }
    
    return height;
}

coord_t SlicingAdaptive::horizontal_facet_distance(double z)
{
    for (const FaceZ& face : m_faces)
    {
        const std::pair<double, double>& zspan = face.z_span;
        
        // Facet's minimum is higher than max forward distance -> end loop
        if (zspan.first > z + (m_slicing_params.max_layer_height / 1000.0))
            break;
            
        // Horizontal facet (min_z == max_z)
        if (zspan.first > z && std::abs(zspan.first - zspan.second) < EPSILON)
        {
            return coord_t((zspan.first - z) * 1000.0); // Convert to microns
        }
    }
    
    // Return max layer height if no horizontal feature found
    double remaining_height = (m_slicing_params.object_height / 1000.0) - z;
    return coord_t(std::max(remaining_height * 1000.0, double(m_slicing_params.max_layer_height)));
}

} // namespace cura