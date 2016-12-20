#include "TextureProximityProcessor.h"

#include <algorithm> // swap
#include <functional> // function

#include "../utils/optional.h"
#include "../utils/linearAlg2D.h"
#include "../slicer/SlicerSegment.h"

namespace cura 
{

TextureProximityProcessor::TextureProximityProcessor(const TextureProximityProcessor::Settings settings, unsigned int slice_layer_count)
: settings(settings)
{
    loc_to_slice.resize(slice_layer_count, SparseLineGrid<TexturedFaceSlice, TexturedFaceSliceLocator>(settings.proximity));
}


void TextureProximityProcessor::registerTexturedFaceSlice(SlicerSegment face_segment, MatSegment texture_segment, unsigned int layer_nr)
{
    TexturedFaceSlice slice{face_segment, texture_segment};
    assert((int)layer_nr >= 0 && layer_nr < loc_to_slice.size());
    loc_to_slice[layer_nr].insert(slice);
}

float TextureProximityProcessor::getColor(const Point location, const unsigned int layer_nr, ColourUsage color, float default_color)
{
    assert((int)layer_nr >= 0 && layer_nr < loc_to_slice.size());
    SparseLineGrid<TexturedFaceSlice, TexturedFaceSliceLocator> grid = loc_to_slice[layer_nr];

    coord_t best_dist2 = std::numeric_limits<coord_t>::max();
    std::optional<TexturedFaceSlice> best;
    std::function<bool (const TexturedFaceSlice& in)> process_func = [location, &best_dist2, &best](const TexturedFaceSlice& in)
    {
        coord_t dist2 = LinearAlg2D::getDist2FromLineSegment(location, in.face_segment.start, in.face_segment.end);
        if (dist2 < best_dist2)
        {
            best_dist2 = dist2;
            best = in;
        }
        return true; // keep going, we're not sure whether we have found the best yet
    };
    
    grid.processNearby(location, settings.proximity, process_func);
    
    if (best_dist2 > settings.proximity * settings.proximity * 4)
    {
        return default_color;
    }
    assert(best && "given that dist2 != max int this variable should have been innitialized");
    const Point p0 = best->face_segment.start;
    const Point p1 = best->face_segment.end;
    const Point x = location;
    // Point r = resulting point on the nearest segment, nearest to [location]
    const MatSegment mat_segment = best->mat_segment;

    const Point v01 = p1 - p0;
    const Point v0x = x - p0;
    const coord_t v01_length2 = vSize2(v01);
    if (v01_length2 <= 4)
    {
        return mat_segment.start.getColor(color);
    }

    const coord_t dot_prod = dot(v0x, v01);
    const int64_t v0r_length2 = dot_prod * dot_prod / v01_length2;
    if (v0r_length2 <= 0)
    {
        return mat_segment.start.getColor(color);
    }
    if (v0r_length2 >= v01_length2)
    {
        return mat_segment.end.getColor(color);
    }
    const coord_t v0r_length = sqrt(v0r_length2);
    const coord_t v01_length = sqrt(v01_length2);
    MatCoord mat_in_between = mat_segment.start;
    mat_in_between.coords = mat_segment.start.coords + (mat_segment.end.coords - mat_segment.start.coords) * v0r_length / v01_length;
    return mat_in_between.getColor(color);
}



}//namespace cura
