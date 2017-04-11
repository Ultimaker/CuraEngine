/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_AREA_TEXTURE_PROCESSOR_H
#define TEXTURE_PROCESSING_AREA_TEXTURE_PROCESSOR_H

// #include <unordered_map>

#include "../settings/settings.h"
#include "../sliceDataStorage.h"
#include "../utils/RStarTree2D.h"

#include "TexturedMesh.h"
#include "MatCoord.h"
#include "FaceNormalStorage.h"

namespace cura
{

/*!
 */
class AreaTextureProcessor
{
public:
    /*!
     * \param mesh Where to get the texture data from
     * \param mesh_storage Where to get the layer z info from
     */
    AreaTextureProcessor(const TexturedMesh& mesh, unsigned int layer_count);

    std::optional<float> getTextureColor(int layer_nr, Point location, ColourUsage color) const;
private:
    coord_t layer_height;
    coord_t initial_slice_z;
    struct TexturedVertex
    {
        Point3 p; //!< location of the vertex
        MatCoord uv; //!< The UV coordinates
    };
    struct TexturedFace
    {
        TexturedVertex verts[3]; //!< The vertices
    };
    std::vector<RStarTree2D<TexturedFace>> layer_to_texture_faces;
    FaceNormalStorage face_normals;

    void registerTexturedFace(const TexturedFace& face);

    static TexturedVertex getFaceUV(const TexturedFace& face, Point location);

    static Point toPoint(const Point3& p);
};

} // namespace cura

#endif // TEXTURE_PROCESSING_AREA_TEXTURE_PROCESSOR_H