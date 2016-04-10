/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURED_MESH_H
#define TEXTURED_MESH_H

#include <vector>
#include <string>

#include "MaterialBase.h"

#include "mesh.h"
#include "utils/intpoint.h"


namespace cura
{

/*!
 * A mesh with a bitmap texture to it.
 */ 
class TexturedMesh : public Mesh 
{
public:
    TexturedMesh(SettingsBaseVirtual* sb);
    /*!
     * Coordinates in texture bitmap 
     */
    struct Coord
    {
        float x, y; //!< Coordinates in texture bitmap
        // 0 to 1
        Coord(float x, float y) //!< constructor
        : x(x)
        , y(y)
        {}
    };
    /*!
     * Coordinates in a specific texture bitmap 
     */
    struct MatCoord
    {
        Coord coords;
        int mat_id; //!< Material id
        MatCoord(Coord coords, int mat_id)
        : coords(coords)
        , mat_id(mat_id)
        {}
    };
    /*!
     * 
     */
    struct FaceTextureCoordIndices
    {
        int index[3]; //!< indices into texture_coords or -1 if no texture data available
        int mat_id; //!< Material id
        FaceTextureCoordIndices(int i1, int i2, int i3, int mat_id)
        : mat_id(mat_id)
        {
            index[0] = i1;
            index[1] = i2;
            index[2] = i3;
        }
    };
    void addTextureCoord(float x, float y);
    void addFace(int vi0, int vi1, int vi2, int ti0, int ti1, int ti2);
    using Mesh::addFace; // otherwise above addFace would shadow the parent addFace

    bool setMaterial(std::string name); //!< set the material to be used in the comming data to be loaded
    Material* addMaterial(std::string name);

    

    virtual void registerFaceSlice(unsigned int face_idx, unsigned int idx_shared, unsigned int idx_first, unsigned int idx_second, int32_t z, Point segment_start, Point segment_end) const;

protected:
    std::vector<Coord> texture_coords;
    std::vector<FaceTextureCoordIndices> face_texture_indices;
    // TODO clean up above lists when super class clear() is called
    // TODO when to clean up below material base?
    MaterialBase material_base;
    /*!
     * Get the material coordinate corresponding to the point on a plane cutting a given edge of the face.
     * \param face_idx The face for which to get the material coord
     * \param z The z of the horizontal plane cutting the face
     * \param p0_idx The index into the first vert of the edge
     * \param p1_idx The index into the second vert of the edge
     * \param result The resulting material Coordinates
     * \return Whether a Material coordinate is defined at the given location
     */
    bool getFaceEdgeMatCoord(unsigned int face_idx, int64_t z, unsigned int p0_idx, unsigned int p1_idx, Coord& result);
private:
    int current_mat; //!< material currently used in loading the face material info
};

} // namespace cura

#endif // TEXTURED_MESH_H