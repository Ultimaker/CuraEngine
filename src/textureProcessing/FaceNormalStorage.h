/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_FACE_NORMAL_STORAGE_H
#define TEXTURE_PROCESSING_FACE_NORMAL_STORAGE_H

#include "../mesh.h"
#include "../utils/NoCopy.h"

namespace cura
{

/*!
 * helper class for storing mesh face data to be used by each TextureBumpMapProcessor of one mesh
 */
class FaceNormalStorage : NoCopy
{
public:
    /*!
     * Constructor to compute the tan angle for all faces in the model.
     */
    FaceNormalStorage(Mesh* mesh);

    /*!
     * Get the ratio between the vertical and horizontal component of the face normal vector,
     * i.e. the ratio between the horizontal and vertical component of the mesh face.
     * 
     * returns a negative amount for faces angling downward
     * (TODO verify above sentence)
     * \return the ratio between the vertical and the horizontal aspect of the normal of the face with index \p face_index (in the list of faces in the \ref Mesh)
     */
    float getFaceTanAngle(unsigned int face_idx);

    /*!
     * Get the vertical component of the face normal vector,
     * i.e. the horizontal component of the mesh face.
     * 
     * returns a negative amount for faces angling downward
     * (TODO verify above sentence)
     * \return the ratio between the vertical and the horizontal aspect of the normal of the face with index \p face_index (in the list of faces in the \ref Mesh)
     */
    float getFaceHorizontalComponent(unsigned int face_idx);

    /*!
     * Get the horizontal component of the face normal vector,
     * i.e. the vertical component of the mesh face.
     * 
     * Always positive
     * \return the ratio between the vertical and the horizontal aspect of the normal of the face with index \p face_index (in the list of faces in the \ref Mesh)
     */
    float getFaceVerticalComponent(unsigned int face_idx);
protected:

    /*!
     * compute the normal of one face
     * \p p0, \p p1 and \p p2 should be in CCW order
     */
    Point3 computeFaceNormal(const Point3 p0, const Point3 p1, const Point3 p2) const;
    std::vector<Point3> face_normal; //!< for each face the normal angle
};

} // namespace cura

#endif // TEXTURE_PROCESSING_FACE_NORMAL_STORAGE_H