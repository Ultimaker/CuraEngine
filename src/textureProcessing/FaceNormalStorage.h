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
     * Get the horizontal component of the face normal
     * 
     * returns a negative amount for faces angling downward
     * (TODO verify above sentence)
     * \return the ratio between the vertical and the horizontal aspect of the normal of the face with index \p face_index (in the list of faes in the \ref Mesh)
     */
    float getFaceTanAngle(unsigned int face_idx);
protected:

    /*!
     * compute the tan angle of one face
     * \p p0, \p p1 and \p p2 should be in CCW order
     */
    float computeFaceTanAngle(const Point3 p0, const Point3 p1, const Point3 p2) const;
    std::vector<float> face_normal_vertical_component; //!< for each face the horizontal component of the normal angle
};

} // namespace cura

#endif // TEXTURE_PROCESSING_FACE_NORMAL_STORAGE_H