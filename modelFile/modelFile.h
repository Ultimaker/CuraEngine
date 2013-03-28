#ifndef MODELFILE_H
#define MODELFILE_H
/**
modelFile contains the model loaders for the slicer. The model loader turns any format that it can read into a list of triangles with 3 X/Y/Z points.

The format returned is a Model class with an array of faces, which have integer points with a resolution of 1 micron. Giving a maximum object size of 4 meters.
**/

#include <vector>

#include "utils/intpoint.h"
#include "utils/floatpoint.h"

extern FILE* binaryMeshBlob;

#define SET_MIN(n, m) do { if ((m) < (n)) n = m; } while(0)
#define SET_MAX(n, m) do { if ((m) > (n)) n = m; } while(0)

/* A SimpleFace is a 3 dimensional model triangle with 3 points. These points are already converted to integers */
class SimpleFace
{
public:
    Point3 v[3];
    
    SimpleFace(Point3& v0, Point3& v1, Point3& v2) { v[0] = v0; v[1] = v1; v[2] = v2; }
};

/* A SimpleModel is the most basic reprisentation of a 3D model. It contains all the faces as SimpleTriangles, with nothing fancy. */
class SimpleModel
{
public:
    std::vector<SimpleFace> faces;
    
    void addFace(Point3& v0, Point3& v1, Point3& v2)
    {
        faces.push_back(SimpleFace(v0, v1, v2));
    }
    
    Point3 min()
    {
        Point3 ret = faces[0].v[0];
        for(unsigned int i=0; i<faces.size(); i++)
        {
            SET_MIN(ret.x, faces[i].v[0].x);
            SET_MIN(ret.y, faces[i].v[0].y);
            SET_MIN(ret.z, faces[i].v[0].z);
            SET_MIN(ret.x, faces[i].v[1].x);
            SET_MIN(ret.y, faces[i].v[1].y);
            SET_MIN(ret.z, faces[i].v[1].z);
            SET_MIN(ret.x, faces[i].v[2].x);
            SET_MIN(ret.y, faces[i].v[2].y);
            SET_MIN(ret.z, faces[i].v[2].z);
        }
        return ret;
    }
    Point3 max()
    {
        Point3 ret = faces[0].v[0];
        for(unsigned int i=0; i<faces.size(); i++)
        {
            SET_MAX(ret.x, faces[i].v[0].x);
            SET_MAX(ret.y, faces[i].v[0].y);
            SET_MAX(ret.z, faces[i].v[0].z);
            SET_MAX(ret.x, faces[i].v[1].x);
            SET_MAX(ret.y, faces[i].v[1].y);
            SET_MAX(ret.z, faces[i].v[1].z);
            SET_MAX(ret.x, faces[i].v[2].x);
            SET_MAX(ret.y, faces[i].v[2].y);
            SET_MAX(ret.z, faces[i].v[2].z);
        }
        return ret;
    }
};

SimpleModel* loadModel(const char* filename, FMatrix3x3& matrix);

#endif//MODELFILE_H
