#ifndef OPTIMIZED_MODEL_H
#define OPTIMIZED_MODEL_H

#include "utils/intpoint.h"

#include <map>
#include <vector>

class OptimizedFace
{
public:
    int index[3];
    int touching[3];
};
class OptimizedPoint3
{
public:
    Point3 p;
    std::vector<uint32_t> faceIndexList;
    
    OptimizedPoint3(Point3 p): p(p) {}
};

#define MELD_DIST 30
class OptimizedModel
{
public:
    std::vector<OptimizedPoint3> points;
    std::vector<OptimizedFace> faces;
    Point3 modelSize;
    Point3 vMin, vMax;
    
    OptimizedModel(SimpleModel* m, Point3 center)
    {
        points.reserve(m->faces.size() * 3);
        faces.reserve(m->faces.size());
    
        vMin = m->min();
        vMax = m->max();
        std::map<uint32_t, std::vector<uint32_t> > indexMap;
        
        double t = getTime();
        int percDone;
        for(uint32_t i=0; i<m->faces.size(); i++)
        {
            OptimizedFace f;
            percDone = 100*i/m->faces.size();
            if((i%1000==0) && (getTime()-t)>2.0) fprintf(stdout, "\rOptimizing faces (%d percent)",percDone);
            for(uint32_t j=0; j<3; j++)
            {
                Point3 p = m->faces[i].v[j];
                int hash = ((p.x + MELD_DIST/2) / MELD_DIST) ^ (((p.y + MELD_DIST/2) / MELD_DIST) << 10) ^ (((p.z + MELD_DIST/2) / MELD_DIST) << 20);
                uint32_t idx;
                bool add = true;
                for(unsigned int n = 0; n < indexMap[hash].size(); n++)
                {
                    if ((points[indexMap[hash][n]].p - p).testLength(MELD_DIST))
                    {
                        idx = indexMap[hash][n];
                        add = false;
                        break;
                    }
                }
                if (add)
                {
                    indexMap[hash].push_back(points.size());
                    idx = points.size();
                    points.push_back(p);
                }
                f.index[j] = idx;
            }
            if (f.index[0] != f.index[1] && f.index[0] != f.index[2] && f.index[1] != f.index[2])
            {
                //Check if there is a face with the same points
                bool duplicate = false;
                for(unsigned int _idx0 = 0; _idx0 < points[f.index[0]].faceIndexList.size(); _idx0++)
                {
                    for(unsigned int _idx1 = 0; _idx1 < points[f.index[1]].faceIndexList.size(); _idx1++)
                    {
                        for(unsigned int _idx2 = 0; _idx2 < points[f.index[2]].faceIndexList.size(); _idx2++)
                        {
                            if (points[f.index[0]].faceIndexList[_idx0] == points[f.index[1]].faceIndexList[_idx1] && points[f.index[0]].faceIndexList[_idx0] == points[f.index[2]].faceIndexList[_idx2])
                                duplicate = true;
                        }
                    }
                }
                if (!duplicate)
                {
                    points[f.index[0]].faceIndexList.push_back(faces.size());
                    points[f.index[1]].faceIndexList.push_back(faces.size());
                    points[f.index[2]].faceIndexList.push_back(faces.size());
                    faces.push_back(f);
                }
            }
        }
        fprintf(stdout, "\rAll faces are optimized in %5.1fs.\n",timeElapsed(t));

        int openFacesCount = 0;
        for(unsigned int i=0;i<faces.size();i++)
        {
            OptimizedFace* f = &faces[i];
            f->touching[0] = getFaceIdxWithPoints(f->index[0], f->index[1], i);
            f->touching[1] = getFaceIdxWithPoints(f->index[1], f->index[2], i);
            f->touching[2] = getFaceIdxWithPoints(f->index[2], f->index[0], i);
            if (f->touching[0] == -1)
                openFacesCount++;
            if (f->touching[1] == -1)
                openFacesCount++;
            if (f->touching[2] == -1)
                openFacesCount++;
        }
        fprintf(stdout, "  Number of open faces: %i\n", openFacesCount);
        
        Point3 vOffset((vMin.x + vMax.x) / 2, (vMin.y + vMax.y) / 2, vMin.z);
        vOffset -= center;
        for(unsigned int i=0;i<points.size();i++)
        {
            points[i].p -= vOffset;
        }
        modelSize = vMax - vMin;
        vMin -= vOffset;
        vMax -= vOffset;
    }
    
    int getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx)
    {
        for(unsigned int i=0;i<points[idx0].faceIndexList.size();i++)
        {
            int f0 = points[idx0].faceIndexList[i];
            if (f0 == notFaceIdx) continue;
            for(unsigned int j=0;j<points[idx1].faceIndexList.size();j++)
            {
                int f1 = points[idx1].faceIndexList[j];
                if (f1 == notFaceIdx) continue;
                if (f0 == f1) return f0;
            }
        }
        return -1;
    }

    void saveDebugSTL(const char* filename)
    {
        char buffer[80] = "Cura_Engine_STL_export";
        uint32_t n;
        uint16_t s;
        float flt;
        FILE* f = fopen(filename, "wb");
        fwrite(buffer, 80, 1, f);
        n = faces.size();
        fwrite(&n, sizeof(n), 1, f);
        for(unsigned int i=0;i<faces.size();i++)
        {
            flt = 0;
            s = 0;
            fwrite(&flt, sizeof(flt), 1, f);
            fwrite(&flt, sizeof(flt), 1, f);
            fwrite(&flt, sizeof(flt), 1, f);

            flt = points[faces[i].index[0]].p.x / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[0]].p.y / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[0]].p.z / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[1]].p.x / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[1]].p.y / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[1]].p.z / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[2]].p.x / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[2]].p.y / 1000.0; fwrite(&flt, sizeof(flt), 1, f);
            flt = points[faces[i].index[2]].p.z / 1000.0; fwrite(&flt, sizeof(flt), 1, f);

            fwrite(&s, sizeof(s), 1, f);
        }
        fclose(f);
        //Export the open faces so you can view the with Cura (hacky)
        /*
        char gcodeFilename[1024];
        strcpy(gcodeFilename, filename);
        strcpy(strchr(gcodeFilename, '.'), ".gcode");
        f = fopen(gcodeFilename, "w");
        for(unsigned int i=0;i<faces.size();i++)
        {
            for(int j=0;j<3;j++)
            {
                if (faces[i].touching[j] == -1)
                {
                    Point3 p0 = points[faces[i].index[j]].p;
                    Point3 p1 = points[faces[i].index[(j+1)%3]].p;
                    fprintf(f, ";Model error(open face): (%f, %f, %f) (%f, %f, %f)\n", p0.x / 1000.0, p0.y / 1000.0, p0.z / 1000.0, p1.x / 1000.0, p1.y / 1000.0, p1.z / 1000.0);
                }
            }
        }
        fclose(f);
        */
    }
};

#endif//OPTIMIZED_MODEL_H
