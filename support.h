/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SUPPORT_H
#define SUPPORT_H

inline void swap(Point3& p0, Point3& p1)
{
    Point3 tmp = p0;
    p0 = p1;
    p1 = tmp;
}
inline void swap(int64_t& n, int64_t& m)
{
    int64_t tmp = n;
    n = m;
    m = tmp;
}

int cmp_SupportPoint(const void* a, const void* b)
{
    return ((SupportPoint*)a)->z - ((SupportPoint*)b)->z;
}

void generateSupportGrid(SupportStorage& storage, OptimizedModel* om)
{
    storage.gridOffset.X = om->vMin.x;
    storage.gridOffset.Y = om->vMin.y;
    storage.gridScale = 200;
    storage.gridWidth = (om->modelSize.x / storage.gridScale) + 1;
    storage.gridHeight = (om->modelSize.y / storage.gridScale) + 1;
    storage.grid = new vector<SupportPoint>[storage.gridWidth * storage.gridHeight];

    for(unsigned int volumeIdx = 0; volumeIdx < om->volumes.size(); volumeIdx++)
    {
        OptimizedVolume* vol = &om->volumes[volumeIdx];
        for(unsigned int faceIdx = 0; faceIdx < vol->faces.size(); faceIdx++)
        {
            OptimizedFace* face = &vol->faces[faceIdx];
            Point3 v0 = vol->points[face->index[0]].p;
            Point3 v1 = vol->points[face->index[1]].p;
            Point3 v2 = vol->points[face->index[2]].p;
            
            Point3 normal = (v1 - v0).cross(v2 - v0);
            int32_t normalSize = normal.vSize();
            
            double cosAngle = fabs(double(normal.z) / double(normalSize));
            
            v0.x = (v0.x - storage.gridOffset.X) / storage.gridScale;
            v0.y = (v0.y - storage.gridOffset.Y) / storage.gridScale;
            v1.x = (v1.x - storage.gridOffset.X) / storage.gridScale;
            v1.y = (v1.y - storage.gridOffset.Y) / storage.gridScale;
            v2.x = (v2.x - storage.gridOffset.X) / storage.gridScale;
            v2.y = (v2.y - storage.gridOffset.Y) / storage.gridScale;

            if (v0.x > v1.x) swap(v0, v1);
            if (v1.x > v2.x) swap(v1, v2);
            if (v0.x > v1.x) swap(v0, v1);
            for(int64_t x=v0.x; x<v1.x; x++)
            {
                int64_t y0 = v0.y + (v1.y - v0.y) * (x - v0.x) / (v1.x - v0.x);
                int64_t y1 = v0.y + (v2.y - v0.y) * (x - v0.x) / (v2.x - v0.x);
                int64_t z0 = v0.z + (v1.z - v0.z) * (x - v0.x) / (v1.x - v0.x);
                int64_t z1 = v0.z + (v2.z - v0.z) * (x - v0.x) / (v2.x - v0.x);

                if (y0 > y1) { swap(y0, y1); swap(z0, z1); }
                for(int64_t y=y0; y<y1; y++)
                    storage.grid[x+y*storage.gridWidth].push_back(SupportPoint(z0 + (z1 - z0) * (y-y0) / (y1-y0), cosAngle));
            }
            for(int64_t x=v1.x; x<v2.x; x++)
            {
                int64_t y0 = v1.y + (v2.y - v1.y) * (x - v1.x) / (v2.x - v1.x);
                int64_t y1 = v0.y + (v2.y - v0.y) * (x - v0.x) / (v2.x - v0.x);
                int64_t z0 = v1.z + (v2.z - v1.z) * (x - v1.x) / (v2.x - v1.x);
                int64_t z1 = v0.z + (v2.z - v0.z) * (x - v0.x) / (v2.x - v0.x);

                if (y0 > y1) { swap(y0, y1); swap(z0, z1); }
                for(int64_t y=y0; y<y1; y++)
                    storage.grid[x+y*storage.gridWidth].push_back(SupportPoint(z0 + (z1 - z0) * (y-y0) / (y1-y0), cosAngle));
            }
        }
    }
    
    for(int32_t x=0; x<storage.gridWidth; x++)
    {
        for(int32_t y=0; y<storage.gridHeight; y++)
        {
            unsigned int n = x+y*storage.gridWidth;
            qsort(storage.grid[n].data(), storage.grid[n].size(), sizeof(SupportPoint), cmp_SupportPoint);
        }
    }
    storage.gridOffset.X += storage.gridScale / 2;
    storage.gridOffset.Y += storage.gridScale / 2;
}

class SupportPolyGenerator
{
public:
    Polygons polygons;

private:
    SupportStorage& storage;
    double cosAngle;
    int32_t z;
    int supportZDistance;
    bool everywhere;
    int* done;

    inline bool needSupportAt(Point p)
    {
        if (p.X < 1) return false;
        if (p.Y < 1) return false;
        if (p.X >= storage.gridWidth - 1) return false;
        if (p.Y >= storage.gridHeight - 1) return false;
        if (done[p.X + p.Y * storage.gridWidth]) return false;
        
        unsigned int n = p.X+p.Y*storage.gridWidth;
        
        if (everywhere)
        {
            bool ok = false;
            for(unsigned int i=0; i<storage.grid[n].size(); i+=2)
            {
                if (storage.grid[n][i].cosAngle >= cosAngle && storage.grid[n][i].z + supportZDistance >= z && (i == 0 || storage.grid[n][i-1].z - supportZDistance < z))
                {
                    ok = true;
                    break;
                }
            }
            if (!ok) return false;
        }else{
            if (storage.grid[n].size() < 1) return false;
            if (storage.grid[n][0].cosAngle < cosAngle) return false;
            if (storage.grid[n][0].z - supportZDistance < z) return false;
        }
        return true;
    }

    void lazyFill(Point startPoint)
    {
        static int nr = 0;
        nr++;
        ClipperLib::Polygon poly;
        ClipperLib::Polygon tmpPoly;

        while(1)
        {
            Point p = startPoint;
            done[p.X + p.Y * storage.gridWidth] = nr;
            while(needSupportAt(p + Point(1, 0)))
            {
                p.X ++;
                done[p.X + p.Y * storage.gridWidth] = nr;
            }
            tmpPoly.push_back(startPoint * storage.gridScale + storage.gridOffset - Point(storage.gridScale/2, 0));
            poly.push_back(p * storage.gridScale + storage.gridOffset);
            startPoint.Y++;
            while(!needSupportAt(startPoint) && startPoint.X <= p.X)
                startPoint.X ++;
            if (startPoint.X > p.X)
            {
                for(unsigned int n=0;n<tmpPoly.size();n++)
                {
                    poly.push_back(tmpPoly[tmpPoly.size()-n-1]);
                }
                polygons.add(poly);
                return;
            }
            while(needSupportAt(startPoint - Point(1, 0)) && startPoint.X > 1)
                startPoint.X --;
        }
    }
    
public:
    SupportPolyGenerator(SupportStorage& storage, int32_t z, int angle, bool everywhere, int supportDistance, int supportZDistance)
    : storage(storage), z(z), everywhere(everywhere)
    {
        cosAngle = cos(double(90 - angle) / 180.0 * M_PI) - 0.01;
        this->supportZDistance = supportZDistance;

        done = new int[storage.gridWidth*storage.gridHeight];
        memset(done, 0, sizeof(int) * storage.gridWidth*storage.gridHeight);
        
        for(int32_t y=1; y<storage.gridHeight; y++)
        {
            for(int32_t x=1; x<storage.gridWidth; x++)
            {
                if (!needSupportAt(Point(x, y)) || done[x + y * storage.gridWidth]) continue;
                
                lazyFill(Point(x, y));
            }
        }

        delete done;
        
        polygons = polygons.offset(supportDistance);
        
        /*
        if (xAxis)
        {
            for(int32_t y=0; y<storage.gridHeight; y+=4)
            {
                for(int32_t x=0; x<storage.gridWidth; x++)
                {
                    if (!needSupportAt(Point(x, y))) continue;
                    int32_t startX = x;
                    while(x < storage.gridWidth && needSupportAt(Point(x, y)))
                    {
                        x ++;
                    }
                    x --;
                    if (x > startX)
                    {
                        Point p0(startX * storage.gridScale + storage.gridOffset.X, y * storage.gridScale + storage.gridOffset.Y);
                        Point p1(x * storage.gridScale + storage.gridOffset.X, y * storage.gridScale + storage.gridOffset.Y);
                        ClipperLib::Polygon p;
                        p.push_back(p0);
                        p.push_back(p1);
                        polygons.push_back(p);
                    }
                }
            }
        }else{
            for(int32_t x=0; x<storage.gridWidth; x+=1)
            {
                for(int32_t y=0; y<storage.gridHeight; y++)
                {
                    if (!needSupportAt(Point(x, y))) continue;
                    
                    int32_t startY = y;
                    while(y < storage.gridHeight && needSupportAt(Point(x, y)))
                    {
                        y ++;
                    }
                    y --;
                    if (y > startY)
                    {
                        Point p0(x * storage.gridScale + storage.gridOffset.X, startY * storage.gridScale + storage.gridOffset.Y);
                        Point p1(x * storage.gridScale + storage.gridOffset.X, y * storage.gridScale + storage.gridOffset.Y);
                        ClipperLib::Polygon p;
                        p.push_back(p0);
                        p.push_back(p1);
                        polygons.push_back(p);
                    }
                }
            }
        }
        */
    }
};

#endif//SUPPORT_H
