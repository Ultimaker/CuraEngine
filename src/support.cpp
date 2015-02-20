/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "support.h"

namespace cura {

template<typename T> inline void swap(T& p0, T& p1)
{
    T tmp = p0;
    p0 = p1;
    p1 = tmp;
}

int cmp_SupportPoint(const void* a, const void* b)
{
    return ((SupportPoint*)a)->z - ((SupportPoint*)b)->z;
}

void generateSupportGrid(SupportStorage& storage, PrintObject* object, int supportAngle, bool supportEverywhere, int supportXYDistance, int supportZDistance)
{
    storage.areaSupport = false;
    storage.generated = false;
    if (supportAngle < 0)
        return;
    storage.generated = true;
    Point3 object_min = object->min();
    Point3 object_max = object->max();
    Point3 object_size = object_max - object_min;
    
    storage.gridOffset.X = object_min.x;
    storage.gridOffset.Y = object_min.y;
    storage.gridScale = 200;
    storage.gridWidth = (object_size.x / storage.gridScale) + 1;
    storage.gridHeight = (object_size.y / storage.gridScale) + 1;
    storage.grid = new std::vector<SupportPoint>[storage.gridWidth * storage.gridHeight];
    storage.angle = supportAngle;
    storage.everywhere = supportEverywhere;
    storage.XYDistance = supportXYDistance;
    storage.ZDistance = supportZDistance;

    for(Mesh& mesh : object->meshes)
    {
        for(MeshFace& face : mesh.faces)
        {
            Point3 v0 = mesh.vertices[face.vertex_index[0]].p;
            Point3 v1 = mesh.vertices[face.vertex_index[1]].p;
            Point3 v2 = mesh.vertices[face.vertex_index[2]].p;
            
            FPoint3 normal = FPoint3(v1 - v0).cross(FPoint3(v2 - v0));
            double normalSize = normal.vSize();
            
            double cosAngle = fabs(normal.z / normalSize);
            
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

bool SupportPolyGenerator::needSupportAt(Point p)
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
            if (storage.grid[n][i].cosAngle >= cosAngle && storage.grid[n][i].z - supportZDistance >= z && (i == 0 || storage.grid[n][i-1].z + supportZDistance < z))
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

void SupportPolyGenerator::lazyFill(Point startPoint)
{
    static int nr = 0;
    nr++;
    PolygonRef poly = polygons.newPoly();
    Polygon tmpPoly;

    while(1)
    {
        Point p = startPoint;
        done[p.X + p.Y * storage.gridWidth] = nr;
        while(needSupportAt(p + Point(1, 0)))
        {
            p.X ++;
            done[p.X + p.Y * storage.gridWidth] = nr;
        }
        tmpPoly.add(startPoint * storage.gridScale + storage.gridOffset - Point(storage.gridScale/2, 0));
        poly.add(p * storage.gridScale + storage.gridOffset);
        startPoint.Y++;
        while(!needSupportAt(startPoint) && startPoint.X <= p.X)
            startPoint.X ++;
        if (startPoint.X > p.X)
        {
            for(unsigned int n=0;n<tmpPoly.size();n++)
            {
                poly.add(tmpPoly[tmpPoly.size()-n-1]);
            }
            polygons.add(poly);
            return;
        }
        while(needSupportAt(startPoint - Point(1, 0)) && startPoint.X > 1)
            startPoint.X --;
    }
}
    
SupportPolyGenerator::SupportPolyGenerator(SupportStorage& storage, int32_t z)
: storage(storage), z(z), everywhere(storage.everywhere)
{
    std::cerr << "storage.generated = " << storage.generated << std::endl;
    if (!storage.generated)
        return;
    
    std::cerr << "storage.areaSupport = " << storage.areaSupport << std::endl;
    if (storage.areaSupport)
    {
        polygons = storage.supportAreasPerLayer[z];
    } else
    {
        cosAngle = cos(double(90 - storage.angle) / 180.0 * M_PI) - 0.01;
        this->supportZDistance = storage.ZDistance;

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

        delete[] done;
        
        polygons = polygons.offset(storage.XYDistance);
    }
}




void generateSupportAreas(SliceDataStorage& storage, PrintObject* object, int layer_count, int supportAngle, bool supportEverywhere, int supportXYDistance, int supportZDistance)
{
    storage.support.angle = supportAngle;
    storage.support.everywhere = supportEverywhere;
    storage.support.XYDistance = supportXYDistance;
    storage.support.ZDistance = supportZDistance;
    storage.support.areaSupport = true;
    
    storage.support.generated = false;
    if (supportAngle < 0)
        return;
    
    
    int layerThickness = object->getSettingInt("layerThickness");
    
    int layerZdistance = supportZDistance / layerThickness + 1; // support must always be 1 layer below overhang

    double cosAngle = cos(double(90 - storage.support.angle) / 180.0 * M_PI) - 0.01;
    int maxDistFromLowerLayer = cosAngle * layerThickness; // max dist which can be bridged
    
    int expansionsDist = maxDistFromLowerLayer; // dist to expand overhang back toward model, effectively supporting stuff between the overhang and the perimeter of the lower layer
    
    int joinDist = 100; // largest distance between separate support parts which will be joined into one
    
    
    // initialization
    for (int l = 0; l < layer_count ; l++)
        storage.support.supportAreasPerLayer.emplace_back();
    
    // compute basic overhang and put in right layer ([layerZdistance] layers down)
    for (int l = layer_count - 1 ; l >= layerZdistance ; l--)
    {
    
        Polygons supported;
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& lowerLayer = mesh.layers[l-1];
            for (SliceLayerPart& part : lowerLayer.parts)
                supported = supported.unionPolygons(part.outline);
        }
        supported = supported.offset(maxDistFromLowerLayer);
        
        Polygons basic_overhang;
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& thisLayer = mesh.layers[l];
            for (SliceLayerPart& part : thisLayer.parts)
                basic_overhang = supported.unionPolygons(part.outline);
        }
        basic_overhang = basic_overhang.difference(supported);
        
        Polygons overhang = basic_overhang.offset(expansionsDist);
        overhang = overhang.intersection(supported);
        storage.support.supportAreasPerLayer[l-layerZdistance] = basic_overhang;
    }
    /*
    for (int l = storage.support.supportAreasPerLayer.size() - 2 ; l >= 0 ; l--)
    {
//         SliceLayer& sliceLayer = storage.layers[l];
        Polygons& supportLayer_up = storage.support.supportAreasPerLayer[l+1];
        Polygons& supportLayer = storage.support.supportAreasPerLayer[l];
        
        // join overhangs downward
        
        Polygons joined = supportLayer.unionPolygons(supportLayer_up);
        joined = joined.offset(joinDist);
        joined = joined.offset(-joinDist);
    
        // inset using XYdistance
        
        Polygons insetted = supportLayer;
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& sliceLayer = mesh.layers[l];
            for (SliceLayerPart& part : sliceLayer.parts)
                insetted = insetted.difference(part.outline.offset(supportXYDistance));
        }
        storage.support.supportAreasPerLayer[l] = insetted;
    }
    
    // do stuff for when not supportEverywhere
    {
        // ...
    }

    
    // move up from model
    for (int l = 0 ; l < storage.support.supportAreasPerLayer.size() ; l++)
    {
//         SliceLayer& sliceLayer = storage.layers[l];
        Polygons& supportLayer = storage.support.supportAreasPerLayer[l];
        
        Polygons without_base = supportLayer;
        for (int l2 = std::min(0, l-layerZdistance); l2 < l; l2++)
            for (SliceMeshStorage& mesh : storage.meshes)
                for (SliceLayerPart& part : mesh.layers[l2].parts)
                    without_base = without_base.difference(part.outline);
        
        
        storage.support.supportAreasPerLayer[l] = without_base;
        
    }
    */
    
    storage.support.generated = true;
}


}//namespace cura
