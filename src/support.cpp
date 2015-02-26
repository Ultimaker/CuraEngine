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
    
SupportPolyGenerator::SupportPolyGenerator(SupportStorage& storage, int32_t z, int layer_nr)
: storage(storage), z(z), everywhere(storage.everywhere)
{
    if (!storage.generated)
    {
        log("No support generated.\n");
        return;
    }
    if (storage.areaSupport)
    {
        polygons = storage.supportAreasPerLayer[layer_nr];
        if (layer_nr==0) log("skirt? raft? or just layer zero? n polygons = %i\n one layer up : %i\n", polygons.size(), storage.supportAreasPerLayer[1].size());
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




void generateSupportAreas(SliceDataStorage& storage, PrintObject* object, int layer_count)
{
    // given settings
    int supportAngle = object->getSettingInt("supportAngle");
    bool supportEverywhere = object->getSettingInt("supportEverywhere") > 0;
    int supportXYDistance = object->getSettingInt("supportXYDistance");
    int supportZDistance = object->getSettingInt("supportZDistance");
    int supportZDistanceBottom = object->getSettingInt("supportZDistanceBottom");
    int supportZDistanceTop = object->getSettingInt("supportZDistanceTop");
    int supportJoinDistance = object->getSettingInt("supportJoinDistance");
    //int supportSkipLayers = object->getSettingInt("supportSkipLayers");
    float backSupportBridge = static_cast<float>(object->getSettingInt("supportBridgeBack"))/100.0;
        
    int layerThickness = object->getSettingInt("layerThickness");
    
    storage.support.angle = supportAngle;
    storage.support.everywhere = supportEverywhere;
    storage.support.XYDistance = supportXYDistance;
    storage.support.ZDistance = supportZDistance;
    storage.support.areaSupport = true;
    
    storage.support.generated = false;
    if (supportAngle < 0)
        return;
    
    // derived settings:
    
    if (supportZDistanceBottom < 0) supportZDistanceBottom = supportZDistance;
    if (supportZDistanceTop < 0)    supportZDistanceTop = supportZDistance;
    
    
    
    int layerZdistanceTop       = supportZDistanceTop / layerThickness + 1; // support must always be 1 layer below overhang
    int layerZdistanceBottom    = supportZDistanceBottom / layerThickness; 

    double tanAngle = tan(double(storage.support.angle) / 180.0 * M_PI) - 0.01;
    int maxDistFromLowerLayer = tanAngle * layerThickness; // max dist which can be bridged
    
    int expansionsDist = backSupportBridge * maxDistFromLowerLayer; // dist to expand overhang back toward model, effectively supporting stuff between the overhang and the perimeter of the lower layer
    
    // computation
    
    std::cerr << "joinging model layers" << std::endl;
    
    // join model layers into polygons
    std::vector<Polygons> joinedLayers;
    for (int l = 0 ; l < layer_count ; l++)
    {
        joinedLayers.emplace_back();
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& layer = mesh.layers[l];
            for (SliceLayerPart& part : layer.parts)
                joinedLayers[l] = joinedLayers[l].unionPolygons(part.outline);
        }
    }
        
    // initialization of supportAreasPerLayer
    for (int l = 0; l < layer_count ; l++)
        storage.support.supportAreasPerLayer.emplace_back();


    std::cerr << "computing support" << std::endl;
    
    Polygons supportLayer_last;
    for (int l = layer_count - 1 - layerZdistanceTop; l >= 0 ; l--)
    {
        // compute basic overhang and put in right layer ([layerZdistanceTOp] layers below)
        Polygons supportLayer_supportee =  joinedLayers[l+layerZdistanceTop];
        Polygons supportLayer_supported =  joinedLayers[l-1+layerZdistanceTop].offset(maxDistFromLowerLayer);
        Polygons basic_overhang = supportLayer_supportee.difference(supportLayer_supported);
        
        Polygons support_extension = basic_overhang.offset(expansionsDist);
        support_extension = support_extension.intersection(supportLayer_supported);
        support_extension = support_extension.intersection(supportLayer_supportee);
        
        Polygons overhang =  basic_overhang.unionPolygons(support_extension);
        
        Polygons& supportLayer_this = overhang; 
        
        
        
        if (l+1 < layer_count)
        { // join with support from layer up
            Polygons& supportLayer_up = supportLayer_last;
            
            Polygons joined = supportLayer_this.unionPolygons(supportLayer_up);
            if (supportJoinDistance > 0)
            {
                joined = joined.offset(supportJoinDistance);
                joined = joined.offset(-supportJoinDistance);
            }
        
            // remove layer
            Polygons insetted = joined.difference(joinedLayers[l]);
            supportLayer_this = insetted;                
            
        }
        
        
        supportLayer_last = supportLayer_this;
        
        // inset using X/Y distance
        if (supportLayer_this.size() > 0)
            supportLayer_this = supportLayer_this.difference(joinedLayers[l].offset(supportXYDistance));
        
        // move up from model
        if (layerZdistanceBottom > 0 && l >= layerZdistanceBottom)
        {
            supportLayer_this = supportLayer_this.difference(joinedLayers[l-layerZdistanceBottom]);
        }
        
        storage.support.supportAreasPerLayer[l] = supportLayer_this;
        
    }
    
    // do stuff for when support on buildplate only
    if (!supportEverywhere)
    {
        std::cerr << "supporting on buildplate only" << std::endl;
        Polygons touching_buildplate = storage.support.supportAreasPerLayer[0];
        for (int l = 1 ; l < storage.support.supportAreasPerLayer.size() ; l++)
        {
            Polygons& supportLayer = storage.support.supportAreasPerLayer[l];
            
            touching_buildplate = supportLayer.intersection(touching_buildplate);
            
            storage.support.supportAreasPerLayer[l] = touching_buildplate;
        }
    }

    
    joinedLayers.clear();
    std::cerr<<"finished area support" <<std::endl;
    
    storage.support.generated = true;
}


}//namespace cura
