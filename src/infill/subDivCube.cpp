#include "subDivCube.h"
#include "functional"
#include "../utils/polygonUtils.h"
#include "../sliceDataStorage.h"
namespace cura {
SubDivCube* baseSubDivCube;
double SubDivCube::rotCoefX;
double SubDivCube::rotCoefY;
std::vector<int64_t> SubDivCube::sideLen;
std::vector<int64_t> SubDivCube::height;
std::vector<int64_t> SubDivCube::maxDrawDiff;
std::vector<int64_t> SubDivCube::squareCutAcross;
std::vector<int64_t> SubDivCube::maxLineOffset;
double SubDivCube::radMult = 1;
int32_t SubDivCube::radAdd = 0;

void SubDivCube::draw(int64_t z, Polygons& result, Polygons** dir)
{
    int epsilon = 10;
    auto addLine = [&](Point from, Point to)//this simply adds a line segment to result
    {
        PolygonRef p = result.newPoly();
        p.add(from);
        p.add(to);
    };
    auto addLineAndCombine = [&](Polygons& group, Point from, Point to)//this adds a line segment to a polygon of parallel line segments and combines segments that touch within epsilon
    {
        for(int x = 0; x < group.size(); x++){
            if(abs(from.X-group[x][1].X)<epsilon && abs(from.Y-group[x][1].Y)<epsilon){
                from = group[x][0];
                group.remove(x);
                x--;
                continue;
            }
            if(abs(to.X-group[x][0].X)<epsilon && abs(to.Y-group[x][0].Y)<epsilon){
                to = group[x][1];
                group.remove(x);
                x--;
                continue;
            }
        }
        PolygonRef p = group.newPoly();
        p.add(from);
        p.add(to);
    };
    int topLevel = 0;//if this cube is the top level (ie root of octree).
    if(dir == NULL){//if the three directional polygons have not yet been created, then this is the root of the recursive draw call
        dir = (Polygons**)calloc(3, sizeof(Polygons*));
        for(int temp = 0; temp < 3; temp++) dir[temp] = new Polygons();
        topLevel=1;
    }
    int32_t diff = abs(z-center.z);//z difference from current layer
    if(diff>height[d]/2){//outside of cube. No drawing or subdivision needed
        return;
    }
    if(diff<maxDrawDiff[d]){//inside of drawing range
        Point rela, relb, a, b;
        rela.X=(squareCutAcross[d]/2)*((double)(maxDrawDiff[d]-diff)/(double)maxDrawDiff[d]);//rela and relb are relative coordinates of the two end points of the drawn line from the cube center
        relb.X=-rela.X;
        rela.Y=maxLineOffset[d]-((z-(center.z-maxDrawDiff[d]))*(1/sqrt(2)));
        relb.Y=rela.Y;
        initRot(rela);
        initRot(relb);
        for(int temp = 0; temp < 3; temp++){//draw and rotate
            a.X = center.x+rela.X;
            a.Y = center.y+rela.Y;
            b.X = center.x+relb.X;
            b.Y = center.y+relb.Y;
            addLineAndCombine(*(dir[temp]), a, b);
            if(temp < 2){
                rot120(rela);
                rot120(relb);
            }
        }
    }
    for(int temp = 0; temp < 8; temp++){//draw all children too
        if(children[temp] != NULL){
            children[temp]->draw(z, result, dir);
        }
    }
    if(topLevel){//take care of copying the contents of the three directional polygons into result set AND free the three directional polygons
        for(int temp = 0; temp < 3; temp++){
            for(unsigned int x = 0; x < dir[temp]->size(); x++){
                addLine((*dir[temp])[x][0], (*dir[temp])[x][1]);
            }
            delete dir[temp];
        }
        free(dir);
    }
}

SubDivCube::SubDivCube(SliceMeshStorage& mesh, Point3& myCenter, int d){
    this->d = d;//d is depth of current recursion. kind of. except 0 is completely recursed...
    center = myCenter;

    if(d == 0) return;//Lowest layer, no need for subdivision.
    Point3 c;//c will be the centers of the new 8 cubes
    int32_t nsideLen = sideLen[d-1];//the new sideLen is the sideLen for recursed one more time...
    long int rad = radMult*height[d]/4+radAdd;//note:divided by four because it is half of half of the parent height.
    //top
    c.x=center.x;
    c.y=center.y;
    c.z=center.z+(height[d]/4);
    if(subDiv(mesh, c, nsideLen, rad)){
        children[0] = new SubDivCube(mesh, c, d-1);
    }
    //top three
    Point relCenter;
    c.z=center.z+height[d]/12;
    relCenter.X=0;
    relCenter.Y=-maxLineOffset[d];
    initRot(relCenter);
    for(int temp = 0; temp < 3; temp++){
        c.x = relCenter.X+center.x;
        c.y = relCenter.Y+center.y;
        if(subDiv(mesh, c, nsideLen, rad)){
            children[temp+1] = new SubDivCube(mesh, c, d-1);
        }
        if(temp < 2){
            rot120(relCenter);
        }
    }
    //bottom
    c.x=center.x;
    c.y=center.y;
    c.z=center.z-(height[d]/4);
    if(subDiv(mesh, c, nsideLen, rad)){
        children[4] = new SubDivCube(mesh, c, d-1);
    }
    //botton three
    c.z=center.z-height[d]/12;
    relCenter.X=0;
    relCenter.Y=maxLineOffset[d];
    initRot(relCenter);
    for(int temp = 0; temp < 3; temp++){
        c.x = relCenter.X+center.x;
        c.y = relCenter.Y+center.y;
        if(subDiv(mesh, c, nsideLen, rad)){
            children[temp+5] = new SubDivCube(mesh, c, d-1);
        }
        if(temp < 2){
            rot120(relCenter);
        }
    }
}

int SubDivCube::subDiv(SliceMeshStorage& mesh, Point3& center, int32_t sideLen, int32_t rad){//put inside of constructor as a lambda? (returns one if a described cube should be subdivided, else zero)
    int32_t distance;
    long int heightRad;//radius of sphere slice on target layer
    int insideSomewhere = 0;
    int outsideSomewhere = 0;
    int inside;
    double partDist;
    long int layer_height = mesh.getSettingInMicrons("layer_height");
    long int bot_layer = (center.z-(rad))/layer_height;
    long int top_layer = (center.z+(rad))/layer_height;
    for(long int templayer = bot_layer; templayer <= top_layer; templayer+=3/*plus three as a quick speed fix... 3 layers doesn't seem like to many?*/){
        partDist = (double)(templayer*layer_height-center.z)/rad;//how far through the radius are we (0-1)
        heightRad = rad*(sqrt(1-(partDist*partDist)));//radius of circumscribed sphere at this level
        Point loc(center.x, center.y);

        inside = dist(mesh, templayer, loc, &distance);
        if(inside == 1){//0 = outside, 1 = inside, 2 = invalid layer(outside)//FIXME LAST does the 2 option cause a speed-up?
            insideSomewhere = 1;
        }else{
            outsideSomewhere = 1;
        }
        if(outsideSomewhere && insideSomewhere){
            return 1;
        }
        if((inside!=2)&&abs(distance) < heightRad){
            return 1;
        }
    }
    return 0;
}

int SubDivCube::dist(SliceMeshStorage& mesh, long int layer_nr, Point& loc, int32_t* dist){
    if(layer_nr < 0 || layer_nr>=mesh.layers.size()){//if this layer is outside of existing layer range...
        return 2;
    }
    Polygons collide;
    mesh.layers[layer_nr].getSecondOrInnermostWalls(collide);
    Point centerpoint = loc;
    bool inside = collide.inside(centerpoint);
    ClosestPolygonPoint rimpoint = PolygonUtils::moveInside2(collide, centerpoint);
    *dist = sqrt((rimpoint.location.X-loc.X)*(rimpoint.location.X-loc.X)+(rimpoint.location.Y-loc.Y)*(rimpoint.location.Y-loc.Y));
    if(inside){
        return 1;
    }
    return 0;
}

void SubDivCube::init(SliceMeshStorage& gMesh){
    radMult = 1;//gMesh.getSettingInMillimeters("sub_div_rad_mult");
    radAdd = 0;//gMesh.getSettingInMicrons("sub_div_rad_add");
    rotCoefX = cos(M_PI/4);//90 degrees hard coded in because it is for the other infills too. This prevents any of the three directions from being parallel to x or y axis
    rotCoefY = sin(M_PI/4);
    int maxDepth = 0;
    for(int64_t maxSideLen = gMesh.getSettingInMicrons("infill_line_distance")*2; maxSideLen < 25600000; maxSideLen *= 2){//beginning at zero (most recursed) precompute values
        sideLen.push_back(maxSideLen);
        height.push_back(sqrt(maxSideLen*maxSideLen*3));
        squareCutAcross.push_back(sqrt(maxSideLen*maxSideLen*2));
        maxDrawDiff.push_back((1.0/sqrt(3.0))*maxSideLen);
        maxLineOffset.push_back((sqrt(2.0/3.0)*maxSideLen)/2);
        maxDepth++;
    }
    Point3 center(0, 0, 0);
    baseSubDivCube = new SubDivCube(gMesh, center, maxDepth-1);
}

void SubDivCube::initRot(Point& targ){//perform initial rotation.
    int64_t x;
    x = rotCoefX*targ.X-rotCoefY*targ.Y;
    targ.Y = rotCoefX*targ.Y+rotCoefY*targ.X;
    targ.X = x;
}

void SubDivCube::rot120(Point& targ){//rotate 120 degrees
    int64_t x;
    x = (-0.5)*targ.X-(sqrt_three_fourths)*targ.Y;
    targ.Y = (-0.5)*targ.Y+(sqrt_three_fourths)*targ.X;
    targ.X = x;
}

}//namespace cura
