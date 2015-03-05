/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SUPPORT_H
#define SUPPORT_H

#include "sliceDataStorage.h"
#include "modelFile/modelFile.h"

namespace cura {

/*!
    Fill the SupportStorage object with data from the 3D model.
    This sets the settings in the SupportStorage object. And it generates a 2D grid on the XY plane.
    Each point on this grid is filled with an array of Z heights where the grid point crosses the 3D model.
    As well as the cosinus of the polygon angle compared to the XY plane (which is later used to classify where support is needed and not)
    The array per point is sorted by Z height, with the lowest height first.
    
    The 2D grid is generated with a fixed distance of 0.2mm
 */
void generateSupportGrid(SupportStorage& storage, PrintObject* om, int supportAngle, bool supportEverywhere, int supportXYDistance, int supportZDistance);

/*!
    SupportPolyGenerator.
    
    This object generates 2D polygons for a certain Z height.
    It uses a Z height as well as the data generated with "generateSupportGrid" to map out which sections
    of the 3D model need support material for the given Z height.
    
    The result is stored in SupportPolyGenerator.polygons
    
    This class is used during GCode generation, and only lives as while this layer is being generated.
 */
class SupportPolyGenerator
{
public:
    Polygons polygons;  //!< These are the resulting polygons that need to be filled to get a proper support structure in a single layer.

private:
    SupportStorage& storage;    //!< This is the reference to the SupportStorage data, which is filled by the generateSupportGrid at the first step of processing.
    double cosAngle;            //!< The angle at which support needs to be generated, converted to a cosinus, to make it easy to compare it with the cosAngle of the grid points in the SupportStorage
    int32_t z;                  //!< The Z height that is checked for support. Given as parameter to the constructor and stored here.
    int supportZDistance;       //!< The minimal distance between the object and this layer of support in the Z direction. (Copied from SupportStorage)
    bool everywhere;            //!< If support needs to be generated everywhere, or only when touching the buildplate (Copied from SupportStorage)
    int* done;                  //!< A 2D grid which matches the size of the SupportStorage grid, contains a flag to indicate that this point has been handled already.

    /*!
        Check of the grid point at p needs support. Checks this according to the height given to the constructor, the everywhere flag, as well as the support angle.
    */
    bool needSupportAt(Point p);
    
    /*!
        A fill algorithm algorithm that generates a single polygon around points that need support.
        
        1) The algorthim starts at "startPoint", and starts filling to the right (x+1) till it sees a point that no longer needs support.
        2) Then it goes down (y+1) a row, and finds the proper point to start the next line (starting at startPoint.x going to the right till it sees a point where it needs support again)
        3) If this is beyond the final point of step 1 then it generates the final polygon and stops.
            If it's within the range of step 1 then it goes to step 1 again to add this line as well. This loops till it can no longer add a line.
        
        This generates polygons that are garanteed closed with no holes. Which are unioned together later on to make the full support material areas.
        The lazyFill algorithm marks each point as done when it has finished generating support for that point.
        
        (Yes, this code is horrible)
    */
    void lazyFill(Point startPoint);
    
public:
    /*!
        SupportPolyGenerator fills the SupportPolyGenerator.polygons with the polygons it needs to print support on this layer.
        It does this by walking trough the SupportStorage grid and for each point where it needs support it will call the lazyFill algorithm.
        And as a final step it uses the offset function to union all the generated support polygons together and apply the XY offset which grows the support material a bit.
        
        Note that this does not cut the object from the support material. The fffProcessor will apply the XY offset to the outline of the print and substract that from the given support polygons.
     */
    SupportPolyGenerator(SupportStorage& storage, int32_t z, int layer_nr);
};



void generateSupportAreas(SliceDataStorage& storage, PrintObject* object, int layer_count);

void generateZigZagSupport(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, int infillOverlap, double rotation, bool connect_zigzags);



}//namespace cura

#endif//SUPPORT_H
