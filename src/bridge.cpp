/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */
#include "bridge.h"

namespace cura {

int bridgeAngle(const Polygons& skinOutline, const SliceLayer* prevLayer, const SupportLayer* supportLayer, Polygons& supportedRegions, const double supportThreshold)
{
    AABB boundaryBox(skinOutline);

    //To detect if we have a bridge, first calculate the intersection of the current layer with the previous layer.
    // This gives us the islands that the layer rests on.
    Polygons islands;

    Polygons prevLayerOutline; // we also want the complete outline of the previous layer

    for(auto prevLayerPart : prevLayer->parts)
    {
        prevLayerOutline.add(prevLayerPart.outline); // not intersected with skin

        if (!boundaryBox.hit(prevLayerPart.boundaryBox))
            continue;
        
        islands.add(skinOutline.intersection(prevLayerPart.outline));
    }
    supportedRegions = islands;

    if (supportLayer)
    {
        // add the regions of the skin that have support below them to supportedRegions
        // but don't add these regions to islands because that can actually cause the code
        // below to consider the skin a bridge when it isn't (e.g. a skin that is supported by
        // the model on one side but the remainder of the skin is above support would look like
        // a bridge because it would have two islands) - FIXME more work required here?

        if (!supportLayer->support_roof.empty())
        {
            AABB support_roof_bb(supportLayer->support_roof);
            if (boundaryBox.hit(support_roof_bb))
            {
                prevLayerOutline.add(supportLayer->support_roof); // not intersected with skin

                Polygons supported_skin(skinOutline.intersection(supportLayer->support_roof));
                if (!supported_skin.empty())
                {
                    supportedRegions.add(supported_skin);
                }
            }
        }
        else
        {
            for (auto support_part : supportLayer->support_infill_parts)
            {
                AABB support_part_bb(support_part.getInfillArea());
                if (boundaryBox.hit(support_part_bb))
                {
                    prevLayerOutline.add(support_part.getInfillArea()); // not intersected with skin

                    Polygons supported_skin(skinOutline.intersection(support_part.getInfillArea()));
                    if (!supported_skin.empty())
                    {
                        supportedRegions.add(supported_skin);
                    }
                }
            }
        }
    }

    if (supportThreshold > 0)
    {
        // if the proportion of the skin region that is supported is less than supportThreshold, it's considered a bridge and we
        // determine the best angle for the skin lines - the current heuristic is that the skin lines should be parallel to the
        // direction of the skin area's longest unsupported edge - if the skin has no unsupported edges, we fall through to the
        // original code

        if ((supportedRegions.area() / (skinOutline.area() + 1)) < supportThreshold)
        {
            Polygons bbPoly;
            bbPoly.add(boundaryBox.toPolygon());

            // airBelow is the region below the skin that is not supported, it extends well past the boundary of the skin.
            // It needs to be shrunk slightly so that the vertices of the skin polygon that would otherwise fall exactly on
            // the air boundary do appear to be supported

            const int bbMaxDim = std::max(boundaryBox.max.X - boundaryBox.min.X, boundaryBox.max.Y - boundaryBox.min.Y);
            const Polygons airBelow(bbPoly.offset(bbMaxDim).difference(prevLayerOutline).offset(-10));

            Polygons skinPerimeterLines;
            for (ConstPolygonRef poly : skinOutline)
            {
                Point p0 = poly[0];
                for (unsigned i = 1; i < poly.size(); ++i)
                {
                    Point p1 = poly[i];
                    skinPerimeterLines.addLine(p0, p1);
                    p0 = p1;
                }
                skinPerimeterLines.addLine(p0, poly[0]);
            }

            Polygons skinPerimeterLinesOverAir(airBelow.intersectionPolyLines(skinPerimeterLines));

            if (skinPerimeterLinesOverAir.size())
            {
                // one or more edges of the skin region are unsupported, determine the longest
                double maxDist2 = 0;
                double lineAngle = -1;
                for (PolygonRef airLine : skinPerimeterLinesOverAir)
                {
                    Point p0 = airLine[0];
                    for (unsigned i = 1; i < airLine.size(); ++i)
                    {
                        const Point& p1(airLine[i]);
                        double dist2 = vSize2(p0 - p1);
                        if (dist2 > maxDist2)
                        {
                            maxDist2 = dist2;
                            lineAngle = angle(p0 - p1);
                        }
                        p0 = p1;
                    }
                }
                return lineAngle;
            }
        }
        else
        {
            // as the proportion of the skin region that is supported is >= supportThreshold, it's not
            // considered to be a bridge and the original bridge detection code below is skipped
            return -1;
        }
    }

    if (islands.size() > 5 || islands.size() < 1)
        return -1;
    
    //Next find the 2 largest islands that we rest on.
    double area1 = 0;
    double area2 = 0;
    int idx1 = -1;
    int idx2 = -1;
    for(unsigned int n=0; n<islands.size(); n++)
    {
        //Skip internal holes
        if (!islands[n].orientation())
            continue;
        double area = fabs(islands[n].area());
        if (area > area1)
        {
            if (area1 > area2)
            {
                area2 = area1;
                idx2 = idx1;
            }
            area1 = area;
            idx1 = n;
        }else if (area > area2)
        {
            area2 = area;
            idx2 = n;
        }
    }
    
    if (idx1 < 0 || idx2 < 0)
        return -1;
    
    Point center1 = islands[idx1].centerOfMass();
    Point center2 = islands[idx2].centerOfMass();

    return angle(center2 - center1);
}

}//namespace cura

