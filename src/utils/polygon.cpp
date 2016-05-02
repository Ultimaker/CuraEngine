/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "polygon.h"

#include "linearAlg2D.h" // pointLiesOnTheRightOfLine

#include "../debug.h"

namespace cura 
{
    
bool PolygonRef::inside(Point p, bool border_result)
{
    PolygonRef thiss = *this;
    if (size() < 1)
    {
        return false;
    }
    
    int crossings = 0;
    Point p0 = back();
    for(unsigned int n=0; n<size(); n++)
    {
        Point p1 = thiss[n];
        // no tests unless the segment p0-p1 is at least partly at, or to right of, p.X
        short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
        if (comp == 1)
        {
            crossings++;
        }
        else if (comp == 0)
        {
            return border_result;
        }
        p0 = p1;
    }
    return (crossings % 2) == 1;
}

bool Polygons::inside(Point p, bool border_result)
{
    Polygons& thiss = *this;
    if (size() < 1)
    {
        return false;
    }
    
    int crossings = 0;
    for (PolygonRef poly : thiss)
    {
        Point p0 = poly.back();
        for(Point& p1 : poly)
        {
            short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
            if (comp == 1)
            {
                crossings++;
            }
            else if (comp == 0)
            {
                return border_result;
            }
            p0 = p1;
        }
    }
    return (crossings % 2) == 1;
}

unsigned int Polygons::findInside(Point p, bool border_result)
{
    Polygons& thiss = *this;
    if (size() < 1)
    {
        return false;
    }
    
    int64_t min_x[size()];
    std::fill_n(min_x, size(), std::numeric_limits<int64_t>::max());  // initialize with int.max
    int crossings[size()];
    std::fill_n(crossings, size(), 0);  // initialize with zeros
    
    for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
    {
        PolygonRef poly = thiss[poly_idx];
        Point p0 = poly.back();
        for(Point& p1 : poly)
        {
            short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
            if (comp == 1)
            {
                crossings[poly_idx]++;
                int64_t x;
                if (p1.Y == p0.Y)
                {
                    x = p0.X;
                }
                else 
                {
                    x = p0.X + (p1.X-p0.X) * (p.Y-p0.Y) / (p1.Y-p0.Y);
                }
                if (x < min_x[poly_idx])
                {
                    min_x[poly_idx] = x;
                }
            }
            else if (border_result && comp == 0)
            {
                return poly_idx;
            }
            p0 = p1;
        }
    }
    
    int64_t min_x_uneven = std::numeric_limits<int64_t>::max();
    unsigned int ret = NO_INDEX;
    unsigned int n_unevens = 0;
    for (unsigned int array_idx = 0; array_idx < size(); array_idx++)
    {
        if (crossings[array_idx] % 2 == 1)
        {
            n_unevens++;
            if (min_x[array_idx] < min_x_uneven)
            {
                min_x_uneven = min_x[array_idx];
                ret = array_idx;
            }
        }
    }
    if (n_unevens % 2 == 0) { ret = NO_INDEX; }
    return ret;
}

void PolygonRef::simplify(int smallest_line_segment_squared, int allowed_error_distance_squared){
    PolygonRef& thiss = *this;
    
    if (size() <= 2)
    {
        clear();
        return; 
    }
    
    { // remove segments smaller than allowed_error_distance
    // this is neccesary in order to avoid the case where a long segment is followed by a lot of small segments would get simplified to a long segment going to the wrong end point
    //  .......                _                 _______
    // |                      /                 |
    // |     would become    /    instead of    |
    // |                    /                   |
        Point* last = &thiss.back();
        unsigned int writing_idx = 0;
        for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
        {
            Point& here = thiss[poly_idx];
            if (vSize2(*last - here) < smallest_line_segment_squared)
            {
                // don't add the point
            }
            else 
            {
                thiss[writing_idx] = here;
                writing_idx++;
                last = &here;
            }
        }
        polygon->erase(polygon->begin() + writing_idx , polygon->end());
    }

    if (size() < 3)
    {
        clear();
        return;
    }

    Point* last = &thiss[0];
    unsigned int writing_idx = 1;
    for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
    {
        Point& here = thiss[poly_idx];
        if ( vSize2(here-*last) < allowed_error_distance_squared )
        {
            // don't add the point to the result
            continue;
        }
        Point& next = thiss[(poly_idx+1) % size()];
        char here_is_beyond_line = 0;
        int64_t error2 = LinearAlg2D::getDist2FromLineSegment(*last, here, next, &here_is_beyond_line);
        if (here_is_beyond_line == 0 && error2 < allowed_error_distance_squared)
        {// don't add the point to the result
        } else 
        {
            thiss[writing_idx] = here;
            writing_idx++;
            last = &here;
        }
    }
    polygon->erase(polygon->begin() + writing_idx , polygon->end());
    
            
    if (size() < 3)
    {
        clear();
        return;
    }
    
    { // handle the line segments spanning the vector end and begin
        Point* last = &thiss.back();
        Point& here = thiss[0];
        if ( vSize2(here-*last) < allowed_error_distance_squared )
        {
            remove(0);
        }
        Point& next = thiss[1];
        int64_t error2 = LinearAlg2D::getDist2FromLineSegment(*last, here, next);
        if (error2 < allowed_error_distance_squared)
        {
            remove(0);
        } else 
        {
            // leave it in
        }
    }
    
    if (size() < 3)
    {
        clear();
        return;
    }
}

std::vector<PolygonsPart> Polygons::splitIntoParts(bool unionAll) const
{
    std::vector<PolygonsPart> ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(polygons, ClipperLib::ptSubject, true);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoParts_processPolyTreeNode(&resultPolyTree, ret);
    return ret;
}

void Polygons::splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<PolygonsPart>& ret) const
{
    for(int n=0; n<node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        PolygonsPart part;
        part.add(child->Contour);
        for(int i=0; i<child->ChildCount(); i++)
        {
            part.add(child->Childs[i]->Contour);
            splitIntoParts_processPolyTreeNode(child->Childs[i], ret);
        }
        ret.push_back(part);
    }
}

unsigned int PartsView::getPartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) 
{
    PartsView& partsView = *this;
    for (unsigned int part_idx_now = 0; part_idx_now < partsView.size(); part_idx_now++)
    {
        std::vector<unsigned int>& partView = partsView[part_idx_now];
        if (partView.size() == 0) { continue; }
        std::vector<unsigned int>::iterator result = std::find(partView.begin(), partView.end(), poly_idx);
        if (result != partView.end()) 
        { 
            if (boundary_poly_idx) { *boundary_poly_idx = partView[0]; }
            return part_idx_now;
        }
    }
    return NO_INDEX;
}

PolygonsPart PartsView::assemblePart(unsigned int part_idx) 
{
    PartsView& partsView = *this;
    PolygonsPart ret;
    if (part_idx != NO_INDEX)
    {
        for (unsigned int poly_idx_ff : partsView[part_idx])
        {
            ret.add(polygons[poly_idx_ff]);
        }
    }
    return ret;
}

PolygonsPart PartsView::assemblePartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) 
{
    PolygonsPart ret;
    unsigned int part_idx = getPartContaining(poly_idx, boundary_poly_idx);
    if (part_idx != NO_INDEX)
    {
        return assemblePart(part_idx);
    }
    return ret;
}

PartsView Polygons::splitIntoPartsView(bool unionAll)
{
    Polygons reordered;
    PartsView partsView(*this);
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(polygons, ClipperLib::ptSubject, true);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoPartsView_processPolyTreeNode(partsView, reordered, &resultPolyTree);
    
    (*this) = reordered;
    return partsView;
}

void Polygons::splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Polygons& reordered, ClipperLib::PolyNode* node)
{
    for(int n=0; n<node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        partsView.emplace_back();
        unsigned int pos = partsView.size() - 1;
        partsView[pos].push_back(reordered.size());
        reordered.add(child->Contour);
        for(int i = 0; i < child->ChildCount(); i++)
        {
            partsView[pos].push_back(reordered.size());
            reordered.add(child->Childs[i]->Contour);
            splitIntoPartsView_processPolyTreeNode(partsView, reordered, child->Childs[i]);
        }
    }
}



}//namespace cura
