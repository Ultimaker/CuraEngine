
#include <stdlib.h>
#include <math.h>
#include <string.h> /* for memcpy, memmove..*/
#include <algorithm>
#include "clipper_convex_hull.h"

using ClipperLib::IntPoint;
using ClipperLib::Path;
using ClipperLib::cInt;


//
// operator to sort IntPoint by y
// (and then by X, where Y are equal)
//
struct Compare_y_lt_oper
{
    inline bool operator()(IntPoint const & a, IntPoint const & b) const
    {
        return (a.Y==b.Y)?(a.X < b.X):(a.Y < b.Y);
    }
};


//
// true if p0 -> p1 -> p2 is strictly convex.
//
static inline bool convex3(cInt x0, cInt y0, cInt x1, cInt y1, cInt x2, cInt y2)
{
    return (y1-y0)*(x1-x2) > (x0-x1)*(y2-y1);
}
static inline bool convex3(IntPoint const& p0, IntPoint const &p1, IntPoint const & p2)
{
    return convex3(p0.X, p0.Y,  p1.X, p1.Y,  p2.X, p2.Y);
}

// Find the convex hull of a set of (x,y) points. Works in-place on a Path
// After the call, 'pointset' will contain a subset of the original
// set, which form the convex hull.
// Could be:
//     - empty (if input is empty, or if there is an internal error)
//     - single point (if all input points are identical)
//     - 2 points (if >=2 input points, but all collinear)
//     - 3 or more points forming a convex polygon in CCW order.
//
// Returns 0  (or -1 if an internal
// error occurs, in which case the result will be empty).
// No case has been found in testing which causes an internal error. It's
// possible that this could occur if 64-bit multplies and adds overflow.
//
// Approach is based based on Andrew's Monotone Chain algorithm 
// ( http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
// But it builds both sides in one pass; each new point is easily determined to belong to one
// side or the other -- or neither - by comparing to lines between the ends of the partial hull
// and the 'max y' point on the same side.
//

int findConvexHullInplace(Path & pointset)
{
    int num = pointset.size();
    IntPoint * points = &pointset[0];

    if (num <= 2)
    {
        if (num == 2
                && points[0].X == points[1].X
                && points[1].Y == points[1].Y)
        {
            // remove one point if two the same
            pointset.pop_back();
        }
        return 0;
    }


    // step 1: sort the values in order from min y to max y.
    // (and increasing X where Y are equal)
    //
    std::sort(&points[0], &points[num], Compare_y_lt_oper());

    cInt miny = points[0].Y;
    cInt maxy = points[num-1].Y;

    cInt ULx, URx, DLy, DRy;
    cInt DLx, DRx;
    int nleft, nright;
    int ipos,ipos0;

    // OK next step is to find out if there's more than one identical
    // 'y' value at the end of the list. Don't to forget to
    // consider the case where all of these are identical...
    //
    {
        int numy = 1;
        ULx = URx = points[num-1].X;
        while(numy < num && points[num-(numy+1)].Y == maxy)
        {
            numy++;
        }
        // if more than one, they will be sorted by increasing X.
        // Delete any in the middle.
        //
        if (numy >=2)
        {
            int ndel = numy-2;	// always delete this many...
            URx = points[num-1].X;
            ULx = points[num-numy].X;
            if (ULx == URx) ndel++;		// delete one more if all the same...
            if (ndel > 0)
            {
                points[num-1-ndel]= points[num-1];
                num -= ndel;
            }
        }
    }
    // We may now have only 1 or 2 points.

    if (num <= 2)
    {
        pointset.resize(num);
        return 0;
    }

    // same thing at the bottoom
    {
        int numy = 1;
        DLx = DRx = points[0].X;
        while(numy < num && points[numy].Y == miny)
        {
            numy++;
        }
        // if more than one, They will be sorted left to right. Delete any in the middle.
        //
        ipos0= 1;
        if (numy >= 2)
        {
            int ndel = numy-2;	// always delete this many...
            DLx = points[0].X;
            DRx = points[numy-1].X;
            if (DLx == DRx) ndel ++;   // delete one more if all the same...
            else ipos0 = 2;			// otherwise we start with 2,
            if (ndel > 0)
            {
                memmove((void*)(points+1), (void*)(points+ndel+1), (num-(ndel+1))*sizeof(points[0]));
                num -= ndel;
            }
        }
    }
    //
    // OK, we now have the 'UL/UR' points and the 'LL/LR' points.
    // Make a left-side list and a right-side list, each
    // of capacity 'num'.
    // note that 'pleft' is in reverse order...
    // pright grows up from 0, and pleft down from 'num+1',
    // in the same array - they can't overlap.
    //
    Path temp_array(num+2);
    IntPoint *pleft  =  & temp_array[num+1];
    IntPoint *pright = & temp_array[0];

    //
    // set up left and right
    //
    pleft[0] = points[0];
    nleft = 1;
    pright[0] = points[ipos0-1];
    nright = 1;

    DLy = DRy= miny;

    for(ipos = ipos0; ipos < num; ipos++)
    {
        IntPoint newPt = points[ipos];		// get new point.

        // left side test:
        // is the new point strictly to the left of a line from (DLx, DLy) to ( ULx, maxy )?
        //
        if (convex3(ULx,maxy, newPt.X,newPt.Y, DLx,DLy))
        {
            // if so, append to the left side list, but first peel off any existing
            // points there which would be  on or inside the new line.
            //
            while(nleft > 1 && !convex3(newPt, pleft[-(nleft-1)], pleft[-(nleft-2)]))
            {
                -- nleft;
            }
            pleft[-nleft] = newPt;
            nleft++;
            DLx = newPt.X;
            DLy = newPt.Y;
        }
        // right side test
        // is the new point strictly to the right of a line from (URx, maxy) to ( DRx, DRy )?
        //
        else if (convex3(DRx, DRy, newPt.X,newPt.Y, URx, maxy))
        {
            // if so, append to the right side list, but first peel off any existing
            // points there which would be  on or inside the new line.
            //
            while(nright > 1 && !convex3(pright[nright-2], pright[nright-1], newPt))
            {
                --nright;
            }
            pright[nright] = newPt;
            nright++;
            DRx = newPt.X;
            DRy = newPt.Y;

        }
        // if neither of the above are true we throw out the point.
    }


    //
    // now add the 'maxy' points to the lists (it will have failed the insert test)
    //
    pleft[-nleft] = IntPoint(ULx, maxy);
    ++nleft;
    if (URx > ULx)
    {
        pright[nright] = IntPoint(URx, maxy);
        ++nright;
    }
    //
    // now copy the lists to the output area.
    //
    // if both lists have the same lower point, delete one now.
    // (pleft could be empty as a result!)
    //
    if (ipos0 == 1)
    {
        --pleft;
        --nleft;
    }
    // this condition should be true now...

    if (!(nleft + nright <= num))        // failure...
    {
        pointset.clear();
        return -1;
    }

    // now just pack the pright and pleft lists into the output.
    num = nleft + nright;

    memcpy((void*)points, (void *)pright, nright * sizeof(points[0]));
    if (nleft > 0)
        memcpy(&points[nright], &pleft[-(nleft-1)], nleft*sizeof(points[0]));
    if ((int)pointset.size() > num)
        pointset.resize(num);
    return 0;
}
