
//
// polygon test
//
#include <string>
#include <cstring>
#include <vector>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <cmath>
#include <clipper/clipper.hpp>
#include "utils/intpoint.h"
#include "utils/polygon.h"

//
// currently this tests the 'area', 'centerOfMass', and 'inside'
// operations.


//using ClipperLib::IntPoint;
//using ClipperLib::cInt;

//
// This is a polgon for the test.
// All X, Y coords are in the range 0 .. 100000
// It gets transformed in various ways to increase test coverage
// (this causes the same 'close calls' to appear in different situations,
// increasing Y, decreasing Y; negative/positive values, etc)
//
static const int test_poly_points[][2] = {
    { 20621, 34081},
    { 38410, 18119}, 
    { 50082,  2321},
    { 50082, 50026},
    { 71021, 42192},
    { 92192, 73212},
    { 71021, 98306},
    { 50082, 58212},
    { 36321, 82391},
    { 28421, 82391}};
static const int N_test_poly = sizeof( test_poly_points )/ sizeof( test_poly_points[0]);

// Thes are the points to be tested
// All are either inside, or outside the polygon; but several are very
// close; also, many are quite far, but have x or y coordinates which happen to match
// points on the polygon.
//
static const int test_points_inside[][2] = {
    {28965, 26594 },        // barely inside
    {28964, 26595 },        // barely inside
    {28421, 58321},
    { 50082, 50081},
    { 50081, 50026},
    { 50083, 58212},
    { 71021, 82391},
    { 36321, 82390},
    { 36320, 82390},
    { 50081, 50023},
    { 28421, 82390},
    { 50081,  2324}     // in a corner, inside
};

static const int test_points_outside[][2] = {
    {28964, 26594 },        // barely outside
    {28421, 20123},
    { 20621, 82391},
    { 50082, 82391},
    {92192, 58212 },
    {92191, 98307 },
    {92192, 98306 },
    {92193, 98305 },
    { 36322, 82391},
    { 36322, 82392},
    { 20620, 35000},        // wholly outside at -X
    { 92194, 59000},        // wholly outside at +X
    { 50083, 50023},        // in a corner but outside
    { 50083,  2324},
    { 28420, 82390}
};
//
// The area of the test poly, as a reference
//
double RefArea;
//
// The 'center of mass', as a reference
//	
Point RefCenterOfMass;

void
findRefValues()
{
    double area_sum = 0;
    double com_sum_x = 0;
    double com_sum_y = 0;
    int n = N_test_poly;
    // choose a 'middle' reference point. This doesn't affect the result
    // except for rounding errors, which are improved when this value
    // is within, or in the neighborhood of the polgon.
    int xmid = 50000;
    int ymid = 50000;

    for( int i = 0, j= n-1; i < n; j=i, i++ )
    {
        double p0x = (double) test_poly_points[j][0] - xmid;
        double p0y = (double) test_poly_points[j][1] - ymid;
        double p1x = (double) test_poly_points[i][0] - xmid;
        double p1y = (double) test_poly_points[i][1] - ymid;
        // This calculation is twice the area of the triangle
        // formed with mid, p0, p1
        double tarea =  p0x * p1y - p0y * p1x;
        area_sum += tarea;
        // the centroid of the triangle (relative to 'mid') is
        // 1/3 the sum of p0 and p1. Make a sum of these
        // weighted by area.
        com_sum_x += (p0x + p1x)*tarea;
        com_sum_y += (p0y + p1y)*tarea;
    }

    RefArea = 0.5*area_sum;

    int com_x = xmid + int( std::floor(0.5+ com_sum_x / (3.0*area_sum) ));
    int com_y = ymid + int( std::floor(0.5+ com_sum_y / (3.0*area_sum) ));

    RefCenterOfMass = Point( com_x, com_y );
}
//
// transform a point according to a transform code
// Operations are as follows (and in the following order):
//   if bits [1:0] are 11: double X and Y
//   if bit 4 : subtract 100000 from X
//   if bit 5 : subtract 100000 from Y
//   if bit 2 : X -> -X
//   if bit 3 : Y -> -Y
//   according to bits 1,0:
//      00 nothing
//      01 swap X,y
//      10 {X,Y}  <- {X+Y, X-Y}
//      11 (nothing)
// The operations are contrived so that the change in area depends on bits 3,2,1,0 only.
//
static Point transformPoint( Point p, int transform_code )
{
    if( (transform_code & 3) == 3 )
    {
        p.X *= 2;
        p.Y *= 2;
    }
    if ( (transform_code & 0x10) != 0 ) p.X -= 100000;
    if ( (transform_code & 0x20) != 0 ) p.Y -= 100000;
    if ( (transform_code & 4) != 0 ) p.X = -p.X;
    if ( (transform_code & 8) != 0 ) p.Y = -p.Y;
    switch( (transform_code  & 3 ) )
    { 
      case 0:
        break;
      case 1:
        std::swap( p.X, p.Y );
        break;
      case 2:
        {
            Point tmp = p; // rotate 45 and scale by sqrt(2)
            p.X = tmp.X - tmp.Y;
            p.Y = tmp.X + tmp.Y;
        }
        break;
      case 3:
      default:
        break;
    }
    return p;
}

// this table defines how the area of a test polygon depends on the lowest
// 4 bits of the transform code
//
static const double transform_area_factor[16] = {
    1.0,  -1.0,  2.0,  4.0,
   -1.0,   1.0, -2.0, -4.0, 
   -1.0,   1.0, -2.0, -4.0, 
    1.0,  -1.0,  2.0,  4.0 };




int polygonTest( int xform_code)
{
    ClipperLib::Path test_path;
    int fail_count = 0;

    for( int i = 0; i < N_test_poly; i ++ )
        test_path.push_back( transformPoint( Point( test_poly_points[i][0], test_poly_points[i][1]),  xform_code) );
    cura::PolygonRef tpoly( test_path );

    // check area
    //
    double found_area = tpoly.area();
    double ref_area = RefArea * transform_area_factor[  xform_code & 15 ];
    if( found_area != ref_area ){
        printf( "** failure - transform code = 0x%X - found area of %.4f, expected %.4f\n", xform_code, found_area, ref_area );
        fail_count ++;
    }

    // check center of mass

    Point found_com = tpoly.centerOfMass();
    Point ref_com = transformPoint( RefCenterOfMass, xform_code );
    if ( std::abs(found_com.X-ref_com.X) > 1
      || std::abs(found_com.Y-ref_com.Y ) > 1 )
    {
        printf( "** failure - transform code = 0x%X - found C of Mass (%lld,%lld), expected (%lld,%lld)\n",
             xform_code, (long long)found_com.X, (long long)found_com.Y,  (long long)ref_com.X, (long long)ref_com.Y );
        fail_count ++;
    }

    // check 'inside' points.

    for( unsigned i = 0; i < sizeof(test_points_inside)/sizeof(test_points_inside[0]); i++ )
    {
        Point tpt = transformPoint( Point(test_points_inside[i][0],test_points_inside[i][1]), xform_code );
        if ( ! tpoly.inside(tpt) )
        {
            printf( "** failure - transform code = 0x%X - inside point #%u (%lld,%lld), not indicated inside\n",
                 xform_code, i, (long long)tpt.X, (long long)tpt.Y);
            fail_count ++;
        }
    }

    for( unsigned i = 0; i < sizeof(test_points_outside)/sizeof(test_points_outside[0]); i++ )
    {
        Point tpt = transformPoint( Point(test_points_outside[i][0],test_points_outside[i][1]), xform_code );
        if ( tpoly.inside(tpt) )
        {
            printf( "** failure - transform code = 0x%X - outside point #%u (%lld,%lld), indicated inside\n",
                 xform_code, i, (long long)tpt.X, (long long)tpt.Y);
            fail_count ++;
        }
    }
    return fail_count;
}

int
main(){
    findRefValues();
    int fail_count = 0;
    for(int i =0; i < 64; i++)
        fail_count += polygonTest(i);

    printf("Total %d tests failed\n", fail_count );
    return 0;
}



