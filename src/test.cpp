/*
 * The contnts of this file may be overwritten at any time!
 */

#include <iostream>
#include <cstdlib> // rand

#include "utils/BucketGrid2D.h"
#include "utils/intpoint.h"

using namespace cura;


#include "utils/polygon.h"
/*!
 * Test whether polygon.inside(point) returns correct results.
 */
void test_poly_inside()
{
    {
        Polygon poly;
        poly.add(Point(2000,2000));  //     /
        poly.add(Point(1000,1000));  //   / /
        poly.add(Point(1100,100));   //   |/
        
        assert (!poly.inside(Point(-2000,1000)));
        assert (poly.inside(Point(1010,1000)));
        assert (!poly.inside(Point(5000,1000)));
        assert (poly.inside(Point(1111,1100)));
        assert (!poly.inside(Point(2001,2001)));
        assert (poly.inside(Point(1999,1998)));
    }
    {
        Polygon poly;
        poly.add(Point(0,0));
        poly.add(Point(100,0));    //
        poly.add(Point(100,100));  //  |\  /|
        poly.add(Point(50,50));    //  | \/ |
        poly.add(Point(0,100));    //  |____|
        
        assert (poly.inside(Point(60,50)));
        assert (!poly.inside(Point(50,60)));
        assert (poly.inside(Point(60,40)));
        assert (poly.inside(Point(50,40)));
        assert (!poly.inside(Point(-1,100)));
        assert (!poly.inside(Point(-10,-10)));
        
    }
    {
        Polygon poly;
        poly.add(Point(   0,2000));  //   |\    .
        poly.add(Point(   0,   0));  //   | >   .
        poly.add(Point(1000,1000));  //   |/
        
        assert (poly.inside(Point(500,1000)));
        assert (poly.inside(Point(200,500)));
        assert (poly.inside(Point(200,1500)));
        assert (poly.inside(Point(800,1000)));
        assert (!poly.inside(Point(-10,1000)));
        assert (!poly.inside(Point(1100,1000)));
        assert (!poly.inside(Point(600,500)));
        assert (!poly.inside(Point(600,1500)));
        assert (!poly.inside(Point(2000,1000)));
    }
}

/*
struct LocationItem
{
    Point p;
    int i;
    LocationItem(Point p, int i) : p(p), i(i) {};
    LocationItem() : p(0,0), i(-1) {};
};
void test_BucketGrid2D()
{
    
    BucketGrid2D<LocationItem> bg(1000);
    for (int i = 0; i < 20000; i++)
    {
        Point p(rand()%100000, rand()%100000);
        LocationItem li(p, i);
        bg.insert(p, li);
    }
//     {Point p(00,00); int i = 1; bg.insert(p, i);}
//     {Point p(05,05); int i = 2; bg.insert(p, i);}
//     {Point p(14,15); int i = 3; bg.insert(p, i);}
//     {Point p(25,25); int i = 4; bg.insert(p, i);}
//     {Point p(39,39); int i = 5; bg.insert(p, i);}
//     {Point p(300,300); int i = 6; bg.insert(p, i);}
    
    Point to(rand()%100000,rand()%100000);
    std::cerr << to << std::endl;
    LocationItem result;
    if (bg.findNearestObject(to, result))
    {
        std::cerr << "best: " << result.p << std::endl;
    }
    else 
    {
        std::cerr << "nothing found!" << std::endl;
    }
    //bg.debug();
}*/


int main(int argc, char **argv)
{
    test_poly_inside();
}