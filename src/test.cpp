/*
 * The contnts of this file may be overwritten at any time!
 */

#include <iostream>
#include <cstdlib> // rand



/*
#include "utils/intpoint.h"
#include "utils/polygon.h"
// Test whether polygon.inside(point) returns correct results.
void test_poly_inside_and_centerOfMass()
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

        std::cerr << "poly.centerOfMass() = " << poly.centerOfMass() << std::endl;
        Point center = poly.centerOfMass();
        for (int i = 0 ; i < 1000; i++)
        {
            Point translation(rand()%4000 - 2000, rand()%4000 - 2000);
            Polygon translated;
            for (Point& p : poly)
            {
                translated.add(p + translation);
            }
            Point translated_center = center + translation;
            if (vSize2(translated.centerOfMass() - translated_center) > 5*5)
            {
                std::cerr << "ERROR! test failed! " << std::endl;
                
                std::cerr << "translated.centerOfMass() = " << translated.centerOfMass() << std::endl;
                std::cerr << "translated_center = " << translated_center << std::endl;
            }
        }
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
        
        std::cerr << "poly.centerOfMass() = " << poly.centerOfMass() << std::endl;
        Point center = poly.centerOfMass();
        for (int i = 0 ; i < 1000; i++)
        {
            Point translation(rand()%4000 - 2000, rand()%4000 - 2000);
            Polygon translated;
            for (Point& p : poly)
            {
                translated.add(p + translation);
            }
            Point translated_center = center + translation;
            if (vSize2(translated.centerOfMass() - translated_center) > 5*5)
            {
                std::cerr << "ERROR! test failed! " << std::endl;
                
                std::cerr << "translated.centerOfMass() = " << translated.centerOfMass() << std::endl;
                std::cerr << "translated_center = " << translated_center << std::endl;
            }
        }
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
        
        std::cerr << "poly.centerOfMass() = " << poly.centerOfMass() << std::endl;
        Point center = poly.centerOfMass();
        for (int i = 0 ; i < 1000; i++)
        {
            Point translation(rand()%4000 - 2000, rand()%4000 - 2000);
            Polygon translated;
            for (Point& p : poly)
            {
                translated.add(translation - Point(-p.X, p.Y));
            }
            Point translated_center = translation - Point(-center.X, center.Y);
            if (vSize2(translated.centerOfMass() - translated_center) > 5*5)
            {
                std::cerr << "ERROR! test failed! " << std::endl;
                std::cerr << "translated.centerOfMass() = " << translated.centerOfMass() << std::endl;
                std::cerr << "translated_center = " << translated_center << std::endl;
            }
        }
    }
}*/

/*
struct LocationItem
{
    Point p;
    int i;
    LocationItem(Point p, int i) : p(p), i(i) {};
    LocationItem() : p(0,0), i(-1) {};
};

#include "utils/BucketGrid2D.h"
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

/*
#include <math.h> 
#include "utils/gettime.h"
#include "utils/polygonUtils.h"

void test_findClosestConnection()
{
    srand(1234);
    if (false)
    {
        Polygon poly2;
        poly2.add(Point(0,300));
        poly2.add(Point(100,300));    //   ____
        poly2.add(Point(100,200));    //  |    |
        poly2.add(Point(50,250));     //  | /\ |
        poly2.add(Point(0,200));      //  |/  \|
        
        Polygon poly1;
        poly1.add(Point(0,0));
        poly1.add(Point(100,0));    //
        poly1.add(Point(100,100));  //  |\  /|
        poly1.add(Point(50,50));    //  | \/ |
        poly1.add(Point(0,100));    //  |____|
        
        ClosestPolygonPoint result1 (poly1);
        ClosestPolygonPoint result2 (poly2);
        
        findSmallestConnection(result1, result2, 3);
        std::cerr << result1.location << " -- " << result2.location << std::endl;
    }
    
    if (false)
    {
        Polygon poly2;
        poly2.add(Point(0,300));
        poly2.add(Point(100,300));    //   ____
        poly2.add(Point(100,200));    //  |    |
        poly2.add(Point(50,250));     //  | /\ |
        poly2.add(Point(10,105));     //  |/  \|
        
        Polygon poly1;
        poly1.add(Point(0,0));
        poly1.add(Point(100,0));    //
        poly1.add(Point(100,100));  //  |\  /|
        poly1.add(Point(50,50));    //  | \/ |
        poly1.add(Point(0,100));    //  |____|
        
        ClosestPolygonPoint result1 (poly1);
        ClosestPolygonPoint result2 (poly2);
        
        findSmallestConnection(result1, result2, 3);
        std::cerr << result1.location << " -- " << result2.location << std::endl;
    }
    
    double creationTime = 0;
    double evalTime = 0;
    long totalLength = 0;
    TimeKeeper timer;
    for (int i = 0; i < 10000; i++)
    { // for vizualization as csv with e.g. Rstudio
        Polygon poly1;
        double dist = 100;
        for (double a = 0; a < 360; a += 1)
        {
            dist += int(rand()%3) -1;
            Point p(static_cast<int>(dist * std::cos(a/180.0*3.1415)), static_cast<int>(dist * std::sin(a/180.0*3.1415)));
            p = p + Point(0, 200);
            if ( a ==0)
                poly1.add(p);
            else 
                poly1.add((poly1.back() + p) / 2);
//             std::cerr << poly1.back().X << ", " << poly1.back().Y << std::endl;
        }
//         std::cerr << " " << std::endl;
        Polygon poly2;
        dist = 100;
        for (double a = 0; a < 360; a += 1)
        {
            
            dist += int(rand()%3) - 1;
            Point p(static_cast<int>(dist * std::cos(a/180.0*3.1415)), static_cast<int>(dist * std::sin(a/180.0*3.1415)));
            if ( a ==0)
                poly2.add(p);
            else 
                poly2.add((poly2.back() + p) / 2);
//             std::cerr << poly2.back().X << ", " << poly2.back().Y << std::endl;
        }
        creationTime += timer.restart();
        ClosestPolygonPoint result1 (poly1);
        ClosestPolygonPoint result2 (poly2);
        
        findSmallestConnection(result1, result2, 240);
        totalLength += vSize(result1.location - result2.location);
        evalTime += timer.restart();
//         std::cerr << " " << std::endl;
//         std::cerr << result1.location.X  << " , " << result1.location.Y << std::endl;
//         std::cerr << result2.location.X  << " , " << result2.location.Y << std::endl;
//         std::cerr << " " << std::endl;
    }
    
    std::cerr << "creationTime : " << creationTime << std::endl;
    std::cerr << "evalTime : " << evalTime << std::endl;
    std::cerr << "totalLength : " << totalLength << std::endl;
}
*/


#include "utils/polygon.h"
using namespace cura;

void test_clipper()
{
    Polygon p;
    p.emplace_back(0, 11004);
    p.emplace_back(0, 10129);
    p.emplace_back(0, 9185);
    p.emplace_back(0, 8477);
    p.emplace_back(1, 8491);
    p.emplace_back(418, 8861);
    p.emplace_back(1080, 9389);
    p.emplace_back(2106, 10142);
    p.emplace_back(3000, 10757);
    p.emplace_back(3000, 12010);
    p.emplace_back(3000, 12790);
    p.emplace_back(3000, 13485);
    p.emplace_back(3000, 14088);
    p.emplace_back(3000, 14601);
    p.emplace_back(3000, 15354);
    p.emplace_back(3000, 24867);
    p.emplace_back(3000, 25469);
    p.emplace_back(3000, 26303);
    p.emplace_back(3000, 27421);
    p.emplace_back(3000, 28242);
    p.emplace_back(2107, 28856);
    p.emplace_back(1080, 29610);
    p.emplace_back(608, 29986);
    p.emplace_back(1, 30508);
    p.emplace_back(1, 30522);
    p.emplace_back(0, 11772);

    Polygons polys;
    polys.add(p);
    
//     polys.debugOutputHTML("output/problem_test.html", true);
//     polys.offset(-400).debugOutputHTML("output/problem_test_offset.html", true);
    polys = polys.removeDegenerateVerts();
//     polys.offset(-400).debugOutputHTML("output/problem_test_offset_solved.html", true);
}
int main(int argc, char **argv)
{
//     test_findClosestConnection();
    test_clipper();
}