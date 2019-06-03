
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>

#include <boost/version.hpp>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "VoronoiQuadrangulation.h"
#include "DistributedBeadingStrategy.h"
#include "utils/VoronoiUtils.h"
#include "NaiveBeadingStrategy.h"

using arachne::Point;

namespace arachne
{

Polygons generateTestPoly(int size, Point border)
{
    Polygons polys;
    PolygonRef poly = polys.newPoly();
    for (int i = 0; i < size; i++)
    {
        poly.emplace_back(rand() % border.X, rand() % border.Y);
    }
    
    polys = polys.unionPolygons(Polygons(), ClipperLib::pftNegative);
//     polys = polys.offset(border.X*1.2, ClipperLib::jtRound);
    
//     polys = polys.offset(border.X*2, ClipperLib::jtRound);
//     polys = polys.offset(-border.X*1.8, ClipperLib::jtRound);
    
//     polys = polys.offset(-10, ClipperLib::jtRound);
//     polys = polys.offset(20, ClipperLib::jtRound);
//     polys = polys.offset(-10, ClipperLib::jtRound);
    polys = polys.offset(-border.X/200, ClipperLib::jtRound);
    polys = polys.offset(border.X/100, ClipperLib::jtRound);
    polys = polys.offset(-border.X/200, ClipperLib::jtRound);
    polys = polys.unionPolygons();
    return polys;
}

static Polygons test_poly_1;
static Polygons squares;
static Polygons parabola_dip;
static Polygons circle;
static Polygons circle_flawed;
static Polygons gMAT_example;
static Polygons wedge;
static Polygons flawed_wedge;
static Polygons rounded_wedge;
static Polygons flawed_wall;
static Polygons marked_local_opt;
static Polygons pikachu;

void generateTestPolys()
{
    PolygonRef poly = test_poly_1.newPoly();
    poly.emplace_back(0, 0);
    poly.emplace_back(10000, 0);
    poly.emplace_back(5000, 1000);
    poly.emplace_back(4000, 2000);
    poly.emplace_back(3000, 5000);
    poly.emplace_back(2000, 6000);
    poly.emplace_back(1000, 5000);
    poly.emplace_back(0, 3000);
    PolygonRef hole = test_poly_1.newPoly();
    hole.emplace_back(1000,1000);
    hole.emplace_back(1100,900);
    hole.emplace_back(1000,900);

    PolygonRef square_1 = squares.newPoly();
    square_1.emplace_back(0, 0);
    square_1.emplace_back(0, 10000);
    square_1.emplace_back(10000, 10000);
    square_1.emplace_back(10000, 0);
    
    PolygonRef parabola_dip_1 = parabola_dip.newPoly();
    parabola_dip_1.emplace_back(0, 10000);
    parabola_dip_1.emplace_back(0, 0);
    parabola_dip_1.emplace_back(10000, 0);
    parabola_dip_1.emplace_back(10000, 10000);
    parabola_dip_1.emplace_back(5500, 10000);
    parabola_dip_1.emplace_back(5000, 5000);
    parabola_dip_1.emplace_back(4500, 10000);
    Point3Matrix rot = Point3Matrix(PointMatrix(25.0)).compose(PointMatrix::scale(.7));
    parabola_dip_1.applyMatrix(rot);
    
    PolygonRef circle_1 = circle.newPoly();
    coord_t r = 10000;
    for (float a = 0; a < 360; a += 10)
    {
        float rad = a / 180 * M_PI;
        circle_1.emplace_back(r * cos(rad), r * sin(rad));
    }
    
    PolygonRef circle_flawed_1 = circle_flawed.newPoly();
    for (float a = 0; a < 360; a += 10)
    {
        r = 5000 + rand() % 2500;
        a += (rand() % 100) / 50.0;
        float rad = a / 180 * M_PI;
        circle_flawed_1.emplace_back(r * cos(rad), r * sin(rad));
    }

    PolygonRef gMAT_example_outline = gMAT_example.newPoly();
    gMAT_example_outline.emplace_back(0, 0);
    gMAT_example_outline.emplace_back(8050, 0);
    gMAT_example_outline.emplace_back(8050, 2000);
    gMAT_example_outline.emplace_back(7000, 2000);
    gMAT_example_outline.emplace_back(7000, 11500);
    gMAT_example_outline.emplace_back(6500, 12000);
    gMAT_example_outline.emplace_back(0, 12000);
    PolygonRef gMAT_example_triangle = gMAT_example.newPoly();
    gMAT_example_triangle.emplace_back(1000, 7000);
    gMAT_example_triangle.emplace_back(1000, 11000);
    gMAT_example_triangle.emplace_back(4000, 9000);
    PolygonRef gMAT_example_round = gMAT_example.newPoly();
    gMAT_example_round.emplace_back(1000, 3000);
    gMAT_example_round.emplace_back(1000, 5000);
    gMAT_example_round.emplace_back(2000, 6000);
    gMAT_example_round.emplace_back(5000, 6000);
    gMAT_example_round.emplace_back(5000, 3000);
    gMAT_example_round.emplace_back(4000, 2000);
    gMAT_example_round.emplace_back(2000, 2000);

    PolygonRef wedge_1 = wedge.newPoly();
    wedge_1.emplace_back(2500, 0);
    wedge_1.emplace_back(0, 2500);
    wedge_1.emplace_back(20000, 20000);
    PointMatrix scaler = PointMatrix::scale(.846); // .846 causes a transition which is just beyond the marked skeleton
    wedge_1.applyMatrix(scaler);

    rounded_wedge = wedge.offset(-400, ClipperLib::jtRound).offset(400, ClipperLib::jtRound); // TODO: this offset gives problems!!
//     rounded_wedge = wedge.offset(-200, ClipperLib::jtRound).offset(200, ClipperLib::jtRound); // TODO: this offset also gives problems!!
//     rounded_wedge = wedge.offset(-205, ClipperLib::jtRound).offset(205, ClipperLib::jtRound);
    
    {
        coord_t l = 20000;
        coord_t h = 2000;
        coord_t r = 200;
        coord_t step = 2000;
        PolygonRef flawed_wedgel_1 = flawed_wedge.newPoly();
        for (coord_t x = 0; x <= l; x += step)
        {
            flawed_wedgel_1.emplace_back(x, (h + rand() % r - r/2) * x / l);
        }
        for (coord_t x = l - step / 2; x >= 0; x -= 800)
        {
            flawed_wedgel_1.emplace_back(x, (rand() % r - r/2) * x / l);
        }
        
        Point3Matrix rot = Point3Matrix(PointMatrix(60.0));
        flawed_wedgel_1.applyMatrix(rot);
    }
    {
        coord_t l = 10000;
        coord_t h = 1000;
        coord_t r = 100;
        coord_t step = 2000;
        PolygonRef flawed_wall_1 = flawed_wall.newPoly();
        for (coord_t x = 0; x <= l; x += step)
        {
            flawed_wall_1.emplace_back(x, h + rand() % r - r/2);
        }
        for (coord_t x = l - step / 2; x >= 0; x -= 800)
        {
            flawed_wall_1.emplace_back(x, rand() % r - r/2);
        }
        
        Point3Matrix rot = Point3Matrix(PointMatrix(60.0));
        flawed_wall_1.applyMatrix(rot);
    }
    {
        PolygonRef marked_local_opt_1 = marked_local_opt.newPoly();
        marked_local_opt_1.emplace_back(5000, 0);
        marked_local_opt_1.emplace_back(0, 400);
        marked_local_opt_1.emplace_back(5000, 610);
        marked_local_opt_1.emplace_back(10000, 400);
        Point3Matrix rot = Point3Matrix(PointMatrix(60.0));
        marked_local_opt_1.applyMatrix(rot);
    }
    {
        PolygonRef pika = pikachu.newPoly();
        pika.emplace_back(100 * 89.906037, 100 * 139.3808);
        pika.emplace_back(pika.back() + Point(100 * -1.17234, 100 * -1.41312 ));
        pika.emplace_back(pika.back() + Point(100 * -0.651263, 100 * -1.75642));
        pika.emplace_back(pika.back() + Point(100 * -0.372023, 100 * -1.25273));
        pika.emplace_back(pika.back() + Point(100 * -1.654167, 100 * -0.13889));
        pika.emplace_back(pika.back() + Point(100 * -1.550094, 100 * -0.26835));
        pika.emplace_back(pika.back() + Point(100 * -1.530072, 100 * -0.37838));
        pika.emplace_back(pika.back() + Point(100 * -1.509569, 100 * -0.44787));
        pika.emplace_back(pika.back() + Point(100 * -1.731946, 100 * -0.53279));
        pika.emplace_back(pika.back() + Point(100 * -1.761748, 100 * -0.42292));
        pika.emplace_back(pika.back() + Point(100 * -1.500614, 100 * 0.3581  ));
        pika.emplace_back(pika.back() + Point(100 * -1.45293, 100 * 0.53635  ));
        pika.emplace_back(pika.back() + Point(100 * -1.354326, 100 * 0.72786 ));
        pika.emplace_back(pika.back() + Point(100 * -0.383074, 100 * 1.67233 ));
        pika.emplace_back(pika.back() + Point(100 * -0.334124, 100 * 1.42244 ));
        pika.emplace_back(pika.back() + Point(100 * -0.926562, 100 * 0.9178  ));
        pika.emplace_back(pika.back() + Point(100 * -0.583689, 100 * -0.20687));
        pika.emplace_back(pika.back() + Point(100 * -0.785806, 100 * -1.66451));
        pika.emplace_back(pika.back() + Point(100 * -0.474324, 100 * -1.27245));
        pika.emplace_back(pika.back() + Point(100 * -1.113176, 100 * -0.70659));
        pika.emplace_back(pika.back() + Point(100 * -1.479306, 100 * -0.83035));
        pika.emplace_back(pika.back() + Point(100 * -1.206837, 100 * -1.2014 ));
        pika.emplace_back(pika.back() + Point(100 * -0.847616, 100 * -1.4725 ));
        pika.emplace_back(pika.back() + Point(100 * -0.257186, 100 * -1.50221));
        pika.emplace_back(pika.back() + Point(100 * -0.146191, 100 * -1.517  ));
        pika.emplace_back(pika.back() + Point(100 * -0.09291, 100 * -1.52242 ));
        pika.emplace_back(pika.back() + Point(100 * -0.10502, 100 * -1.87834 ));
        pika.emplace_back(pika.back() + Point(100 * -0.209815, 100 * -1.86798));
        pika.emplace_back(pika.back() + Point(100 * -1.641724, 100 * -0.37185));
        pika.emplace_back(pika.back() + Point(100 * -1.678959, 100 * -0.20223));
        pika.emplace_back(pika.back() + Point(100 * -1.8653, 100 * -0.23802  ));
        pika.emplace_back(pika.back() + Point(100 * -1.854564, 100 * -0.31673));
        pika.emplace_back(pika.back() + Point(100 * -1.835595, 100 * -0.40956));
        pika.emplace_back(pika.back() + Point(100 * -1.623536, 100 * -0.42455));
        pika.emplace_back(pika.back() + Point(100 * -1.633699, 100 * -0.37567));
        pika.emplace_back(pika.back() + Point(100 * -0.859896, 100 * -0.63156));
        pika.emplace_back(pika.back() + Point(100 * -0.282621, 100 * -0.65928));
        pika.emplace_back(pika.back() + Point(100 * -0.136582, 100 * -0.55524));
        pika.emplace_back(pika.back() + Point(100 * -0.1804, 100 * -0.91396  ));
        pika.emplace_back(pika.back() + Point(100 * 0.138109, 100 * -0.79729 ));
        pika.emplace_back(pika.back() + Point(100 * 0.302046, 100 * -0.91136 ));
        pika.emplace_back(pika.back() + Point(100 * 0.536435, 100 * -0.64746 ));
        pika.emplace_back(pika.back() + Point(100 * 0.998381, 100 * -0.22873 ));
        pika.emplace_back(pika.back() + Point(100 * 0.948532, 100 * -0.39421 ));
        pika.emplace_back(pika.back() + Point(100 * 1.386568, 100 * -0.62048 ));
        pika.emplace_back(pika.back() + Point(100 * 1.416049, 100 * -0.55126 ));
        pika.emplace_back(pika.back() + Point(100 * 1.440422, 100 * -0.48223 ));
        pika.emplace_back(pika.back() + Point(100 * -0.831226, 100 * -1.62449));
        pika.emplace_back(pika.back() + Point(100 * -1.072123, 100 * -1.48697));
        pika.emplace_back(pika.back() + Point(100 * -0.791195, 100 * -1.64785));
        pika.emplace_back(pika.back() + Point(100 * 0.02221, 100 * -1.81249  ));
        pika.emplace_back(pika.back() + Point(100 * 0.691467, 100 * -1.69621 ));
        pika.emplace_back(pika.back() + Point(100 * 0.559776, 100 * -1.39386 ));
        pika.emplace_back(pika.back() + Point(100 * 0.393299, 100 * -1.45129 ));
        pika.emplace_back(pika.back() + Point(100 * 0.379781, 100 * -1.37644 ));
        pika.emplace_back(pika.back() + Point(100 * 0.561121, 100 * -1.310563));
        pika.emplace_back(pika.back() + Point(100 * 0.506817, 100 * -1.226285));
        pika.emplace_back(pika.back() + Point(100 * 0.162126, 100 * -1.316944));
        pika.emplace_back(pika.back() + Point(100 * 0.11911, 100 * -1.943668 ));
        pika.emplace_back(pika.back() + Point(100 * 0.28763, 100 * -1.927392 ));
        pika.emplace_back(pika.back() + Point(100 * 0.41164, 100 * -1.904537 ));
        pika.emplace_back(pika.back() + Point(100 * 0.517435, 100 * -1.877348));
        pika.emplace_back(pika.back() + Point(100 * 0.621578, 100 * -1.847328));
        pika.emplace_back(pika.back() + Point(100 * 0.737778, 100 * -1.803356));
        pika.emplace_back(pika.back() + Point(100 * 0.893256, 100 * -1.730373));
        pika.emplace_back(pika.back() + Point(100 * 1.168369, 100 * -1.550458));
        pika.emplace_back(pika.back() + Point(100 * 0.766054, 100 * -0.733927));
        pika.emplace_back(pika.back() + Point(100 * 0.212517, 100 * 0.763617 ));
        pika.emplace_back(pika.back() + Point(100 * 0.164136, 100 * 1.628567 ));
        pika.emplace_back(pika.back() + Point(100 * 0.01938, 100 * 1.639965  ));
        pika.emplace_back(pika.back() + Point(100 * -0.04089, 100 * 1.639888 ));
        pika.emplace_back(pika.back() + Point(100 * -0.09884, 100 * 1.637984 ));
        pika.emplace_back(pika.back() + Point(100 * -0.204231, 100 * 1.625546));
        pika.emplace_back(pika.back() + Point(100 * -0.388633, 100 * 1.645484));
        pika.emplace_back(pika.back() + Point(100 * -0.458392, 100 * 1.626348));
        pika.emplace_back(pika.back() + Point(100 * -0.419818, 100 * 1.791148));
        pika.emplace_back(pika.back() + Point(100 * 1.931639, 100 * -0.320108));
        pika.emplace_back(pika.back() + Point(100 * 1.901214, 100 * -0.264146));
        pika.emplace_back(pika.back() + Point(100 * 1.919632, 100 * 0.0063   ));
        pika.emplace_back(pika.back() + Point(100 * 1.906742, 100 * 0.231255 ));
        pika.emplace_back(pika.back() + Point(100 * 1.861582, 100 * 0.471016 ));
        pika.emplace_back(pika.back() + Point(100 * 1.531013, 100 * 0.513408 ));
        pika.emplace_back(pika.back() + Point(100 * 1.114821, 100 * -0.750746));
        pika.emplace_back(pika.back() + Point(100 * 1.57013, 100 * -0.982223 ));
        pika.emplace_back(pika.back() + Point(100 * 1.628749, 100 * -0.88099 ));
        pika.emplace_back(pika.back() + Point(100 * 1.678266, 100 * -0.783038));
        pika.emplace_back(pika.back() + Point(100 * 1.720632, 100 * -0.684362));
        pika.emplace_back(pika.back() + Point(100 * 1.758509, 100 * -0.581617));
        pika.emplace_back(pika.back() + Point(100 * 1.792764, 100 * -0.470255));
        pika.emplace_back(pika.back() + Point(100 * 1.825014, 100 * -0.330236));
        pika.emplace_back(pika.back() + Point(100 * 1.842886, 100 * -0.208967));
        pika.emplace_back(pika.back() + Point(100 * 1.85238, 100 * -0.108083 ));
        pika.emplace_back(pika.back() + Point(100 * 1.854223, 100 * 0.01549  ));
        pika.emplace_back(pika.back() + Point(100 * 1.715226, 100 * 0.472457 ));
        pika.emplace_back(pika.back() + Point(100 * -1.113248, 100 * 1.466724));
        pika.emplace_back(pika.back() + Point(100 * -1.362053, 100 * 1.265622));
        pika.emplace_back(pika.back() + Point(100 * -1.451254, 100 * 1.164803));
        pika.emplace_back(pika.back() + Point(100 * -1.424331, 100 * 0.965091));
        pika.emplace_back(pika.back() + Point(100 * -1.488395, 100 * 0.859553));
        pika.emplace_back(pika.back() + Point(100 * -1.538727, 100 * 0.76777 ));
        pika.emplace_back(pika.back() + Point(100 * -1.580297, 100 * 0.67842 ));
        pika.emplace_back(pika.back() + Point(100 * -1.618064, 100 * 0.583667));
        pika.emplace_back(pika.back() + Point(100 * -1.652798, 100 * 0.472605));
        pika.emplace_back(pika.back() + Point(100 * -1.294976, 100 * 0.319042));
        pika.emplace_back(pika.back() + Point(100 * -1.292115, 100 * 0.331543));
        pika.emplace_back(pika.back() + Point(100 * 0.217671, 100 * 0.800681 ));
        pika.emplace_back(pika.back() + Point(100 * 0.492364, 100 * 1.82909  ));
        pika.emplace_back(pika.back() + Point(100 * 0.416273, 100 * 1.84813  ));
        pika.emplace_back(pika.back() + Point(100 * 0.375498, 100 * 1.57701  ));
        pika.emplace_back(pika.back() + Point(100 * 0.471525, 100 * 1.55097  ));
        pika.emplace_back(pika.back() + Point(100 * 0.588764, 100 * 1.51058  ));
        pika.emplace_back(pika.back() + Point(100 * 0.711763, 100 * 1.45549  ));
        pika.emplace_back(pika.back() + Point(100 * 0.823914, 100 * 1.39673  ));
        pika.emplace_back(pika.back() + Point(100 * 0.912204, 100 * 1.34034  ));
        pika.emplace_back(pika.back() + Point(100 * 0.976721, 100 * 1.29445  ));
        pika.emplace_back(pika.back() + Point(100 * 1.020174, 100 * 1.25803  ));
        pika.emplace_back(pika.back() + Point(100 * 1.073314, 100 * 1.29718  ));
        pika.emplace_back(pika.back() + Point(100 * 1.054554, 100 * 1.3101   ));
        pika.emplace_back(pika.back() + Point(100 * 1.017591, 100 * 1.34143  ));
        pika.emplace_back(pika.back() + Point(100 * 0.955575, 100 * 1.38458  ));
        pika.emplace_back(pika.back() + Point(100 * 0.886809, 100 * 1.34655  ));
        pika.emplace_back(pika.back() + Point(100 * 0.835136, 100 * -0.43401 ));
        pika.emplace_back(pika.back() + Point(100 * 0.835136, 100 * -0.43402 ));
        pika.emplace_back(pika.back() + Point(100 * -1.040513, 100 * -1.134  ));
        pika.emplace_back(pika.back() + Point(100 * -1.040514, 100 * -1.134  ));
        pika.emplace_back(pika.back() + Point(100 * -1.040513, 100 * -1.134  ));
        pika.emplace_back(pika.back() + Point(100 * -1.040512, 100 * -1.134  ));
        pika.emplace_back(pika.back() + Point(100 * 1.181456, 100 * -0.88495 ));
        pika.emplace_back(pika.back() + Point(100 * 1.181456, 100 * -0.88494 ));
        pika.emplace_back(pika.back() + Point(100 * 1.235015, 100 * -0.96488 ));
        pika.emplace_back(pika.back() + Point(100 * 1.127897, 100 * -1.07321 ));
        pika.emplace_back(pika.back() + Point(100 * -0.420596, 100 * -1.06032));
        pika.emplace_back(pika.back() + Point(100 * -0.497292, 100 * -1.03055));
        pika.emplace_back(pika.back() + Point(100 * -0.771421, 100 * -1.62024));
        pika.emplace_back(pika.back() + Point(100 * -0.727889, 100 * -1.64006));
        pika.emplace_back(pika.back() + Point(100 * -0.693152, 100 * -1.65568));
        pika.emplace_back(pika.back() + Point(100 * -0.593817, 100 * -1.69097));
        pika.emplace_back(pika.back() + Point(100 * 0.857558, 100 * -1.35682 ));
        pika.emplace_back(pika.back() + Point(100 * 1.022775, 100 * -1.2532  ));
        pika.emplace_back(pika.back() + Point(100 * 1.058852, 100 * -1.2213  ));
        pika.emplace_back(pika.back() + Point(100 * 1.081476, 100 * -1.201541));
        pika.emplace_back(pika.back() + Point(100 * 1.099893, 100 * -1.186193));
        pika.emplace_back(pika.back() + Point(100 * 1.114853, 100 * -1.170043));
        pika.emplace_back(pika.back() + Point(100 * 1.131305, 100 * -1.154942));
        pika.emplace_back(pika.back() + Point(100 * 1.148545, 100 * -1.13708 ));
        pika.emplace_back(pika.back() + Point(100 * 1.224743, 100 * -1.182268));
        pika.emplace_back(pika.back() + Point(100 * 1.236949, 100 * -1.171757));
        pika.emplace_back(pika.back() + Point(100 * 1.25026, 100 * -1.155192 ));
        pika.emplace_back(pika.back() + Point(100 * 1.2716, 100 * -1.13238   ));
        pika.emplace_back(pika.back() + Point(100 * 1.30931, 100 * -1.08899  ));
        pika.emplace_back(pika.back() + Point(100 * 1.43967, 100 * -0.790679 ));
        pika.emplace_back(pika.back() + Point(100 * 0.48008, 100 * 1.555496  ));
        pika.emplace_back(pika.back() + Point(100 * 0.37261, 100 * 1.588004  ));
        pika.emplace_back(pika.back() + Point(100 * 0.34265, 100 * 1.594033  ));
        pika.emplace_back(pika.back() + Point(100 * 0.32304, 100 * 1.599108  ));
        pika.emplace_back(pika.back() + Point(100 * 0.30542, 100 * 1.602782  ));
        pika.emplace_back(pika.back() + Point(100 * 0.28523, 100 * 1.604608  ));
        pika.emplace_back(pika.back() + Point(100 * 0.25606, 100 * 1.611349  ));
        pika.emplace_back(pika.back() + Point(100 * 0.17132, 100 * 1.620775  ));
        pika.emplace_back(pika.back() + Point(100 * -1.35971, 100 * 0.96551  ));
        pika.emplace_back(pika.back() + Point(100 * -1.39922, 100 * 0.91483  ));
        pika.emplace_back(pika.back() + Point(100 * -1.40723, 100 * 0.90216  ));
        pika.emplace_back(pika.back() + Point(100 * -1.4113, 100 * 0.89234   ));
        pika.emplace_back(pika.back() + Point(100 * -1.45593, 100 * 0.92397  ));
        pika.emplace_back(pika.back() + Point(100 * -1.446338, 100 * 0.93799 ));
        pika.emplace_back(pika.back() + Point(100 * -1.434825, 100 * 0.9591  ));
        pika.emplace_back(pika.back() + Point(100 * -1.375387, 100 * 1.03297 ));
        pika.emplace_back(pika.back() + Point(100 * 0.08292, 100 * 1.9504    ));
        pika.emplace_back(pika.back() + Point(100 * 0.206846, 100 * 1.94581  ));
        pika.emplace_back(pika.back() + Point(100 * 0.292504, 100 * 1.93242  ));
        pika.emplace_back(pika.back() + Point(100 * -0.738556, 100 * 1.22525 ));
        pika.emplace_back(pika.back() + Point(100 * -1.123489, 100 * 0.97589 ));
        pika.emplace_back(pika.back() + Point(100 * -1.030391, 100 * 0.88165 ));
        pika.emplace_back(pika.back() + Point(100 * -0.950762, 100 * 0.95686 ));
        pika.emplace_back(pika.back() + Point(100 * 0.69182, 100 * 1.09457   ));
        pika.emplace_back(pika.back() + Point(100 * 0.763389, 100 * 1.05027  ));
        pika.emplace_back(pika.back() + Point(100 * 0.758719, 100 * 1.03952  ));
        pika.emplace_back(pika.back() + Point(100 * 0.696489, 100 * 1.07749  ));
        pika.emplace_back(pika.back() + Point(100 * -1.096689, 100 * 0.58711 ));
        pika.emplace_back(pika.back() + Point(100 * -1.152269, 100 * 0.48632 ));
        pika.emplace_back(pika.back() + Point(100 * -1.175455, 100 * 0.51325 ));
        pika.emplace_back(pika.back() + Point(100 * -1.073504, 100 * 0.67162 ));
        pika.emplace_back(pika.back() + Point(100 * 0.926042, 100 * 1.18094  ));
        pika.emplace_back(pika.back() + Point(100 * 0.926042, 100 * 1.07716  ));
        pika.emplace_back(pika.back() + Point(100 * -0.755256, 100 * 0.33066 ));
        pika.emplace_back(pika.back() + Point(100 * -0.748826, 100 * 0.5623  ));
        pika.emplace_back(pika.back() + Point(100 * -0.154493, 100 * 0.99212 ));
        pika.emplace_back(pika.back() + Point(100 * -0.265942, 100 * 1.36391 ));
        pika.emplace_back(pika.back() + Point(100 * -0.434011, 100 * 1.31818 ));
        pika.emplace_back(pika.back() + Point(100 * -0.470906, 100 * 1.87155 ));
        pika.emplace_back(pika.back() + Point(100 * -0.03421, 100 * 1.93445  ));
        pika.emplace_back(pika.back() + Point(100 * 0.198152, 100 * 1.92659  ));
        pika.emplace_back(pika.back() + Point(100 * -0.08296, 100 * 1.91823  ));
        pika.emplace_back(pika.back() + Point(100 * -1.140304, 100 * 0.19548 ));
    }
}

void test()
{
    
    // Preparing Input Geometries.
    int r;
    r = 1558617038;
    r = time(0);
//     r = 1558618076;
//     r = 1558692831;
//     r = 1558983814;
//     r = 1558985782;
//     r = 1559215562;
//     r = 1559224125;
//     r = 1559224469;
//     r = 68431;
//     r = 1559234570;
    r = 1559564752;
    srand(r);
    printf("r = %d;\n", r);
    fflush(stdout);
    logDebug("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    generateTestPolys();
    Polygons polys = generateTestPoly(6, Point(10000, 10000));
//     Polygons polys = test_poly_1;
//     Polygons polys = parabola_dip;
//     Polygons polys = squares;
//     Polygons polys = circle;
//     Polygons polys = circle_flawed;
//     Polygons polys = gMAT_example;
//     Polygons polys = wedge;
//     Polygons polys = flawed_wedge;
//     Polygons polys = flawed_wall;
//     Polygons polys = marked_local_opt;
//     Polygons polys = pikachu;
    polys = polys.unionPolygons();
    {
        SVG svg("output/outline.svg", AABB(Point(0,0), Point(10000, 10000)));
        svg.writePolygons(polys);
    }
    {
        SVG svg("output/normal.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::RED, 2);
        Polygons insets;
        Polygons last_inset = polys.offset(-200);
        while (!last_inset.empty())
        {
            insets.add(last_inset);
            last_inset = last_inset.offset(-400);
        }
        for (PolygonRef poly : insets)
        {
            Point prev = poly.back();
            for (Point p : poly)
            {
                ExtrusionSegment segment(prev, 400, p, 400);
                svg.writeAreas(segment.toPolygons(), SVG::Color::GRAY);;
                prev = p;
            }
        }
        svg.writePolygons(insets, SVG::Color::WHITE, 2);
    }
    
    
    TimeKeeper tk;
    
    VoronoiQuadrangulation vq(polys);

    DistributedBeadingStrategy beading_strategy(300, 400, 600);
//     NaiveBeadingStrategy beading_strategy(400);
    std::vector<ExtrusionSegment> segments = vq.generateToolpaths(beading_strategy);
    Polygons paths;
    for (ExtrusionSegment& segment : segments)
    {
        PolygonRef poly = paths.newPoly();
        poly.emplace_back(segment.from);
        poly.emplace_back(segment.to);
    }

    {
        SVG svg("output/after.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::GRAY, 2);
        vq.debugOutput(svg, false, false, true);
        svg.writePolygons(paths, SVG::Color::BLACK, 2);
    }
    {
        SVG svg("output/toolpath_locations.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLACK, 2);
        for (auto poly : paths)
            for (Point p : poly)
                svg.writePoint(p, true, 1);
    }
    {
        SVG svg("output/toolpaths.svg", AABB(polys));
        for (ExtrusionSegment& segment : segments)
        {
//             segment.from_width = segment.from_width * 4 / 5;
//             segment.to_width = segment.to_width * 4 / 5;
            svg.writeAreas(segment.toPolygons(), SVG::Color::GRAY);
        }
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::WHITE, 2);
    }
    logError("Total processing took %fs\n", tk.restart());
}


} // namespace arachne


int main() {
    arachne::test();
    return 0;
}
