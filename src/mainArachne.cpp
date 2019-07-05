
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>

#include <boost/version.hpp>

#include <unordered_set>
#include <unordered_map>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "VoronoiQuadrangulation.h"
#include "DistributedBeadingStrategy.h"
#include "LimitedDistributedBeadingStrategy.h"
#include "utils/VoronoiUtils.h"
#include "NaiveBeadingStrategy.h"
#include "CenterDeviationBeadingStrategy.h"
#include "ConstantBeadingStrategy.h"

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
    
    polys = polys.unionPolygons(Polygons(), ClipperLib::pftPositive);
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
static Polygons jin;
static Polygons um;
static Polygons spikes;

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

    {
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
        gMAT_example.applyMatrix(PointMatrix::scale(1.0));
    }

    {
        PolygonRef wedge_1 = wedge.newPoly();
        wedge_1.emplace_back(2500, 0);
        wedge_1.emplace_back(0, 2500);
        wedge_1.emplace_back(20000, 20000);
        PointMatrix scaler = PointMatrix::scale(.846); // .846 causes a transition which is just beyond the marked skeleton
        wedge_1.applyMatrix(scaler);
        PointMatrix rot(0);
        wedge_1.applyMatrix(rot);
    }

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

        Point3Matrix rot = Point3Matrix(PointMatrix(170.0));
        pika.applyMatrix(rot);
    }
    {
        PolygonRef jinn = jin.newPoly();
        float scaler = 14;
        jinn.emplace_back(scaler * 1239.9478,38.566027);
        jinn.emplace_back(jinn.back() + Point(scaler * 9.1429, scaler * 16.947077));
        jinn.emplace_back(jinn.back() + Point(scaler * 7.5947, scaler * 17.699747));
        jinn.emplace_back(jinn.back() + Point(scaler * 6.3727, scaler * 18.178184));
        jinn.emplace_back(jinn.back() + Point(scaler * 5.3771, scaler * 18.497175));
        jinn.emplace_back(jinn.back() + Point(scaler * 4.5414, scaler * 18.71988));
        jinn.emplace_back(jinn.back() + Point(scaler * 3.8216, scaler * 18.88187));
        jinn.emplace_back(jinn.back() + Point(scaler * 3.1853, scaler * 18.99895));
        jinn.emplace_back(jinn.back() + Point(scaler * 2.6114, scaler * 19.0875));
        jinn.emplace_back(jinn.back() + Point(scaler * 2.0825, scaler * 19.15146));
        jinn.emplace_back(jinn.back() + Point(scaler * 1.5864, scaler * 19.19885));
        jinn.emplace_back(jinn.back() + Point(scaler * 1.1125, scaler * 19.23186));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.6515, scaler * 19.25403));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.195, scaler * 19.26316));
        jinn.emplace_back(jinn.back() + Point(scaler * -0.265, scaler * 19.26319));
        jinn.emplace_back(jinn.back() + Point(scaler * -0.7377, scaler * 19.25093));
        jinn.emplace_back(jinn.back() + Point(scaler * -1.2329, scaler * 19.22485));
        jinn.emplace_back(jinn.back() + Point(scaler * -1.7632, scaler * 19.18368));
        jinn.emplace_back(jinn.back() + Point(scaler * -2.345, scaler * 19.12013));
        jinn.emplace_back(jinn.back() + Point(scaler * -3.0011, scaler * 19.0292));
        jinn.emplace_back(jinn.back() + Point(scaler * -3.765, scaler * 18.8919));
        jinn.emplace_back(jinn.back() + Point(scaler * -4.6895, scaler * 18.68222));
        jinn.emplace_back(jinn.back() + Point(scaler * -5.3825, scaler * 17.79596));
        jinn.emplace_back(jinn.back() + Point(scaler * -6.1722, scaler * 17.53803));
        jinn.emplace_back(jinn.back() + Point(scaler * -7.1652, scaler * 17.15515));
        jinn.emplace_back(jinn.back() + Point(scaler * -8.4171, scaler * 16.57458));
        jinn.emplace_back(jinn.back() + Point(scaler * -9.9694, scaler * 15.68454));
        jinn.emplace_back(jinn.back() + Point(scaler * -11.7976, scaler * 14.35399));
        jinn.emplace_back(jinn.back() + Point(scaler * -13.7266, scaler * 12.51708));
        jinn.emplace_back(jinn.back() + Point(scaler * -15.4342, scaler * 10.3397));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.6597, scaler * 8.23787));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.3801, scaler * 6.59921));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.739, scaler * 5.56782));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.8679, scaler * 5.45874));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.7262, scaler * 5.89946));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.5255, scaler * 6.47141));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.2245, scaler * 7.22964));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.7526, scaler * 8.25926));
        jinn.emplace_back(jinn.back() + Point(scaler * -15.9701, scaler * 9.67884));
        jinn.emplace_back(jinn.back() + Point(scaler * -14.6169, scaler * 11.60614));
        jinn.emplace_back(jinn.back() + Point(scaler * -12.3383, scaler * 13.98742));
        jinn.emplace_back(jinn.back() + Point(scaler * -9.0791, scaler * 16.28441));
        jinn.emplace_back(jinn.back() + Point(scaler * -9.09918, scaler * 17.69635));
        jinn.emplace_back(jinn.back() + Point(scaler * -12.44885, scaler * 15.48314));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.39917, scaler * 11.14299));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.11085, scaler * 5.36278));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.87633, scaler * 0.93834));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.88691, scaler * -1.09593));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.88222, scaler * -1.19587));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.1901, scaler * -0.65917));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.19689, scaler * -0.41274));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.19132, scaler * 0.4649));
        jinn.emplace_back(jinn.back() + Point(scaler * -18.98401, scaler * 2.71606));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.95853, scaler * 6.66399));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.17841, scaler * 10.29639));
        jinn.emplace_back(jinn.back() + Point(scaler * -14.84564, scaler * 12.16939));
        jinn.emplace_back(jinn.back() + Point(scaler * -11.88547, scaler * 11.03756));
        jinn.emplace_back(jinn.back() + Point(scaler * -13.9808, scaler * 7.78003));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.16268, scaler * 0.65849));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.1495, scaler * -1.49828));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.08719, scaler * -2.06366));
        jinn.emplace_back(jinn.back() + Point(scaler * -19.09982, scaler * -2.55633));
        jinn.emplace_back(jinn.back() + Point(scaler * -18.99456, scaler * -3.23208));
        jinn.emplace_back(jinn.back() + Point(scaler * -18.70023, scaler * -4.62771));
        jinn.emplace_back(jinn.back() + Point(scaler * -18.14638, scaler * -6.45642));
        jinn.emplace_back(jinn.back() + Point(scaler * -17.27089, scaler * -8.524));
        jinn.emplace_back(jinn.back() + Point(scaler * -16.05868, scaler * -10.63026));
        jinn.emplace_back(jinn.back() + Point(scaler * -14.57678, scaler * -12.58676));
        jinn.emplace_back(jinn.back() + Point(scaler * -12.94746, scaler * -14.26053));
        jinn.emplace_back(jinn.back() + Point(scaler * -11.30097, scaler * -15.59921));
        jinn.emplace_back(jinn.back() + Point(scaler * -9.73812, scaler * -16.62196));
        jinn.emplace_back(jinn.back() + Point(scaler * -8.31284, scaler * -17.38059));
        jinn.emplace_back(jinn.back() + Point(scaler * -7.04349, scaler * -17.93507));
        jinn.emplace_back(jinn.back() + Point(scaler * -5.92543, scaler * -18.33486));
        jinn.emplace_back(jinn.back() + Point(scaler * -4.94596, scaler * -18.62422));
        jinn.emplace_back(jinn.back() + Point(scaler * -4.08683, scaler * -18.83051));
        jinn.emplace_back(jinn.back() + Point(scaler * -3.33188, scaler * -18.97981));
        jinn.emplace_back(jinn.back() + Point(scaler * -2.66502, scaler * -19.08557));
        jinn.emplace_back(jinn.back() + Point(scaler * -2.07293, scaler * -19.15823));
        jinn.emplace_back(jinn.back() + Point(scaler * -1.54473, scaler * -19.20902));
        jinn.emplace_back(jinn.back() + Point(scaler * -1.07073, scaler * -19.24033));
        jinn.emplace_back(jinn.back() + Point(scaler * -0.64338, scaler * -19.26033));
        jinn.emplace_back(jinn.back() + Point(scaler * -0.25609, scaler * -19.26936));
        jinn.emplace_back(jinn.back() + Point(scaler * -0.0549, scaler * -19.49343));
        jinn.emplace_back(jinn.back() + Point(scaler * -0.007, scaler * -19.49325));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.0545, scaler * -19.49273));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.13243, scaler * -19.49189));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.23172, scaler * -19.49222));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.35875, scaler * -19.49007));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.52227, scaler * -19.4857));
        jinn.emplace_back(jinn.back() + Point(scaler * 0.73453, scaler * -19.47827));
        jinn.emplace_back(jinn.back() + Point(scaler * 1.01293, scaler * -19.4663));
        jinn.emplace_back(jinn.back() + Point(scaler * 1.38219, scaler * -19.44216));
        jinn.emplace_back(jinn.back() + Point(scaler * 1.87839, scaler * -19.40211));
        jinn.emplace_back(jinn.back() + Point(scaler * 2.55129, scaler * -19.32262));
        jinn.emplace_back(jinn.back() + Point(scaler * 3.46817, scaler * -19.17877));
        jinn.emplace_back(jinn.back() + Point(scaler * 4.70106, scaler * -18.91284));
        jinn.emplace_back(jinn.back() + Point(scaler * 6.28387, scaler * -18.44515));
        jinn.emplace_back(jinn.back() + Point(scaler * 8.12105, scaler * -17.7107));
        jinn.emplace_back(jinn.back() + Point(scaler * 9.93387, scaler * -16.76088));
        jinn.emplace_back(jinn.back() + Point(scaler * 11.40132, scaler * -15.803776));
        jinn.emplace_back(jinn.back() + Point(scaler * 12.38087, scaler * -15.05277));
        jinn.emplace_back(jinn.back() + Point(scaler * 12.93006, scaler * -14.585829));
        jinn.emplace_back(jinn.back() + Point(scaler * 13.17677, scaler * -14.364368));
        jinn.emplace_back(jinn.back() + Point(scaler * 13.23272, scaler * -14.313574));
        jinn.emplace_back(jinn.back() + Point(scaler * 13.63427, scaler * -13.786472));
        jinn.emplace_back(jinn.back() + Point(scaler * 14.49978, scaler * -12.872874));
        jinn.emplace_back(jinn.back() + Point(scaler * 15.23246, scaler * -11.9984642));
        jinn.emplace_back(jinn.back() + Point(scaler * 15.85469, scaler * -11.1647508));
        jinn.emplace_back(jinn.back() + Point(scaler * 16.38418, scaler * -10.369847));
        jinn.emplace_back(jinn.back() + Point(scaler * 16.84004, scaler * -9.61317));
        jinn.emplace_back(jinn.back() + Point(scaler * 17.2326, scaler * -8.889433));
        jinn.emplace_back(jinn.back() + Point(scaler * 17.57397, scaler * -8.195575));
        jinn.emplace_back(jinn.back() + Point(scaler * 17.87027, scaler * -7.526437));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.12982, scaler * -6.878814));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.35808, scaler * -6.248806));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.55627, scaler * -5.631779));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.72914, scaler * -5.024901));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.88002, scaler * -4.425094));
        jinn.emplace_back(jinn.back() + Point(scaler * 19.01033, scaler * -3.829031));
        jinn.emplace_back(jinn.back() + Point(scaler * 19.1208, scaler * -3.233564));
        jinn.emplace_back(jinn.back() + Point(scaler * 19.21098, scaler * -2.635612));
        jinn.emplace_back(jinn.back() + Point(scaler * 19.28487, scaler * -2.032806));
        jinn.emplace_back(jinn.back() + Point(scaler * 19.3391, scaler * -1.421817));
        jinn.emplace_back(jinn.back() + Point(scaler * 19.37535, scaler * -0.799995));
        jinn.emplace_back(jinn.back() + Point(scaler * 19.39115, scaler * -0.164362));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.87903, scaler * 0.69094));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.81348, scaler * 1.71442));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.7005, scaler * 2.678208));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.5448, scaler * 3.598251));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.351, scaler * 4.488661));
        jinn.emplace_back(jinn.back() + Point(scaler * 18.1148, scaler * 5.360164));
        jinn.emplace_back(jinn.back() + Point(scaler * 17.837, scaler * 6.223788));
        jinn.emplace_back(jinn.back() + Point(scaler * 17.5106, scaler * 7.087582));
        jinn.emplace_back(jinn.back() + Point(scaler * 17.1324, scaler * 7.960867));
        jinn.emplace_back(jinn.back() + Point(scaler * 16.69, scaler * 8.849109));
        jinn.emplace_back(jinn.back() + Point(scaler * 16.1738, scaler * 9.758569));
        jinn.emplace_back(jinn.back() + Point(scaler * 15.5714, scaler * 10.694243));
        jinn.emplace_back(jinn.back() + Point(scaler * 14.864, scaler * 11.656573));
        jinn.emplace_back(jinn.back() + Point(scaler * 14.0327, scaler * 12.6435098));
        jinn.emplace_back(jinn.back() + Point(scaler * 13.0583, scaler * 13.6491273));
        jinn.emplace_back(jinn.back() + Point(scaler * 11.9163, scaler * 14.6551449));
        jinn.emplace_back(jinn.back() + Point(scaler * 10.5909, scaler * 15.637161));
        Point b = jinn.back();
        PolygonRef jinn2 = jin.newPoly();
        jinn2.emplace_back(b + Point(scaler * 226.5281, scaler * 332.873643));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -0.3085, scaler * -19.74316));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -0.928, scaler * -19.72448));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -1.5502, scaler * -19.68528));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -2.174, scaler * -19.6261));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -2.7984, scaler * -19.5473));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -3.4221, scaler * -19.44743));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -4.0439, scaler * -19.32683));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -4.6633, scaler * -19.18716));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -5.2787, scaler * -19.02678));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -5.8897, scaler * -18.84703));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -6.4951, scaler * -18.64754));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -7.094, scaler * -18.42788));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -7.6856, scaler * -18.18877));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -8.2692, scaler * -17.93069));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -8.8448, scaler * -17.65494));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -9.4103, scaler * -17.359085));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -9.9666, scaler * -17.046169));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -10.5117, scaler * -16.71438));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -11.0464, scaler * -16.365756));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -11.5704, scaler * -16.000633));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -12.0827, scaler * -15.618869));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -12.5812, scaler * -15.218402));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -13.0684, scaler * -14.803091));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -13.5419, scaler * -14.371133));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -14.0017, scaler * -13.923166));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -14.4482, scaler * -13.460141));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -14.879, scaler * -12.980882));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -15.2964, scaler * -12.487693));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -15.6978, scaler * -11.97927));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -16.0827, scaler * -11.45616));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -16.4521, scaler * -10.91959));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -16.8036, scaler * -10.36886));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -17.1391, scaler * -9.80589));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -17.4554, scaler * -9.22969));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -17.7537, scaler * -8.64184));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.0344, scaler * -8.04332));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.2934, scaler * -7.43329));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.533, scaler * -6.81386));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.7514, scaler * -6.18549));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.95, scaler * -5.5497));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.1257, scaler * -4.90678));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.28151, scaler * -4.25876));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.41346, scaler * -3.60604));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.5237, scaler * -2.95042));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.6116, scaler * -2.29317));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.67734, scaler * -1.63568));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.72244, scaler * -0.97943));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.74392, scaler * -0.32566));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.62678, scaler * 0.32769));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.60619, scaler * 0.98313));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.56078, scaler * 1.63759));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.49561, scaler * 2.29035));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.40826, scaler * 2.94036));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.29899, scaler * 3.5867));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.16942, scaler * 4.2288));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -19.01759, scaler * 4.86542));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.84523, scaler * 5.49609));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.65206, scaler * 6.11996));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.43884, scaler * 6.73644));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -18.20489, scaler * 7.34454));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -17.95161, scaler * 7.94392));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -17.67818, scaler * 8.53347));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -17.38692, scaler * 9.11347));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -17.0765, scaler * 9.68249));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -16.74746, scaler * 10.23993));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -16.40073, scaler * 10.7854));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -16.03793, scaler * 11.31921));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -15.65818, scaler * 11.8399));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -15.26093, scaler * 12.34594));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -14.84996, scaler * 12.839362));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -14.42306, scaler * 13.317338));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -13.98106, scaler * 13.779292));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -13.67908, scaler * 14.39264));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -13.19911, scaler * 14.833649));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -12.70602, scaler * 15.258791));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -12.19921, scaler * 15.665999));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -11.68097, scaler * 16.056866));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -11.15091, scaler * 16.429702));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -10.60977, scaler * 16.784211));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -10.05851, scaler * 17.120612));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -9.49714, scaler * 17.437598));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -8.92674, scaler * 17.73564));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -8.34811, scaler * 18.01501));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -7.76171, scaler * 18.27546));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -7.16808, scaler * 18.51676));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -6.56784, scaler * 18.73916));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -5.96063, scaler * 18.93969));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -5.34853, scaler * 19.12277));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -4.73072, scaler * 19.28371));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -4.10892, scaler * 19.42666));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -3.48295, scaler * 19.54825));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -2.85392, scaler * 19.65103));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -2.22193, scaler * 19.73114));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -1.58825, scaler * 19.79283));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -0.95326, scaler * 19.83397));
        jinn2.emplace_back(jinn2.back() + Point(scaler * -0.31775, scaler * 19.85473));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 0.31772, scaler * 19.85399));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 0.95316, scaler * 19.83268));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 1.58822, scaler * 19.79313));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 2.22191, scaler * 19.73142));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 2.85374, scaler * 19.65028));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 3.48286, scaler * 19.54815));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 4.10884, scaler * 19.42662));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 4.73061, scaler * 19.2836));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 5.34854, scaler * 19.12312));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 5.96058, scaler * 18.93975));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 6.56767, scaler * 18.73889));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 7.1678, scaler * 18.5163));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 7.76189, scaler * 18.27608));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 8.34811, scaler * 18.0152));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 8.92748, scaler * 17.73719));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 9.49715, scaler * 17.43766));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 10.05792, scaler * 17.11967));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 10.60986, scaler * 16.78443));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 11.15063, scaler * 16.42938));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 11.68112, scaler * 16.05716));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 12.19924, scaler * 15.66611));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 12.70571, scaler * 15.2585));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 13.19956, scaler * 14.83421));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 13.67974, scaler * 14.39336));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 13.98173, scaler * 13.77992));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 14.42247, scaler * 13.31678));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 14.849, scaler * 12.83855));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 15.26167, scaler * 12.34657));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 15.65801, scaler * 11.83978));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 16.03872, scaler * 11.31976));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 16.40206, scaler * 10.78623));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 16.74809, scaler * 10.24022));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 17.07538, scaler * 9.68177));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 17.38623, scaler * 9.11307));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 17.67847, scaler * 8.53357));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 17.95166, scaler * 7.94391));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.20375, scaler * 7.34406));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.43843, scaler * 6.73631));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.65075, scaler * 6.11957));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.84491, scaler * 5.49608));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.01848, scaler * 4.86571));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.16923, scaler * 4.2288));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.29983, scaler * 3.5869));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.40858, scaler * 2.94043));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.49521, scaler * 2.29032));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.56169, scaler * 1.63767));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.60463, scaler * 0.98307));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.62818, scaler * 0.32774));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.74384, scaler * -0.32565));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.72194, scaler * -0.9794));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.67809, scaler * -1.63574));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.61283, scaler * -2.29333));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.52323, scaler * -2.9504));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.41318, scaler * -3.60601));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.28126, scaler * -4.25874));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 19.1263, scaler * -4.90694));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.9498, scaler * -5.54968));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.7521, scaler * -6.18576));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.533, scaler * -6.81391));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.2935, scaler * -7.43341));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 18.0336, scaler * -8.04299));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 17.7536, scaler * -8.64184));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 17.4566, scaler * -9.23037));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 17.139, scaler * -9.80592));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 16.8042, scaler * -10.3693));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 16.4525, scaler * -10.92003));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 16.0825, scaler * -11.45611));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 15.6975, scaler * -11.9792));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 15.2959, scaler * -12.4874));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 14.8792, scaler * -12.98108));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 14.448, scaler * -13.46013));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 14.001, scaler * -13.92257));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 13.5418, scaler * -14.37106));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 13.0685, scaler * -14.80336));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 12.5816, scaler * -15.21895));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 12.0824, scaler * -15.61854));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 11.5707, scaler * -16.00132));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 11.0472, scaler * -16.36704));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 10.5115, scaler * -16.71434));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 9.9662, scaler * -17.04581));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 9.4107, scaler * -17.35999));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 8.8442, scaler * -17.65412));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 8.2695, scaler * -17.9316));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 7.6856, scaler * -18.1892));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 7.0938, scaler * -18.42785));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 6.4946, scaler * -18.64634));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 5.8895, scaler * -18.84662));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 5.2786, scaler * -19.02686));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 4.6631, scaler * -19.18707));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 4.0438, scaler * -19.32667));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 3.422, scaler * -19.44756));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 2.7982, scaler * -19.54634));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 2.1739, scaler * -19.6249));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 1.5502, scaler * -19.68535));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 0.9279, scaler * -19.72424));
        jinn2.emplace_back(jinn2.back() + Point(scaler * 0.3085, scaler * -19.74263));
    }
    {
        coord_t r = 3600;
        coord_t inr = 2400;
        coord_t b = 10000;
        coord_t h = b;
        PolygonRef um_1 = um.newPoly();
        um_1.emplace_back(r, r);
        um_1.emplace_back(r, h);
        um_1.emplace_back(b - r, h);
        um_1.emplace_back(b - r, r);
        um = um.offset(r, ClipperLib::jtRound);
        Polygon bb;
        bb.emplace_back(inr, inr);
        bb.emplace_back(inr, h + inr);
        bb.emplace_back(b - inr, h + inr);
        bb.emplace_back(b - inr, inr);
        Polygons bs;
        bs.add(bb);
        um = um.difference(bs);
        Polygon a;
        a.emplace_back(-r, h);
        a.emplace_back(-r, h + 2 * r);
        a.emplace_back(b + r, h + 2 * r);
        a.emplace_back(b + r, h);
        Polygons as;
        as.add(a);
        um = um.difference(as);
    }
    if (false)
    {
        coord_t min_r = 3000;
        coord_t max_r = 8000;
        Polygons circles;
        PolygonUtils::makeCircle(Point(0,0), 1600, circles);
        for (int a = 0; a < 360; a += 360 / 10)
        {
            Polygons dot;
            coord_t r = min_r + (max_r - min_r) * a / 360;
            PolygonUtils::makeCircle(Point(-r * cos(a /180.0 * M_PI), r * sin(a /180.0 * M_PI)), 10, dot);
            dot = dot.unionPolygons(circles);
            dot = dot.approxConvexHull();
            spikes = spikes.unionPolygons(dot);
        }
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
//     r = 1559564752;
//     r = 1559566333;
//     r = 1559568483;
//     r = 1559579388;
//     r = 1559580888;
    srand(r);
    printf("r = %d;\n", r);
    fflush(stdout);
    logDebug("boost version: %s\n", BOOST_LIB_VERSION);
    
    
    
    generateTestPolys();
//     Polygons polys = generateTestPoly(20, Point(10000, 10000));
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
//     Polygons polys = um;
//     Polygons polys = spikes;
//     Polygons polys = enclosed_region;
    Polygons polys = jin;
    polys = polys.unionPolygons();

#ifdef DEBUG
    {
        SVG svg("output/outline.svg", AABB(Point(0,0), Point(10000, 10000)));
        svg.writePolygons(polys);
    }
#endif

    TimeKeeper tk;

    VoronoiQuadrangulation vq(polys);

//     DistributedBeadingStrategy beading_strategy(300, 400, 600, M_PI / 3);
//     LimitedDistributedBeadingStrategy beading_strategy(300, 400, 600, 6, M_PI / 6);
//     NaiveBeadingStrategy beading_strategy(400);
//     ConstantBeadingStrategy beading_strategy(400, 4);
    CenterDeviationBeadingStrategy beading_strategy(400, .5, 1.7);
    std::vector<ExtrusionSegment> segments = vq.generateToolpaths(beading_strategy);
    logError("Processing took %fs\n", tk.restart());


    Polygons insets;
    Polygons last_inset = polys.offset(-200);
    while (!last_inset.empty())
    {
        insets.add(last_inset);
        last_inset = last_inset.offset(-400, ClipperLib::jtRound);
    }
    logError("Naive processing took %fs\n", tk.restart());

#ifdef DEBUG
    logAlways("Generating SVGs...\n");
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
        std::ofstream csv("output/segments.csv", std::ofstream::out | std::ofstream::trunc);
        csv << "from_x; from_y; from_width; to_x; to_y; to_width\n";
        for (const ExtrusionSegment& segment : segments)
            csv << segment.from.X << "; " << segment.from.Y << "; " << segment.from_width << "; " << segment.to.X << "; " << segment.to.Y << "; " << segment.to_width << '\n';
        csv.close();
    }
    {
        SVG svg("output/toolpath_locations.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLACK, 2);
        for (auto poly : paths)
            for (Point p : poly)
                svg.writePoint(p, true, 1);
    }
    Polygons area_covered;
    Polygons overlaps;
    std::unordered_set<Point> points_visited;
    for (ExtrusionSegment& segment : segments)
    {
        if (segment.from == segment.to)
        {
            continue;
        }
//         segment.from_width = segment.from_width * 4 / 5;
//         segment.to_width = segment.to_width * 4 / 5;
        area_covered = area_covered.unionPolygons(segment.toPolygons());
        Polygons extruded = segment.toPolygons();
        Polygons reduction;
        if (points_visited.count(segment.from) > 0)
        {
            PolygonUtils::makeCircle(segment.from, segment.from_width / 2, reduction);
        }
        if (points_visited.count(segment.to) > 0)
        {
            PolygonUtils::makeCircle(segment.to, segment.to_width / 2, reduction);
        }
        extruded = extruded.difference(reduction);
        overlaps.add(extruded);
        points_visited.emplace(segment.from);
        points_visited.emplace(segment.to);
    }
    {
        SVG svg("output/toolpaths.svg", AABB(polys));
        for (PolygonRef poly : overlaps)
        {
            svg.writeAreas(poly, SVG::Color::GRAY);
        }
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    {
        SVG svg("output/overlaps.svg", AABB(polys));
        svg.writeAreas(overlaps.xorPolygons(area_covered), SVG::Color::GRAY);
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    {
        SVG svg("output/overlaps2.svg", AABB(polys));
        svg.writeAreas(overlaps.unionPolygons(), SVG::Color::GRAY);
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    {
        SVG svg("output/total_area.svg", AABB(polys));
        svg.writeAreas(area_covered, SVG::Color::GRAY);
        svg.writePolygons(polys, SVG::Color::RED, 2);
        svg.writePolygons(paths, SVG::Color::BLUE, 2);
    }
    
    {
        SVG svg("output/normal.svg", AABB(polys));
        svg.writePolygons(polys, SVG::Color::RED, 2);
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
        svg.writePolygons(insets, SVG::Color::BLUE, 2);
    }

    logError("Writing output files took %fs\n", tk.restart());
#endif // DEBUG
}


} // namespace arachne


int main() {
    arachne::test();
    return 0;
}
