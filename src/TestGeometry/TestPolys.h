//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_TEST_POLYS_H
#define TEST_GEOMETRY_TEST_POLYS_H


#include "TestGeometry/Pika.h"
#include "TestGeometry/Jin.h"
#include "TestGeometry/Moessen.h"
#include "TestGeometry/Prescribed.h"
#include "TestGeometry/Spiky.h"
#include "TestGeometry/SVGloader.h"
#include "TestGeometry/Microstructure.h"

namespace arachne
{

class TestPolys
{
public:
    static Polygons generateTestPoly(int size, Point border)
    {
        Polygons polys;
        PolygonRef poly = polys.newPoly();
        for (int i = 0; i < size; i++)
        {
            poly.emplace_back(rand() % border.X, rand() % border.Y);
        }
        polys = polys.execute(ClipperLib::pftPositive);
        return polys;
    }

    static Polygons test_poly_1;
    static Polygons squares;
    static Polygons circle;
    static Polygons circle_flawed;
    static Polygons cross_shape;
    static Polygons gMAT_example;
    static Polygons test_various_aspects;
    static Polygons simple_MAT_example;
    static Polygons simple_MAT_example_rounded_corner;
    static Polygons beading_conflict;
    static Polygons legend;
    static Polygons wedge;
    static Polygons limit_wedge;
    static Polygons double_wedge;
    static Polygons flawed_wedge;
    static Polygons clean_and_flawed_wedge_part;
    static Polygons rounded_wedge;
    static Polygons flawed_wall;
    static Polygons marked_local_opt;
    static Polygons parabola;
    static Polygons pikachu;
    static Polygons jin;
    static Polygons um;
    static Polygons spikes;
    static Polygons spikes_row;
    static Polygons enclosed_region;

    static void generateTestPolys()
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
            PolygonRef cross1 = cross_shape.newPoly();
            cross1.emplace_back(400, 0);
            cross1.emplace_back(400,400);
            cross1.emplace_back(0,400);
            cross1.emplace_back(0,500);
            cross1.emplace_back(400,500);
            cross1.emplace_back(400,900);
            cross1.emplace_back(500,900);
            cross1.emplace_back(500,500);
            cross1.emplace_back(900,500);
            cross1.emplace_back(900,400);
            cross1.emplace_back(500,400);
            cross1.emplace_back(500,0);
            PointMatrix scaler = PointMatrix::scale(13.8);
            cross1.applyMatrix(scaler);
        }

        {
            PolygonRef gMAT_example_outline = gMAT_example.newPoly();
            gMAT_example_outline.emplace_back(0, -20);
            gMAT_example_outline.emplace_back(7000, -20);
            gMAT_example_outline.emplace_back(8050, -20);
            gMAT_example_outline.emplace_back(8050, 2000);
            gMAT_example_outline.emplace_back(7500, 12000); // extra wedge part
            gMAT_example_outline.emplace_back(7000, 2000);
            gMAT_example_outline.emplace_back(7000, 11500);
            gMAT_example_outline.emplace_back(6500, 12000);
            gMAT_example_outline.emplace_back(0, 12000);
            PolygonRef gMAT_example_triangle = gMAT_example.newPoly();
            gMAT_example_triangle.emplace_back(1000, 7000);
            gMAT_example_triangle.emplace_back(1000, 7400);
            gMAT_example_triangle.emplace_back(5000, 9000);
            gMAT_example_triangle.emplace_back(1000, 10600);
            gMAT_example_triangle.emplace_back(1000, 11000);
            gMAT_example_triangle.emplace_back(6000, 9000);
            PolygonRef gMAT_example_round = gMAT_example.newPoly();
            gMAT_example_round.emplace_back(1000, 2000);
            gMAT_example_round.emplace_back(1000, 5600);
            gMAT_example_round.emplace_back(2000, 6000);
            gMAT_example_round.emplace_back(5000, 6000);
            gMAT_example_round.emplace_back(5000, 3000);
            gMAT_example_round.emplace_back(4000, 2000);
            gMAT_example_round.emplace_back(2200, 2000);
            gMAT_example_round.emplace_back(2200, 4400);

            gMAT_example_round.emplace_back(2500, 4700);
            gMAT_example_round.emplace_back(2800, 4800);
            gMAT_example_round.emplace_back(3000, 4800);
            gMAT_example_round.emplace_back(3300, 4700);
            
            gMAT_example_round.emplace_back(3600, 4400);
            gMAT_example_round.emplace_back(3600, 3000);
            gMAT_example_round.emplace_back(4200, 3000);
            gMAT_example_round.emplace_back(4200, 4400);
            
            gMAT_example_round.emplace_back(4000, 4800);
            gMAT_example_round.emplace_back(3800, 5000);
            gMAT_example_round.emplace_back(3400, 5200);
            gMAT_example_round.emplace_back(2600, 5200);
            gMAT_example_round.emplace_back(2200, 5000);
            gMAT_example_round.emplace_back(2000, 4800);
            
            gMAT_example_round.emplace_back(1800, 4400);
            gMAT_example_round.emplace_back(1800, 2000);
            
            PolygonRef circle_hole = gMAT_example.newPoly();
            coord_t r = 700;
            for (float a = 360; a > 0; a -= 360 / 37)
            {
                circle_hole.add(Point(cos(a/180 * M_PI) * r, sin(a/180 * M_PI) * r) + Point(1600,9000));
            }
        }
        {
            {
                PolygonRef poly = test_various_aspects.newPoly();
                float s = MM2INT(1);
                poly.emplace_back(s*4.333,s*0.000);
                poly.emplace_back(s*4.667,s*0.333);
                poly.emplace_back(s*4.667,s*1.083);
                poly.emplace_back(s*3.667,s*1.417);
                poly.emplace_back(s*4.667,s*1.667);
                poly.emplace_back(s*4.667,s*5.667);
                poly.emplace_back(s*-0.000,s*5.667);
                poly.emplace_back(s*0.000,s*3.000);
                poly.emplace_back(s*0.000,s*2.667);
                poly.emplace_back(s*-0.000,s*1.000);
                poly.emplace_back(s*0.107,s*0.676);
                poly.emplace_back(s*0.351,s*0.434);
                poly.emplace_back(s*0.651,s*0.262);
                poly.emplace_back(s*0.975,s*0.141);
                poly.emplace_back(s*1.312,s*0.061);
                poly.emplace_back(s*1.654,s*0.015);
                poly.emplace_back(s*2.000,s*0.000);
            }
            {
                PolygonRef poly = test_various_aspects.newPoly();
                float s = MM2INT(1);
                poly.emplace_back(s*0.195,s*3.000);
                poly.emplace_back(s*0.195,s*5.233);
                poly.emplace_back(s*2.333,s*5.233);
                poly.emplace_back(s*4.000,s*4.805);
                poly.emplace_back(s*2.333,s*4.389);
                poly.emplace_back(s*2.068,s*4.276);
                poly.emplace_back(s*1.877,s*4.021);
                poly.emplace_back(s*1.756,s*3.708);
                poly.emplace_back(s*1.676,s*3.258);
                poly.emplace_back(s*1.676,s*2.797);
                poly.emplace_back(s*1.756,s*2.347);
                poly.emplace_back(s*1.877,s*2.035);
                poly.emplace_back(s*2.068,s*1.779);
                poly.emplace_back(s*2.333,s*1.667);
                poly.emplace_back(s*3.333,s*1.417);
                poly.emplace_back(s*2.333,s*1.000);
                poly.emplace_back(s*0.195,s*1.000);
                poly.emplace_back(s*0.195,s*2.667);
            }
        }
        
        {
            PolygonRef simple_MAT_example_ = simple_MAT_example.newPoly();
            simple_MAT_example_.emplace_back(0, 2000);
            simple_MAT_example_.emplace_back(1000, 2000);
            simple_MAT_example_.emplace_back(400, 1000);
            simple_MAT_example_.emplace_back(1000, 0);
            simple_MAT_example_.emplace_back(0, 0);
        }
        
        {
            coord_t size = 200;
            PolygonRef p = simple_MAT_example_rounded_corner.newPoly();
            p.emplace_back(0, 2000);
            p.emplace_back(1000, 2000);
            p.emplace_back(400, 1000);
            p.emplace_back(1000, 0);
            p.emplace_back(size, 0);
            p.emplace_back(.65 * size, .25 * size);
            p.emplace_back(.25 * size, .65 * size);
            p.emplace_back(0, size);
        }
        {
            PolygonRef p = beading_conflict.newPoly();
            coord_t l = 1000;
    //         coord_t a = 300;
    //         coord_t b = 950;
            coord_t a = 600;
            coord_t b = 1395;
            coord_t dy = 2;
            coord_t dx = 5;
            p.emplace_back(l, 0);
            p.emplace_back(l, l);
            p.emplace_back(l-dx, l+dy);
            p.emplace_back(0, l);
            p.emplace_back(0, l + a);
            p.emplace_back(l-dx, l + a -dy);
            p.emplace_back(l, l + a);
            p.emplace_back(l, l * 2 + a);
            p.emplace_back(l + b, l * 2 + a);
            p.emplace_back(l + b, l + a);
            p.emplace_back(l + b +dx, l + a -dy);
            p.emplace_back(l * 2 + b, l + a);
            p.emplace_back(l * 2 + b, l);
            p.emplace_back(l + b +dx, l +dy);
            p.emplace_back(l + b, l);
            p.emplace_back(l + b, 0);
        }
        
        {
            PolygonRef p = legend.newPoly();
            
            coord_t l = 1000;
            coord_t gap = 400;
            coord_t bridge = 400;
            
            p.emplace_back(0, 0);
            p.emplace_back(0, l);
            p.emplace_back(l-gap/2, l);
            p.emplace_back(l, l/2 + bridge/2);
            p.emplace_back(l * 3/2, l * 2);
            p.emplace_back(l * 2, bridge);
            p.emplace_back(l * 2 + gap, l);
            p.emplace_back(l * 3, l);
            p.emplace_back(l * 3, 0);
            p.emplace_back(l + gap/2, 0);
            p.emplace_back(l, l/2 - bridge/2);
            p.emplace_back(l - gap/2, 0);
        }

        {
            PolygonRef wedge_1 = wedge.newPoly();
            wedge_1.emplace_back(2500, 0);
            wedge_1.emplace_back(0, 2500);
            wedge_1.emplace_back(20000, 20000);
    //         PointMatrix scaler = PointMatrix::scale(.846); // .846 causes a transition which is just beyond the marked skeleton
            PointMatrix scaler = PointMatrix::scale(.5); // .846 causes a transition which is just beyond the marked skeleton
            wedge_1.applyMatrix(scaler);
            PointMatrix rot(-45);
            wedge_1.applyMatrix(rot);
        }

        {
            PolygonRef wedge_1 = limit_wedge.newPoly();
            coord_t length = 10000;
            wedge_1.emplace_back(0, 0);
            wedge_1.emplace_back(-length * tan(M_PI / 8), length + 100);
            wedge_1.emplace_back(length * tan(M_PI / 8), length + 100);
    //         PointMatrix scaler = PointMatrix::scale(.846); // .846 causes a transition which is just beyond the marked skeleton
            PointMatrix scaler = PointMatrix::scale(.5); // .846 causes a transition which is just beyond the marked skeleton
            wedge_1.applyMatrix(scaler);
        }

        {
            PolygonRef wedge_1 = double_wedge.newPoly();
            wedge_1.emplace_back(2500, 0);
            wedge_1.emplace_back(-20000, -20000);
            wedge_1.emplace_back(0, 2500);
            wedge_1.emplace_back(20000, 20000);
    //         PointMatrix scaler = PointMatrix::scale(.846); // .846 causes a transition which is just beyond the marked skeleton
            PointMatrix scaler = PointMatrix::scale(.5); // .846 causes a transition which is just beyond the marked skeleton
            wedge_1.applyMatrix(scaler);
            PointMatrix rot(-45);
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
            
            Point3Matrix rot = Point3Matrix(PointMatrix(-90.0));
            flawed_wedgel_1.applyMatrix(rot);
        }
        {
            coord_t large_w = 2100;
            coord_t small_w = 610;
            coord_t h = 3000;
            coord_t d = 1400;
            coord_t step_count = 7;
            coord_t deflection = 20;
            PolygonRef poly = clean_and_flawed_wedge_part.newPoly();
            poly.emplace_back(0, 0);
            poly.emplace_back(large_w / 2 - small_w / 2, h);
            poly.emplace_back(large_w / 2 + small_w / 2, h);
            poly.emplace_back(large_w, 0);
            poly.emplace_back(large_w / 2 + small_w / 2, -h);
    //         poly.emplace_back(large_w / 2 - small_w / 2, -h);
            Point from(large_w / 2 - small_w / 2, -h);
            Point to(0,0);
            bool alternate = true;
            for (coord_t step = 0; step < step_count; step++)
            {
                Point mid = from + (to - from) * step / step_count;
                mid += Point(alternate? deflection : -deflection, 0);
                poly.add(mid);
                alternate = !alternate;
            }
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
            PolygonRef pb = parabola.newPoly();
            coord_t w = 2000;
            coord_t h = 8000;
            coord_t step = 100;
            for (coord_t x = -w / 2; x <= w / 2; x += step)
            {
                pb.add(Point(x, x * x * h / w / w));
            }
        }
        
        pikachu = Pika::generatePika();
        
        jin = Jin::generateJin();
        
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
        {
            coord_t min_r = 6000;
            coord_t max_r = 24000;
            Polygons circles;
            PolygonUtils::makeCircle(Point(0,0), 1600, circles);
            for (int a = 0; a < 360; a += 360 / 10)
            {
                Polygons dot;
                coord_t r = min_r + (max_r - min_r) * a / 360;
                PolygonUtils::makeCircle(Point(-r * cos(a /180.0 * M_PI), r * sin(a /180.0 * M_PI)), 10, dot);
                dot = dot.unionPolygons(circles);
                dot = dot.approxConvexHull();
                dot.makeConvex();
                spikes = spikes.unionPolygons(dot);
            }
        }
        {
            coord_t min_r = 6000;
            coord_t max_r = 16000;
            coord_t step = 1600;
            coord_t spike_count = 10;
            PolygonRef p = spikes_row.newPoly();
    //         p.emplace_back(spike_count * step, 0);
            p.emplace_back(spike_count * step, -step);
            p.emplace_back(0, -step);
            p.emplace_back(0, 0);
            for (coord_t spike = 0; spike < spike_count; spike++)
            {
                p.emplace_back(spike * step + step / 2, min_r + (max_r - min_r) * spike / (spike_count - 1));
                p.emplace_back(spike * step + step, 0);
            }
        }
        {
            PolygonRef outline = enclosed_region.newPoly();
            outline.emplace_back(0, 0);
            outline.emplace_back(0, 10000);
            outline.emplace_back(10000, 10000);
            outline.emplace_back(10000, 0);
            PolygonRef hole = enclosed_region.newPoly();
            hole.emplace_back(1000, 1000);
            hole.emplace_back(9000, 1000);
            hole.emplace_back(9000, 9000);
            hole.emplace_back(1000, 9000);
            PolygonRef outline2 = enclosed_region.newPoly();
            outline2.emplace_back(2000, 2000);
            outline2.emplace_back(2000, 8000);
            outline2.emplace_back(8000, 8000);
            outline2.emplace_back(8000, 2000);
            PolygonRef hole2 = enclosed_region.newPoly();
            hole2.emplace_back(4000, 4000);
            hole2.emplace_back(6000, 4000);
            hole2.emplace_back(6000, 6000);
            hole2.emplace_back(4000, 6000);

            PointMatrix rot = PointMatrix(170.0);
            enclosed_region.applyMatrix(rot);
        }
    }

};




} // namespace arachne
#endif // TEST_GEOMETRY_TEST_POLYS_H
