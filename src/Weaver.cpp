#include "Weaver.h"

#include <cmath> // sqrt
#include <fstream> // debug IO
#include <unistd.h>

#include "progress/Progress.h"
#include "weaveDataStorage.h"
#include "PrintFeature.h"

namespace cura
{

void Weaver::weave(MeshGroup* meshgroup)
{
    wireFrame.meshgroup = meshgroup;

    int maxz = meshgroup->max().z;

    int layer_count = (maxz - initial_layer_thickness) / connectionHeight + 1;

    std::cerr << "Layer count: " << layer_count << "\n";

    std::vector<cura::Slicer*> slicerList;

    for(Mesh& mesh : meshgroup->meshes)
    {
        cura::Slicer* slicer = new cura::Slicer(&mesh, initial_layer_thickness, connectionHeight, layer_count, mesh.getSettingBoolean("meshfix_keep_open_polygons"), mesh.getSettingBoolean("meshfix_extensive_stitching"));
        slicerList.push_back(slicer);
    }

    int starting_layer_idx;
    { // find first non-empty layer
        for (starting_layer_idx = 0; starting_layer_idx < layer_count; starting_layer_idx++)
        {
            Polygons parts;
            for (cura::Slicer* slicer : slicerList)
                parts.add(slicer->layers[starting_layer_idx].polygons);

            if (parts.size() > 0)
                break;
        }
        if (starting_layer_idx > 0)
        {
            logWarning("First %i layers are empty!\n", starting_layer_idx);
        }
    }


    std::cerr<< "chainifying layers..." << std::endl;
    {
        int starting_z = -1;
        for (cura::Slicer* slicer : slicerList)
            wireFrame.bottom_outline.add(slicer->layers[starting_layer_idx].polygons);

        CommandSocket::sendPolygons(PrintFeatureType::OuterWall, /*0,*/ wireFrame.bottom_outline, 1);

        if (slicerList.empty()) //Wait, there is nothing to slice.
        {
            wireFrame.z_bottom = 0;
        }
        else
        {
            wireFrame.z_bottom = slicerList[0]->layers[starting_layer_idx].z;
        }

        Point starting_point_in_layer;
        if (wireFrame.bottom_outline.size() > 0)
            starting_point_in_layer = (wireFrame.bottom_outline.max() + wireFrame.bottom_outline.min()) / 2;
        else
            starting_point_in_layer = (Point(0,0) + meshgroup->max() + meshgroup->min()) / 2;

        Progress::messageProgressStage(Progress::Stage::INSET_SKIN, nullptr);
        for (int layer_idx = starting_layer_idx + 1; layer_idx < layer_count; layer_idx++)
        {
            Progress::messageProgress(Progress::Stage::INSET_SKIN, layer_idx+1, layer_count); // abuse the progress system of the normal mode of CuraEngine

            Polygons parts1;
            for (cura::Slicer* slicer : slicerList)
                parts1.add(slicer->layers[layer_idx].polygons);


            Polygons chainified;

            chainify_polygons(parts1, starting_point_in_layer, chainified);

            CommandSocket::sendPolygons(PrintFeatureType::OuterWall, /*layer_idx - starting_layer_idx,*/ chainified, 1);

            if (chainified.size() > 0)
            {
                if (starting_z == -1) starting_z = slicerList[0]->layers[layer_idx-1].z;
                wireFrame.layers.emplace_back();
                WeaveLayer& layer = wireFrame.layers.back();

                layer.z0 = slicerList[0]->layers[layer_idx-1].z - starting_z;
                layer.z1 = slicerList[0]->layers[layer_idx].z - starting_z;

                layer.supported = chainified;

                starting_point_in_layer = layer.supported.back().back();
            }
        }
    }

    std::cerr<< "finding horizontal parts..." << std::endl;
    {
        Progress::messageProgressStage(Progress::Stage::SUPPORT, nullptr);
        for (unsigned int layer_idx = 0; layer_idx < wireFrame.layers.size(); layer_idx++)
        {
            Progress::messageProgress(Progress::Stage::SUPPORT, layer_idx+1, wireFrame.layers.size()); // abuse the progress system of the normal mode of CuraEngine

            WeaveLayer& layer = wireFrame.layers[layer_idx];

            Polygons empty;
            Polygons& layer_above = (layer_idx+1 < wireFrame.layers.size())? wireFrame.layers[layer_idx+1].supported : empty;

            createHorizontalFill(layer, layer_above);
        }
    }
    // at this point layer.supported still only contains the polygons to be connected
    // when connecting layers, we further add the supporting polygons created by the roofs

    std::cerr<< "connecting layers..." << std::endl;
    {
        Polygons* lower_top_parts = &wireFrame.bottom_outline;
        int last_z = wireFrame.z_bottom;
        for (unsigned int layer_idx = 0; layer_idx < wireFrame.layers.size(); layer_idx++) // use top of every layer but the last
        {
            WeaveLayer& layer = wireFrame.layers[layer_idx];

            connect_polygons(*lower_top_parts, last_z, layer.supported, layer.z1, layer);
            layer.supported.add(layer.roofs.roof_outlines);
            lower_top_parts = &layer.supported;

            last_z = layer.z1;
        }
    }


    { // roofs:
        if (!wireFrame.layers.empty()) //If there are no layers, create no roof.
        {
            WeaveLayer& top_layer = wireFrame.layers.back();
            Polygons to_be_supported; // empty for the top layer
            fillRoofs(top_layer.supported, to_be_supported, -1, top_layer.z1, top_layer.roofs);
        }
    }


    { // bottom:
        if (!wireFrame.layers.empty()) //If there are no layers, create no bottom.
        {
            Polygons to_be_supported; // is empty for the bottom layer, cause the order of insets doesn't really matter (in a sense everything is to be supported)
            fillRoofs(wireFrame.bottom_outline, to_be_supported, -1, wireFrame.layers.front().z0, wireFrame.bottom_infill);
        }
    }

}



void Weaver::createHorizontalFill(WeaveLayer& layer, Polygons& layer_above)
{
    int64_t bridgable_dist = connectionHeight;

//     Polygons& polys_below = lower_top_parts;
    Polygons& polys_here = layer.supported;
    Polygons& polys_above = layer_above;


    { // roofs
        Polygons to_be_supported =  polys_above.offset(bridgable_dist);
        fillRoofs(polys_here, to_be_supported, -1, layer.z1, layer.roofs);

    }

    { // floors
        Polygons to_be_supported =  polys_above.offset(-bridgable_dist);
        fillFloors(polys_here, to_be_supported, 1, layer.z1, layer.roofs);
    }

    {// optimize away doubly printed regions (boundaries of holes in layer etc.)
        for (WeaveRoofPart& inset : layer.roofs.roof_insets)
            connections2moves(inset);
    }

}


void Weaver::fillRoofs(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& horizontals)
{
    std::vector<WeaveRoofPart>& insets = horizontals.roof_insets;

    if (supporting.size() == 0) return; // no parts to start the roof from!

    Polygons roofs = supporting.difference(to_be_supported);

    roofs = roofs.offset(-roof_inset).offset(roof_inset);

    if (roofs.size() == 0) return;


    Polygons roof_outlines;
    Polygons roof_holes;
    { // split roofs into outlines and holes
        std::vector<PolygonsPart> roof_parts = roofs.splitIntoParts();
        for (PolygonsPart& roof_part : roof_parts)
        {
            roof_outlines.add(roof_part[0]);
            for (unsigned int hole_idx = 1; hole_idx < roof_part.size(); hole_idx++)
            {
                roof_holes.add(roof_part[hole_idx]);
                roof_holes.back().reverse();
            }
        }
    }


    Polygons supporting_outlines;

    std::vector<PolygonsPart> supporting_parts = supporting.splitIntoParts();
    for (PolygonsPart& supporting_part : supporting_parts)
        supporting_outlines.add(supporting_part[0]); // only add outlines, not the holes



    Polygons inset1;
    Polygons last_inset;
    Polygons last_supported = supporting;
    for (Polygons inset0 = supporting_outlines; inset0.size() > 0; inset0 = last_inset)
    {
        last_inset = inset0.offset(direction * roof_inset, ClipperLib::jtRound);
        inset1 = last_inset.intersection(roof_outlines); // stay within roof area
        inset1 = inset1.unionPolygons(roof_holes);// make insets go around holes

        if (inset1.size() == 0) break;

        insets.emplace_back();

        connect(last_supported, z, inset1, z, insets.back());

        inset1 = inset1.remove(roof_holes); // throw away holes which appear in every intersection
        inset1 = inset1.remove(roof_outlines);// throw away fully filled regions

        last_supported = insets.back().supported; // chainified

    }



    horizontals.roof_outlines.add(roofs); // TODO just add the new lines, not the lines of the roofs which are already supported ==> make outlines into a connection from which we only print the top, not the connection
}

void Weaver::fillFloors(Polygons& supporting, Polygons& to_be_supported, int direction, int z, WeaveRoof& horizontals)
{
    std::vector<WeaveRoofPart>& outsets = horizontals.roof_insets;

    if (to_be_supported.size() == 0) return; // no parts to start the floor from!
    if (supporting.size() == 0) return; // no parts to start the floor from!

    Polygons floors = to_be_supported.difference(supporting);

    floors = floors.offset(-roof_inset).offset(roof_inset);

    if (floors.size() == 0) return;


    std::vector<PolygonsPart> floor_parts = floors.splitIntoParts();

    Polygons floor_outlines;
    Polygons floor_holes;
    for (PolygonsPart& floor_part : floor_parts)
    {
        floor_outlines.add(floor_part[0]);
        for (unsigned int hole_idx = 1; hole_idx < floor_part.size(); hole_idx++)
        {
            floor_holes.add(floor_part[hole_idx]);
            //floor_holes.back().reverse();
        }
    }


    Polygons outset1;

    Polygons last_supported = supporting;

    for (Polygons outset0 = supporting; outset0.size() > 0; outset0 = outset1)
    {
        outset1 = outset0.offset(roof_inset * direction, ClipperLib::jtRound).intersection(floors);
        outset1 = outset1.remove(floor_holes); // throw away holes which appear in every intersection
        outset1 = outset1.remove(floor_outlines); // throw away holes which appear in every intersection


        outsets.emplace_back();

        connect(last_supported, z, outset1, z, outsets.back());

        outset1 = outset1.remove(floor_outlines);// throw away fully filled regions

        last_supported = outsets.back().supported; // chainified

    }


    horizontals.roof_outlines.add(floors);
}


void Weaver::connections2moves(WeaveRoofPart& inset)
{


    bool include_half_of_last_down = true;


    for (WeaveConnectionPart& part : inset.connections)
    {
        std::vector<WeaveConnectionSegment>& segments = part.connection.segments;
        for (unsigned int idx = 0; idx < part.connection.segments.size(); idx += 2)
        {
            WeaveConnectionSegment& segment = segments[idx];
            assert(segment.segmentType == WeaveSegmentType::UP);
            Point3 from = (idx == 0)? part.connection.from : segments[idx-1].to;
            bool skipped = (segment.to - from).vSize2() < line_width * line_width;
            if (skipped)
            {
                unsigned int begin = idx;
                for (; idx < segments.size(); idx += 2)
                {
                    WeaveConnectionSegment& segment = segments[idx];
                    assert(segments[idx].segmentType == WeaveSegmentType::UP);
                    Point3 from = (idx == 0)? part.connection.from : segments[idx-1].to;
                    bool skipped = (segment.to - from).vSize2() < line_width * line_width;
                    if (!skipped)
                    {
                        break;
                    }
                }
                int end = idx - ((include_half_of_last_down)? 2 : 1);
                if (idx >= segments.size())
                    segments.erase(segments.begin() + begin, segments.end());
                else
                {
                    segments.erase(segments.begin() + begin, segments.begin() + end);
                    if (begin < segments.size())
                    {
                        segments[begin].segmentType = WeaveSegmentType::MOVE;
                        if (include_half_of_last_down)
                            segments[begin+1].segmentType = WeaveSegmentType::DOWN_AND_FLAT;
                    }
                    idx = begin + ((include_half_of_last_down)? 2 : 1);
                }
            }
        }
    }
}

void Weaver::connect(Polygons& parts0, int z0, Polygons& parts1, int z1, WeaveConnection& result)
{
    // TODO: convert polygons (with outset + difference) such that after printing the first polygon, we can't be in the way of the printed stuff
    // something like:
    // for (m > n)
    //     parts[m] = parts[m].difference(parts[n].offset(nozzle_top_diameter))
    // according to the printing order!
    //
    // OR! :
    //
    // unify different parts if gap is too small

    Polygons& supported = result.supported;

    if (parts1.size() == 0) return;

    Point& start_close_to = (parts0.size() > 0)? parts0.back().back() : parts1.back().back();

    chainify_polygons(parts1, start_close_to, supported);

    if (parts0.size() == 0) return;

    connect_polygons(parts0, z0, supported, z1, result);

}


void Weaver::chainify_polygons(Polygons& parts1, Point start_close_to, Polygons& result)
{
    for (unsigned int prt = 0 ; prt < parts1.size(); prt++)
    {
        ConstPolygonRef upperPart = parts1[prt];

        ClosestPolygonPoint closestInPoly = PolygonUtils::findClosest(start_close_to, upperPart);


        PolygonRef part_top = result.newPoly();

        GivenDistPoint next_upper;
        bool found = true;
        int idx = 0;

        for (Point upper_point = upperPart[closestInPoly.point_idx]; found; upper_point = next_upper.location)
        {
            found = PolygonUtils::getNextPointWithDistance(upper_point, nozzle_top_diameter, upperPart, idx, closestInPoly.point_idx, next_upper);


            if (!found)
            {
                break;
            }

            part_top.add(upper_point);

            idx = next_upper.pos;
        }
        if (part_top.size() > 0)
            start_close_to = part_top.back();
        else
            result.remove(result.size()-1);
    }
}


void Weaver::connect_polygons(Polygons& supporting, int z0, Polygons& supported, int z1, WeaveConnection& result)
{

    if (supporting.size() < 1)
    {
        std::cerr << "lower layer has zero parts!\n";
        return;
    }

    result.z0 = z0;
    result.z1 = z1;

    std::vector<WeaveConnectionPart>& parts = result.connections;

    for (unsigned int prt = 0 ; prt < supported.size(); prt++)
    {

        ConstPolygonRef upperPart(supported[prt]);


        parts.emplace_back(prt);
        WeaveConnectionPart& part = parts.back();
        PolyLine3& connection = part.connection;

        Point3 last_upper;
        bool firstIter = true;

        for (const Point& upper_point : upperPart)
        {

            ClosestPolygonPoint lowerPolyPoint = PolygonUtils::findClosest(upper_point, supporting);
            Point& lower = lowerPolyPoint.location;

            Point3 lower3 = Point3(lower.X, lower.Y, z0);
            Point3 upper3 = Point3(upper_point.X, upper_point.Y, z1);


            if (firstIter)
                connection.from = lower3;
            else
                connection.segments.emplace_back<>(lower3, WeaveSegmentType::DOWN);

            connection.segments.emplace_back<>(upper3, WeaveSegmentType::UP);
            last_upper = upper3;

            firstIter = false;
        }
    }
}


}//namespace cura
