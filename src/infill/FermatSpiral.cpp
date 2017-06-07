/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#include <cassert>
#include <cstdint>

#ifndef NDEBUG
# include <iostream>
#endif // NDEBUG

#include "math.h"
#include "FermatSpiral.h"
#include "FermatSpiralMST.h"

using namespace cura;


void FermatSpiralInfillGenerator::generateInfill(
    const Polygons& in_outline,
    Polygons& result_lines,
    const SliceMeshStorage* mesh)
{
    const int64_t infill_width = 300;
    // construct the MST
    SpiralContourTree tree;

    tree.setInfillWidth(infill_width);

    tree.setPolygons(in_outline.getPaths());
    tree.constructTree();

    // TODO: connect the contours
    ClipperLib::Path full_path;

    const std::vector<struct SpiralContourNode *>& all_node_list = tree.getAllContourNodeList();

    tree.connectContours();
    tree.generateFullPath(full_path);

    std::cout << "full_path size = " << full_path.size() << std::endl;

    result_lines.add(full_path);
}
