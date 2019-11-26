#include <cassert>
#include "ZigzagConnectorProcessor.h"

using namespace cura;


void ZigzagConnectorProcessor::registerVertex(const Point& vertex)
{
    if (is_first_connector)
    {
        first_connector.push_back(vertex);
    }
    else
    { // it's yet unclear whether the polygon segment should be included, so we store it until we know
        current_connector.push_back(vertex);
    }
}


bool ZigzagConnectorProcessor::shouldAddCurrentConnector(int start_scanline_idx, int end_scanline_idx) const
{
    int direction = end_scanline_idx - start_scanline_idx;
    //
    // Decide whether we should add the current connection or not.
    // Add the current zag connection in the following cases:
    //  - if the current zag starts on an even scanline and ends on an odd scanline
    //  - if the current zag is an end piece (check if the previous and the current scanlines are the same)
    //    and "use end piece" is enabled
    // Don't add a zag if:
    //  - the current zag is NOT an end piece, "skip some zags" is enabled, and the current zag lays in a
    //    segment which needs to be skipped.
    // Moreover:
    //  - if a "connected end pieces" is not enabled and the current connection is an end piece, the last line
    //    of this end piece will not be added.
    //
    // The rules above also apply to how the last part is processed (in polygon finishes)
    //
    const bool is_this_endpiece = start_scanline_idx == end_scanline_idx;
    const bool is_this_connection_even = start_scanline_idx % 2 == 0;
    bool should_skip_this_connection = false;
    if (skip_some_zags && zag_skip_count > 0)
    {
        //
        // Here is an illustration of how the zags will be removed.
        // ***We use skip_count = 3 as an example:***
        //
        // segment numbers:    0     1     2     3     4     5     6     7     8     9     10
        //    top zags ->   |     |-----|     |--x--|     |-----|     |-----|     |--x--|     |
        //                  |     |     |     |     |     |     |     |     |     |     |     |
        // bottom zags ->   |--x--|     |-----|     |-----|     |--x--|     |-----|     |-----|
        //
        // LEGEND:  |  :    zigs
        //          -  :    zags
        //
        // Following the line from left to right, we want to remove a zag in every 3. So here we
        // are removing zags 0, 3, 6, 9, ..., which is (n * 3), where n = 0, 1, 2, ...
        //
        // We treat top and bottom zags differently:
        //   - a bottom zag goes from left -> right. Example: for zag 6, it goes from scanline 6 to 7.
        //   - a top zag goes from right -> left. Example: for zag 3, it goes from scanline 4 to 3.
        // We call the start and end scanline indices <start> and <end> separately.
        // So, to get the results described above, we skip zags in the following way:
        //   - if direction > 0 (bottom zags: left -> right), skip zags whose <start> mod 3 == 0
        //   - if direction < 0 (top zags: right -> left), skip zags whose <end> mod 3 == 0
        //
        if (direction > 0)
        {
            should_skip_this_connection = start_scanline_idx % zag_skip_count == 0;
        }
        else
        {
            should_skip_this_connection = (start_scanline_idx - 1) % zag_skip_count == 0;
        }
    }

    const bool should_add =
        (is_this_connection_even && !is_this_endpiece && !should_skip_this_connection) // normal connections that should be added
        || (use_endpieces && is_this_endpiece);  // end piece if it is enabled;

    return should_add;
}


void ZigzagConnectorProcessor::registerScanlineSegmentIntersection(const Point& intersection, int scanline_index)
{
    if (is_first_connector)
    {
        // process as the first connector if we haven't found one yet
        // this will be processed with the last remaining piece at the end (when the polygon finishes)
        first_connector.push_back(intersection);
        first_connector_end_scanline_index = scanline_index;
        is_first_connector = false;
    }
    else
    {
        // add the current connector if needed
        if (shouldAddCurrentConnector(last_connector_index, scanline_index))
        {
            const bool is_this_endpiece = scanline_index == last_connector_index;
            current_connector.push_back(intersection);
            addZagConnector(current_connector, is_this_endpiece);
        }
    }

    // update state
    current_connector.clear(); // we're starting a new (odd) zigzag connector, so clear the old one
    current_connector.push_back(intersection);
    last_connector_index = scanline_index;
}


void ZigzagConnectorProcessor::registerPolyFinished()
{
    int scanline_start_index = last_connector_index;
    int scanline_end_index = first_connector_end_scanline_index;
    const bool is_endpiece = is_first_connector || (!is_first_connector && scanline_start_index == scanline_end_index);

    // decides whether to add this zag according to the following rules
    if ((is_endpiece && use_endpieces)
        || (!is_endpiece && shouldAddCurrentConnector(scanline_start_index, scanline_end_index)))
    {
        // for convenience, put every point in one vector
        for (const Point& point : first_connector)
        {
            current_connector.push_back(point);
        }
        first_connector.clear();

        addZagConnector(current_connector, is_endpiece);
    }

    // reset member variables
    reset();
}


void ZigzagConnectorProcessor::addZagConnector(std::vector<Point>& points, bool is_endpiece)
{
    // don't include the last line yet
    if (points.size() >= 3)
    {
        for (size_t point_idx = 1; point_idx <= points.size() - 2; ++point_idx)
        {
            addLine(points[point_idx - 1], points[point_idx]);
        }
    }
    // only add the last line if:
    //  - it is not an end piece, or
    //  - it is an end piece and "connected end pieces" is enabled
    if ((!is_endpiece || (is_endpiece && connected_endpieces)) && points.size() >= 2)
    {
        addLine(points[points.size() - 2], points[points.size() - 1]);
    }
}
