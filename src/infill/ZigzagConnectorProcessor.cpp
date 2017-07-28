#include <cassert>
#include "ZigzagConnectorProcessor.h"

using namespace cura;


void ZigzagConnectorProcessor::registerVertex(const Point& vertex)
{
    if (this->is_first_connector)
    {
        this->first_connector.push_back(vertex);
    }
    else
    { // it's yet unclear whether the line segment should be included, so we store it until we know
        this->current_connector.push_back(vertex);
    }
}


bool ZigzagConnectorProcessor::shouldAddThisConnector(int start_scanline_idx, int end_scanline_idx, int direction) const
{
    assert((direction != 1 || direction != -1) && "direction should be either +1 or -1");
    //
    // Decide whether we should add this connection or not.
    // Add this zag connection in the following cases:
    //  - if this zag lays in an even-numbered scanline segment
    //  - if this zag is an end piece (check if the previous and the current scanlines are the same)
    //    and "use end piece" is enabled
    // Don't add a zag if:
    //  - this zag is NOT an end piece, "skip some zags" is enabled, and this zag lays in a segment
    //    which needs to be skipped.
    // Moreover:
    //  - if a "connected end pieces" is not enabled and this is an end piece, the last line
    //    of this end piece will not be added.
    //
    // The rules above also apply to how the last part is processed (in polygon finishes)
    //
    const bool is_this_endpiece = start_scanline_idx == end_scanline_idx;
    const bool is_this_connection_even = start_scanline_idx % 2 == 0;
    bool should_skip_this_connection = false;
    if (this->skip_some_zags && this->zag_skip_count > 0)
    {
        //
        // Skipping some zags is done in the following way:
        //  > direction: negative number means from <right> -> <left>
        //               positive number means from <left> -> <right>
        //  > for a connection:
        //    - if it comes from <left> -> <right> (direction >= 0), we skip connections that
        //      lay in segments which mean the condition: "index mod skip_count = 0"
        //    - for connections that come from the other direction, we skip on
        //      "(index - 1) mod skip_count = 0"
        //
        if (direction > 0)
        {
            should_skip_this_connection = start_scanline_idx % this->zag_skip_count == 0;
        }
        else
        {
            should_skip_this_connection = (start_scanline_idx - 1) % this->zag_skip_count == 0;
        }
    }

    const bool should_add =
        (is_this_connection_even && !is_this_endpiece && !should_skip_this_connection) // normal connections that should be added
        || (this->use_endpieces && is_this_endpiece);  // end piece if it is enabled;

    return should_add;
}


void ZigzagConnectorProcessor::registerScanlineSegmentIntersection(const Point& intersection, int scanline_index, int direction)
{

    if (this->is_first_connector)
    {
        // process as the first connector if we haven't found one yet
        // this will be processed with the last remaining piece at the end (when the polygon finishes)
        this->first_connector.push_back(intersection);
        this->first_connector_end_scanline_index = scanline_index;
        this->first_connector_direction = direction;
        this->is_first_connector = false;
    }
    else
    {
        // add this connector if needed
        if (this->shouldAddThisConnector(this->last_connector_index, scanline_index, direction))
        {
            for (unsigned int point_idx = 0; point_idx < this->current_connector.size() - 1; ++point_idx)
            {
                addLine(this->current_connector[point_idx], this->current_connector[point_idx + 1]);
            }
            // only add the last line if:
            //  - it is not an end piece, or
            //  - it is an end piece and "connected end pieces" is enabled
            const bool is_this_endpiece = scanline_index == this->last_connector_index;
            if (!is_this_endpiece || (is_this_endpiece && this->connected_endpieces))
            {
                addLine(this->current_connector.back(), intersection);
            }
        }
    }

    // update state
    this->current_connector.clear(); // we're starting a new (odd) zigzag connector, so clear the old one
    this->current_connector.push_back(intersection);
    this->last_connector_index = scanline_index;
    this->last_connector_direction = direction;
}


void ZigzagConnectorProcessor::registerPolyFinished()
{
    int scanline_start_index = this->last_connector_index;
    int scanline_end_index = this->first_connector_end_scanline_index;
    int direction = scanline_end_index > scanline_start_index ? 1 : -1;
    const bool is_endpiece = this->is_first_connector || (!this->is_first_connector && scanline_start_index == scanline_end_index);

    // decides whether to add this zag according to the following rules
    if ((is_endpiece && this->use_endpieces)
        || (!is_endpiece && this->shouldAddThisConnector(scanline_start_index, scanline_end_index, direction)))
    {
        // for convenience, put every point in one vector
        for (const Point& point : this->first_connector)
        {
            this->current_connector.push_back(point);
        }
        this->first_connector.clear();

        for (unsigned int point_idx = 1; point_idx < this->current_connector.size() - 1; ++point_idx)
        {
            addLine(this->current_connector[point_idx - 1], this->current_connector[point_idx]);
        }
        // only add the last line if:
        //  - it is not an end piece, or
        //  - it is an end piece and "connected end pieces" is enabled
        if (!is_endpiece || (is_endpiece && this->connected_endpieces && this->current_connector.size() >= 2))
        {
            addLine(this->current_connector[current_connector.size() - 2],
                    this->current_connector[current_connector.size() - 1]);
        }
    }

    // reset member variables
    this->reset();
}
