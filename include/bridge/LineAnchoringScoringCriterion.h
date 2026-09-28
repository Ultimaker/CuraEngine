// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "bridge/BridgeAngleScoringCriterion.h"
#include "utils/Coord_t.h"

namespace cura
{

class Shape;
class TransformedShape;

/*! Scoring criterion for bridge angle candidate that bases the score on whether the bridging lines will properly be anchored */
class LineAnchoringScoringCriterion : public BridgeAngleScoringCriterion
{
public:
    explicit LineAnchoringScoringCriterion(const std::vector<AngleDegrees>& candidates, const Shape& skin_outline, const Shape& supported_regions, const coord_t line_width);

protected:
    double computeScore(const TransformedShape& transformed_skin, const TransformedShape& transformed_supported_regions) const override;

private:
    /*!
     * Evaluates a potential bridging line to see if it can actually bridge between two supported regions
     * @param line_y The Y coordinate of the horizontal line
     * @param transformed_skin_area The skin outline, transformed so that the bridging line is horizontal
     * @param transformed_supported_area The supported regions, transformed so that the bridging line is horizontal
     * @return A tuple containing:
     *         - The length of the potentially bridging line
     *         - The score of the line regarding bridging, which is a portion of the line length that is properly bridging. It will be close to the line length if the line is
     *           mostly bridging, bun can also be negative if it is mostly hanging
     *
     * The score is based on the following criteria:
     *   - Properly bridging segments, i.e. between two supported areas, add their length to the score
     *   - Hanging segments, i.e. supported on one side but not the other (or not at all), subtract their length from the score
     *   - Segments that lie on a supported area substract part of their length from the score  */
    static std::tuple<coord_t, coord_t> evaluateBridgeLine(const coord_t line_y, const TransformedShape& transformed_skin_area, const TransformedShape& transformed_supported_area);

    /*!
     * Calculates all the intersections between a horizontal line and the given transformed shape
     * @param line_y The horizontal line Y coordinate
     * @param transformed_shape The shape to intersect with
     * @return The list of X coordinates of the intersections, unsorted
     */
    static std::vector<coord_t> shapeLineIntersections(const coord_t line_y, const TransformedShape& transformed_shape);

private:
    const coord_t line_width_;
};

} // namespace cura
