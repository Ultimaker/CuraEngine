// Copyright (c) 2025 UltiMaker and contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/BoneInfill.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <execution>
#include <numbers>
#include <numeric>
#include <random>
#include <unordered_map>

#include "geometry/OpenLinesSet.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "settings/types/Angle.h"
#include "utils/AABB.h"
#include "utils/OpenPolylineStitcher.h"

namespace cura
{

namespace
{
constexpr int wave_count = 64; // Enough waves for the field to look random without a visible repeating pattern.
constexpr double max_level = 0.9; // The field has a standard deviation of 1, so beyond this the structure is mostly empty.

double gaussianDensity(const double x)
{
    return std::exp(-0.5 * x * x) / std::sqrt(2.0 * std::numbers::pi);
}
} // namespace

BoneInfill::BoneInfill(const BoneParameters& parameters)
    : parameters_(parameters)
{
    std::mt19937_64 rng(static_cast<uint64_t>(parameters.seed) * 0x9E3779B97F4A7C15ull + 12345ull);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> gauss(0.0, 1.0);

    // The main direction of the structure, tilted away from vertical.
    const double tilt = parameters.tilt * std::numbers::pi / 180.0;
    const double azimuth = parameters.azimuth * std::numbers::pi / 180.0;
    const std::array<double, 3> axis{ std::sin(tilt) * std::cos(azimuth), std::sin(tilt) * std::sin(azimuth), std::cos(tilt) };

    // Squashing the part of every wave direction that lies along the main direction leaves waves that are mostly
    // perpendicular to it. A field made of those varies little along the main direction: struts run along it.
    const double alignment = std::clamp(parameters.alignment, 0.0, 0.97);
    const double squash = 1.0 - alignment;
    const double spread = std::clamp(parameters.irregularity, 0.0, 1.0) * 0.5; // Variation in wavelength, which varies the thickness and spacing of the struts.

    waves_.reserve(wave_count);
    for (int i = 0; i < wave_count; ++i)
    {
        // A uniformly distributed direction.
        const double cos_theta = 2.0 * uniform(rng) - 1.0;
        const double phi = 2.0 * std::numbers::pi * uniform(rng);
        const double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
        std::array<double, 3> u{ sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta };

        const double along = u[0] * axis[0] + u[1] * axis[1] + u[2] * axis[2];
        for (int d = 0; d < 3; ++d)
        {
            u[d] -= (1.0 - squash) * along * axis[d];
        }
        const double length = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        const double magnitude = std::clamp(std::exp(spread * gauss(rng)), 0.4, 2.5);

        waves_.push_back(Wave{ u[0] / length * magnitude, u[1] / length * magnitude, u[2] / length * magnitude, 2.0 * std::numbers::pi * uniform(rng) });
    }

    // The gradient of the field in the layer plane is a Gaussian vector; its mean size tells how many contour lines
    // there are per mm, which is used to scale the structure to the requested line distance.
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const Wave& wave : waves_)
    {
        sxx += wave.kx * wave.kx;
        sxy += wave.kx * wave.ky;
        syy += wave.ky * wave.ky;
    }
    sxx /= wave_count;
    sxy /= wave_count;
    syy /= wave_count;
    const double l11 = std::sqrt(std::max(sxx, 1e-12));
    const double l21 = sxy / l11;
    const double l22 = std::sqrt(std::max(syy - l21 * l21, 1e-12));
    std::mt19937_64 sample_rng(7);
    double sum = 0.0;
    constexpr int samples = 20000;
    for (int i = 0; i < samples; ++i)
    {
        const double g1 = gauss(sample_rng);
        const double g2 = gauss(sample_rng);
        sum += std::hypot(l11 * g1, l21 * g1 + l22 * g2);
    }
    gradient_strength_ = sum / samples;
}

BoneInfill::Grid BoneInfill::evaluate(const double scale, const AABB& area, const double z_mm, const double cell_mm) const
{
    Grid grid;
    grid.cell = cell_mm;
    grid.x0 = static_cast<double>(area.min_.X) / 1000.0 - 2.0 * cell_mm;
    grid.y0 = static_cast<double>(area.min_.Y) / 1000.0 - 2.0 * cell_mm;
    grid.nx = static_cast<size_t>(std::ceil((static_cast<double>(area.max_.X - area.min_.X) / 1000.0 + 4.0 * cell_mm) / cell_mm)) + 1;
    grid.ny = static_cast<size_t>(std::ceil((static_cast<double>(area.max_.Y - area.min_.Y) / 1000.0 + 4.0 * cell_mm) / cell_mm)) + 1;
    grid.values.assign(grid.nx * grid.ny, 0.0);

    const double amplitude = std::sqrt(2.0 / wave_count);
    std::vector<size_t> rows(grid.ny);
    std::iota(rows.begin(), rows.end(), size_t{ 0 });
    std::for_each(
        std::execution::par,
        rows.begin(),
        rows.end(),
        [&](const size_t j)
        {
            double* row = grid.values.data() + j * grid.nx;
            const double y = grid.y0 + static_cast<double>(j) * grid.cell;
            for (const Wave& wave : waves_)
            {
                // cos(a + i * step) for every node of the row by rotating a vector, which avoids a cosine per node.
                const double start = scale * (wave.kx * grid.x0 + wave.ky * y + wave.kz * z_mm) + wave.phase;
                const double step = scale * wave.kx * grid.cell;
                double c = std::cos(start);
                double s = std::sin(start);
                const double cd = std::cos(step);
                const double sd = std::sin(step);
                for (size_t i = 0; i < grid.nx; ++i)
                {
                    row[i] += c;
                    const double c_next = c * cd - s * sd;
                    s = s * cd + c * sd;
                    c = c_next;
                }
            }
            for (size_t i = 0; i < grid.nx; ++i)
            {
                row[i] *= amplitude;
            }
        });
    return grid;
}

std::vector<std::vector<Point2LL>> BoneInfill::trace(const Grid& grid, const double level)
{
    struct Segment
    {
        int64_t key_a, key_b; // The grid edges the segment starts and ends on.
        double ax, ay, bx, by;
    };
    std::vector<Segment> segments;

    const auto value = [&](const size_t i, const size_t j)
    {
        return grid.values[j * grid.nx + i] - level;
    };

    for (size_t j = 0; j + 1 < grid.ny; ++j)
    {
        for (size_t i = 0; i + 1 < grid.nx; ++i)
        {
            const double v00 = value(i, j);
            const double v10 = value(i + 1, j);
            const double v11 = value(i + 1, j + 1);
            const double v01 = value(i, j + 1);
            const int index = (v00 > 0 ? 1 : 0) | (v10 > 0 ? 2 : 0) | (v11 > 0 ? 4 : 0) | (v01 > 0 ? 8 : 0);
            if (index == 0 || index == 15)
            {
                continue;
            }

            const double x_left = grid.x0 + static_cast<double>(i) * grid.cell;
            const double y_bottom = grid.y0 + static_cast<double>(j) * grid.cell;

            // The four edges of the cell: where the contour crosses, and a number that identifies the edge.
            enum Edge
            {
                Bottom,
                Right,
                Top,
                Left
            };
            const auto edge_key = [&](const Edge edge) -> int64_t
            {
                const int64_t node = static_cast<int64_t>(j * grid.nx + i);
                switch (edge)
                {
                case Bottom:
                    return 2 * node;
                case Top:
                    return 2 * (node + static_cast<int64_t>(grid.nx));
                case Left:
                    return 2 * node + 1;
                default:
                    return 2 * (node + 1) + 1;
                }
            };
            const auto edge_point = [&](const Edge edge) -> std::pair<double, double>
            {
                switch (edge)
                {
                case Bottom:
                    return { x_left + grid.cell * v00 / (v00 - v10), y_bottom };
                case Top:
                    return { x_left + grid.cell * v01 / (v01 - v11), y_bottom + grid.cell };
                case Left:
                    return { x_left, y_bottom + grid.cell * v00 / (v00 - v01) };
                default:
                    return { x_left + grid.cell, y_bottom + grid.cell * v10 / (v10 - v11) };
                }
            };
            const auto add = [&](const Edge a, const Edge b)
            {
                const auto pa = edge_point(a);
                const auto pb = edge_point(b);
                segments.push_back(Segment{ edge_key(a), edge_key(b), pa.first, pa.second, pb.first, pb.second });
            };

            const double center = (v00 + v10 + v11 + v01) / 4.0;
            switch (index)
            {
            case 1:
            case 14:
                add(Left, Bottom);
                break;
            case 2:
            case 13:
                add(Bottom, Right);
                break;
            case 3:
            case 12:
                add(Left, Right);
                break;
            case 4:
            case 11:
                add(Right, Top);
                break;
            case 6:
            case 9:
                add(Bottom, Top);
                break;
            case 7:
            case 8:
                add(Left, Top);
                break;
            case 5: // Two opposite corners are positive: the centre value decides whether they are connected.
                if (center > 0)
                {
                    add(Bottom, Right);
                    add(Left, Top);
                }
                else
                {
                    add(Left, Bottom);
                    add(Right, Top);
                }
                break;
            case 10:
                if (center > 0)
                {
                    add(Left, Bottom);
                    add(Right, Top);
                }
                else
                {
                    add(Bottom, Right);
                    add(Left, Top);
                }
                break;
            }
        }
    }

    // Chain the segments that share a grid edge into polylines.
    std::unordered_map<int64_t, std::array<int, 2>> at_edge;
    at_edge.reserve(segments.size() * 2);
    const auto register_edge = [&](const int64_t key, const int segment)
    {
        auto [it, inserted] = at_edge.try_emplace(key, std::array<int, 2>{ -1, -1 });
        (it->second[0] < 0 ? it->second[0] : it->second[1]) = segment;
    };
    for (size_t s = 0; s < segments.size(); ++s)
    {
        register_edge(segments[s].key_a, static_cast<int>(s));
        register_edge(segments[s].key_b, static_cast<int>(s));
    }
    const auto neighbour = [&](const int64_t key, const int segment) -> int
    {
        const auto& pair = at_edge.at(key);
        return pair[0] == segment ? pair[1] : pair[0];
    };

    std::vector<char> used(segments.size(), 0);
    std::vector<std::vector<Point2LL>> result;
    const auto to_point = [](const double x, const double y)
    {
        return Point2LL(static_cast<coord_t>(std::llround(x * 1000.0)), static_cast<coord_t>(std::llround(y * 1000.0)));
    };

    for (size_t start = 0; start < segments.size(); ++start)
    {
        if (used[start])
        {
            continue;
        }
        used[start] = 1;

        // Walk forward from the end of this segment, then backward from its start.
        std::vector<Point2LL> forward;
        int current = static_cast<int>(start);
        int64_t key = segments[start].key_b;
        bool closed = false;
        while (true)
        {
            const int next = neighbour(key, current);
            if (next < 0)
            {
                break;
            }
            if (next == static_cast<int>(start))
            {
                closed = true;
                break;
            }
            if (used[next])
            {
                break;
            }
            used[next] = 1;
            // The shared edge is the segment's start; continue from its other end.
            const Segment& segment = segments[next];
            if (segment.key_a == key)
            {
                forward.push_back(to_point(segment.bx, segment.by));
                key = segment.key_b;
            }
            else
            {
                forward.push_back(to_point(segment.ax, segment.ay));
                key = segment.key_a;
            }
            current = next;
        }

        std::vector<Point2LL> backward;
        if (! closed)
        {
            current = static_cast<int>(start);
            key = segments[start].key_a;
            while (true)
            {
                const int next = neighbour(key, current);
                if (next < 0 || used[next])
                {
                    break;
                }
                used[next] = 1;
                const Segment& segment = segments[next];
                if (segment.key_a == key)
                {
                    backward.push_back(to_point(segment.bx, segment.by));
                    key = segment.key_b;
                }
                else
                {
                    backward.push_back(to_point(segment.ax, segment.ay));
                    key = segment.key_a;
                }
                current = next;
            }
        }

        std::vector<Point2LL> line;
        line.reserve(backward.size() + forward.size() + 3);
        for (auto it = backward.rbegin(); it != backward.rend(); ++it)
        {
            line.push_back(*it);
        }
        line.push_back(to_point(segments[start].ax, segments[start].ay));
        line.push_back(to_point(segments[start].bx, segments[start].by));
        line.insert(line.end(), forward.begin(), forward.end());
        if (closed)
        {
            line.push_back(line.front());
        }
        if (line.size() >= 2)
        {
            result.push_back(std::move(line));
        }
    }
    return result;
}

void BoneInfill::generateInfill(
    OpenLinesSet& result_polylines,
    Shape& result_polygons,
    const coord_t line_distance,
    const Shape& in_outline,
    const coord_t z,
    const coord_t line_width,
    const AngleDegrees& rotation) const
{
    if (line_distance <= 0 || in_outline.empty())
    {
        return;
    }

    Shape rotated_outline = in_outline;
    PointMatrix rotation_matrix(rotation);
    if (rotation != 0)
    {
        rotated_outline.applyMatrix(rotation_matrix);
    }
    const AABB bounding_box(rotated_outline);

    const double distance_mm = static_cast<double>(line_distance) / 1000.0;
    const double width_mm = static_cast<double>(line_width) / 1000.0;
    const double z_mm = static_cast<double>(z) / 1000.0;

    // Level of the field that is followed. 0 gives a network of plates where everything is connected.
    const double level = (1.0 - std::clamp(parameters_.connectivity, 0.0, 1.0)) * max_level;
    // Contour lines per mm are density(level) * mean gradient * scale, and there should be 1 / line_distance of them.
    const double scale = 1.0 / (distance_mm * gaussianDensity(level) * gradient_strength_);

    // Grid resolution: fine enough to follow the curves, but bounded so that a large part stays cheap.
    double cell_mm = std::clamp(distance_mm / 8.0, 0.08, 0.6);
    const double area_mm2 = static_cast<double>(bounding_box.max_.X - bounding_box.min_.X) * static_cast<double>(bounding_box.max_.Y - bounding_box.min_.Y) / 1.0e6;
    constexpr double max_nodes = 6.0e6;
    cell_mm = std::max(cell_mm, std::sqrt(area_mm2 / max_nodes));

    const Grid grid = evaluate(scale, bounding_box, z_mm, cell_mm);

    // Cut the curves so that they stay inside the area. All curves are handed over at once, which is much cheaper than one by one.
    const auto clip_to = [](const Shape& area, const std::vector<std::vector<Point2LL>>& lines, OpenLinesSet& out)
    {
        OpenLinesSet curves;
        for (const auto& points : lines)
        {
            OpenPolyline polyline;
            for (const Point2LL& p : points)
            {
                polyline.push_back(p);
            }
            if (polyline.isValid())
            {
                curves.push_back(polyline);
            }
        }
        constexpr bool restitch = false;
        constexpr coord_t max_stitching_distance = 0;
        constexpr bool split_into_segments = false;
        out.push_back(area.intersection(curves, restitch, max_stitching_distance, split_into_segments));
    };

    OpenLinesSet fit_lines;
    clip_to(rotated_outline, trace(grid, level), fit_lines);

    // Denser zone along the walls: extra struts next to each strut, like the thickening towards the hard shell of a bone.
    if (parameters_.cortical_width > 0 && parameters_.cortical_lines > 0)
    {
        const Shape inner = rotated_outline.offset(-parameters_.cortical_width);
        const Shape band = rotated_outline.difference(inner);
        // Spacing between the extra struts and the one they follow, converted to a step in the field.
        const double step = 1.6 * width_mm / (distance_mm * gaussianDensity(level));
        for (int n = 1; n <= parameters_.cortical_lines; ++n)
        {
            for (const double sign : { -1.0, 1.0 })
            {
                clip_to(band, trace(grid, level + sign * n * step), fit_lines);
            }
        }
    }

    OpenPolylineStitcher::stitch(fit_lines, result_polylines, result_polygons, line_width);

    // The structure has many small closed loops. Handing them over as polygons would let Cura's "Connect Infill Polygons"
    // join neighbouring loops with straight bridges, which distorts the organic shape; print them as plain lines instead.
    for (const Polygon& loop : result_polygons)
    {
        OpenPolyline line;
        for (const Point2LL& p : loop)
        {
            line.push_back(p);
        }
        if (! loop.empty())
        {
            line.push_back(loop.front());
        }
        if (line.isValid())
        {
            result_polylines.push_back(line);
        }
    }
    result_polygons.clear();

    if (rotation != 0)
    {
        rotation_matrix = rotation_matrix.inverse();
        result_polylines.applyMatrix(rotation_matrix);
        result_polygons.applyMatrix(rotation_matrix);
    }
}

} // namespace cura
