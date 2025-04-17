//
// Based on https://github.com/davideberly/GeometricTools/blob/master/GTE/Mathematics/MinimumAreaBox2.h
// Which is licenced under the Boost Software License which is compatible with GPL based on https://www.gnu.org/licenses/license-list.en.html which in turn is compatible with AGPL
// So using it here should be fine. Should that be wrong the contents of this file are most likely to be considered to be under the Boost Software License.
//

#ifndef CURAENGINE_MINIMUMBOUNDINGBOX_H
#define CURAENGINE_MINIMUMBOUNDINGBOX_H
#include "geometry/Point2LL.h"
#include "geometry/SingleShape.h"
#include "linearAlg2D.h"
#include "utils/Coord_t.h"

namespace cura
{
struct MinimumBoundingBox
{
    MinimumBoundingBox()
        : axis{ Point2LL(0, 0), Point2LL(0, 0) }
        , index{ 0, 0, 0, 0 }
        , size2_u0_((coord_t)0)
        , area((coord_t)0)
    {
    }

    MinimumBoundingBox(SingleShape shape)
    {
        //Simplify ensures no unnecessary points on a line are present.
        Simplify simplify(50,5,100);
        shape = simplify.polygon(shape);

        shape.makeConvex();
        Polygon polygon = shape.outerPolygon();
        MinimumBoundingBox box = ComputeBoxForEdgeOrderN(polygon);
        axis = box.axis;
        index = box.index;
        size2_u0_ = box.size2_u0_;
        area = box.area;
        Finalize(polygon);
    }

    std::array<Point2LL, 4> getVertices() const
    {
        return { center + axis[0] + axis[1],
                 center + axis[0] - axis[1],
                 center - axis[0] - axis[1],
                 center - axis[0] + axis[1]
        };
    }

    bool inside(Point2LL p)
    {
        if(vSize(axis[0]) == 0 || vSize(axis[1]) == 0)
        {
            return false;
        }
        Point2LL direction = center - p;
        Point2LL direction_local = Point2LL(dot(direction, axis[0]) / vSize(axis[0]), dot(direction, axis[1]) / vSize(axis[1]));
        return std::abs(direction_local.X) <= extent.X && std::abs(direction_local.Y) <= extent.Y;
    }

    coord_t minimumDistance(MinimumBoundingBox& other)
    {
        std::array<Point2LL, 4> vertices_me = getVertices();
        std::array<Point2LL, 4> vertices_other = other.getVertices();

        coord_t min_dist2 = std::numeric_limits<coord_t>::max();

        // Check all edge pairs from both squares
        for (int i = 0; i < 4; ++i)
        {
            Point2LL a1 = vertices_me[i];
            Point2LL a2 = vertices_me[(i + 1) % 4];

            for (int j = 0; j < 4; ++j)
            {
                Point2LL b1 = vertices_other[j];
                Point2LL b2 = vertices_other[(j + 1) % 4];
                if (inside(b1))
                {
                    return 0;
                }
                coord_t dist2_here = LinearAlg2D::getDist2BetweenLineSegments(a1, a2, b1, b2);
                // Check the distance from each vertex of square1 to each edge of square2
                min_dist2 = std::min(min_dist2, dist2_here);
            }
        }

        return sqrt(min_dist2);
    }

    Point2LL center;
    Point2LL extent;
    std::array<Point2LL, 2> axis;
    coord_t area;

private:
    coord_t size2_u0_;
    std::array<int32_t, 4> index; // order: bottom, right, top, left


    // Sort the angles indirectly. The sorted indices are returned. This
    // avoids swapping elements of A[], which can be expensive when
    // ComputeType is an exact rational type.
    std::array<int32_t, 4> SortAngles(std::array<std::pair<coord_t, int32_t>, 4> const& A, int32_t numA)
    {
        std::array<int32_t, 4> sort = { 0, 1, 2, 3 };
        if (numA > 1)
        {
            if (numA == 2)
            {
                if (A[sort[0]].first > A[sort[1]].first)
                {
                    std::swap(sort[0], sort[1]);
                }
            }
            else if (numA == 3)
            {
                if (A[sort[0]].first > A[sort[1]].first)
                {
                    std::swap(sort[0], sort[1]);
                }
                if (A[sort[0]].first > A[sort[2]].first)
                {
                    std::swap(sort[0], sort[2]);
                }
                if (A[sort[1]].first > A[sort[2]].first)
                {
                    std::swap(sort[1], sort[2]);
                }
            }
            else // numA == 4
            {
                if (A[sort[0]].first > A[sort[1]].first)
                {
                    std::swap(sort[0], sort[1]);
                }
                if (A[sort[2]].first > A[sort[3]].first)
                {
                    std::swap(sort[2], sort[3]);
                }
                if (A[sort[0]].first > A[sort[2]].first)
                {
                    std::swap(sort[0], sort[2]);
                }
                if (A[sort[1]].first > A[sort[3]].first)
                {
                    std::swap(sort[1], sort[3]);
                }
                if (A[sort[1]].first > A[sort[2]].first)
                {
                    std::swap(sort[1], sort[2]);
                }
            }
        }
        return sort;
    }


    bool UpdateSupport(
        std::array<std::pair<coord_t, int32_t>, 4> const& A,
        int32_t numA,
        std::array<int32_t, 4> const& sort,
        const Polygon& shape,
        std::vector<bool>& visited,
        MinimumBoundingBox& box)
    {
        // Replace the support vertices of those edges attaining minimum
        // angle with the other endpoints of the edges.
        const int32_t numVertices = shape.size();
        auto const& amin = A[sort[0]];
        for (int32_t k = 0; k < numA; ++k)
        {
            auto const& a = A[sort[k]];
            if (a.first == amin.first)
            {
                if (++box.index[a.second] == numVertices)
                {
                    box.index[a.second] = 0;
                }
            }
        }

        int32_t bottom = box.index[amin.second];
        if (visited[bottom])
        {
            // We have already processed this polygon edge.
            return false;
        }
        visited[bottom] = true;

        // Cycle the vertices so that the bottom support occurs first.
        std::array<int32_t, 4> nextIndex{};
        for (int32_t k = 0; k < 4; ++k)
        {
            nextIndex[k] = box.index[(amin.second + k) % 4];
        }
        box.index = nextIndex;

        // Compute the box axis directions.
        int32_t j1 = box.index[0], j0 = j1 - 1;
        if (j0 < 0)
        {
            j0 = numVertices - 1;
        }
        box.axis[0] = shape[j1] - shape[j0];
        box.axis[1] = turn90CCW(box.axis[0]);
        box.size2_u0_ = dot(box.axis[0], box.axis[0]);

        // Compute the box area.
        std::array<Point2LL, 2> diff = { shape[box.index[1]] - shape[box.index[3]], shape[box.index[2]] - shape[box.index[0]] };
        box.area = dot(box.axis[0], diff[0]) * dot(box.axis[1], diff[1]) / box.size2_u0_;
        return true;
    }

    // Compute the smallest box for the polygon edge <V[i0],V[i1]>.
    MinimumBoundingBox SmallestBox(int32_t i0, int32_t i1, Polygon polygon)
    {
        MinimumBoundingBox box{};
        box.axis[0] = polygon[i1] - polygon[i0];
        box.axis[1] = turn90CCW(box.axis[0]);
        box.index = { i1, i1, i1, i1 };
        box.size2_u0_ = dot(box.axis[0], box.axis[0]);

        coord_t const zero = static_cast<coord_t>(0);
        Point2LL const& origin = polygon[i1];
        std::array<Point2LL, 4> support{};
        for (size_t j = 0; j < 4; ++j)
        {
            support[j] = { zero, zero };
        }

        int32_t i = 0;
        for (auto const& vertex : polygon)
        {
            Point2LL diff = vertex - origin;
            Point2LL v = { dot(box.axis[0], diff), dot(box.axis[1], diff) };

            // The right-most vertex of the bottom edge is vertices[i1].
            // The assumption of no triple of collinear vertices
            // guarantees that box.index[0] is i1, which is the initial
            // value assigned at the beginning of this function.
            // Therefore, there is no need to test for other vertices
            // farther to the right than vertices[i1].

            if (v.X > support[1].X || (v.X == support[1].X && v.Y > support[1].Y))
            {
                // New right maximum OR same right maximum but closer
                // to top.
                box.index[1] = i;
                support[1] = v;
            }

            if (v.Y > support[2].Y || (v.Y == support[2].Y && v.X < support[2].X))
            {
                // New top maximum OR same top maximum but closer
                // to left.
                box.index[2] = i;
                support[2] = v;
            }

            if (v.X < support[3].X || (v.X == support[3].X && v.Y < support[3].Y))
            {
                // New left minimum OR same left minimum but closer
                // to bottom.
                box.index[3] = i;
                support[3] = v;
            }

            ++i;
        }

        // support[0] = { 0, 0 }, so the scaled height
        // (support[2][1] - support[0][1]) is simply support[2][1].
        coord_t scaledWidth = support[1].X - support[3].X;
        coord_t scaledHeight = support[2].Y;
        box.area = scaledWidth * scaledHeight / box.size2_u0_;
        return box;
    }


    // Compute (sin(angle))^2 for the polygon edges emanating from the
    // support vertices of the box. The return value is 'true' if at
    // least one angle is in [0,pi/2); otherwise, the return value is
    // 'false' and the original polygon must be a rectangle.
    bool ComputeAngles(Polygon polygon, MinimumBoundingBox const& box, std::array<std::pair<coord_t, int32_t>, 4>& A, int32_t& numA)
    {
        int32_t const numVertices = static_cast<int32_t>(polygon.size());
        numA = 0;
        for (int32_t k0 = 3, k1 = 0; k1 < 4; k0 = k1++)
        {
            if (box.index[k0] != box.index[k1])
            {
                // The box edges are ordered in k1 as U[0], U[1],
                // -U[0], -U[1].
                Point2LL D = ((k0 & 2) ? -box.axis[k0 & 1] : box.axis[k0 & 1]);
                int32_t j0 = box.index[k0], j1 = j0 + 1;
                if (j1 == numVertices)
                {
                    j1 = 0;
                }
                Point2LL E = polygon[j1] - polygon[j0];
                coord_t dp = dot(D, -turn90CCW(E));
                coord_t esqrlen = dot(E, E);
                coord_t sinThetaSqr = (dp * dp) / esqrlen;
                A[numA] = std::make_pair(sinThetaSqr, k0);
                ++numA;
            }
        }
        return numA > 0;
    }


    MinimumBoundingBox ComputeBoxForEdgeOrderN(Polygon polygon)
    {
        // The inputs are assumed to be the vertices of a convex polygon
        // that is counterclockwise ordered. The input points must not
        // contain three consecutive collinear points.

        // When the bounding box corresponding to a polygon edge is
        // computed, we mark the edge as visited. If the edge is
        // encountered later, the algorithm terminates.
        std::vector<bool> visited(polygon.size(), false);

        // Start the minimum-area rectangle search with the edge from the
        // last polygon vertex to the first. When updating the extremes,
        // we want the bottom-most point on the left edge, the top-most
        // point on the right edge, the left-most point on the top edge,
        // and the right-most point on the bottom edge. The polygon edges
        // starting at these points are then guaranteed not to coincide
        // with a box edge except when an extreme point is shared by two
        // box edges (at a corner).
        MinimumBoundingBox minBox = SmallestBox(polygon.size() - 1, 0, polygon);
        visited[minBox.index[0]] = true;

        // Execute the rotating calipers algorithm.
        MinimumBoundingBox box = minBox;
        for (size_t i = 0; i < polygon.size(); ++i)
        {
            std::array<std::pair<coord_t, int32_t>, 4> A{};
            int32_t numA{};
            if (! ComputeAngles(polygon, box, A, numA))
            {
                // The polygon is a rectangle, so the search is over.
                break;
            }

            // Indirectly sort the A-array.
            std::array<int32_t, 4> sort = SortAngles(A, numA);

            // Update the supporting indices (box.index[]) and the box
            // axis directions (box.U[]).
            if (! UpdateSupport(A, numA, sort, polygon, visited, box))
            {
                // We have already processed the box polygon edge, so the
                // search is over.
                break;
            }

            if (box.area < minBox.area)
            {
                minBox = box;
            }
        }

        return minBox;
    }

    void Finalize(const Polygon& polygon)
    {
        // The sum, difference, and center are all computed exactly.
        std::array<Point2LL, 2> sum = { polygon[index[1]] + polygon[index[3]], polygon[index[2]] + polygon[index[0]] };

        std::array<Point2LL, 2> difference = { polygon[index[1]] - polygon[index[3]], polygon[index[2]] - polygon[index[0]] };

        center = (dot(axis[0], sum[0]) * axis[0] + dot(axis[1], sum[1]) * axis[1]) / (2 * size2_u0_);

        // Calculate the squared extent using ComputeType to avoid loss of
        // precision before computing a squared root.
        Point2LL sqrExtent;
        sqrExtent.X = dot(axis[0], difference[0]) / 2;
        sqrExtent.X *= sqrExtent.X;
        sqrExtent.X /= size2_u0_;
        extent.X = round(std::sqrt(sqrExtent.X));

        sqrExtent.Y = dot(axis[1], difference[1]) / 2;
        sqrExtent.Y *= sqrExtent.Y;
        sqrExtent.Y /= size2_u0_;
        extent.Y = round(std::sqrt(sqrExtent.Y));

        axis[0] = axis[0] / std::max(std::fabs(axis[0].X), std::fabs(axis[0].Y));
        axis[0] = (extent.X * axis[0]) / vSize(axis[0]);

        axis[1] = axis[1] / std::max(std::fabs(axis[1].X), std::fabs(axis[1].Y));
        axis[1] = (extent.Y * axis[1]) / vSize(axis[1]);
    }
};


} // namespace cura
#endif // CURAENGINE_MINIMUMBOUNDINGBOX_H
