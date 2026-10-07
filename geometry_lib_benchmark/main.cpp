// Quick and dirty one-off benchmark: Clipper 6.4.2 vs Clipper2 vs boost::geometry.
// Every library receives exactly the same integer (micrometre) input, converted outside the timed region.
//
// Usage: geobench [output.csv] [max_size] [case_filter]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <numbers>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <clipper.hpp>
#include <clipper2/clipper.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;

// ---------------------------------------------------------------------------------------------------------------------
// Neutral data model
// ---------------------------------------------------------------------------------------------------------------------
struct Pt
{
    int64_t x, y;
};
using Ring = std::vector<Pt>;
struct Poly
{
    Ring outer; // CCW (positive area, Y up)
    std::vector<Ring> holes; // CW
};
using MPoly = std::vector<Poly>;

struct Case
{
    std::string name;
    std::string size_label;
    MPoly a;
    MPoly b;
};

size_t vertexCount(const MPoly& mp)
{
    size_t n = 0;
    for (const auto& p : mp)
    {
        n += p.outer.size();
        for (const auto& h : p.holes)
            n += h.size();
    }
    return n;
}

constexpr double PI = std::numbers::pi;

Pt pt(double x, double y)
{
    return { std::llround(x), std::llround(y) };
}

Ring ellipse(double cx, double cy, double rx, double ry, size_t n, bool ccw = true)
{
    Ring r;
    r.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        double a = 2 * PI * i / n;
        if (! ccw)
            a = -a;
        r.push_back(pt(cx + rx * std::cos(a), cy + ry * std::sin(a)));
    }
    return r;
}

MPoly transform(const MPoly& in, double angle_deg, double dx, double dy)
{
    // rotate around bounding box center, then translate
    int64_t minx = INT64_MAX, miny = INT64_MAX, maxx = INT64_MIN, maxy = INT64_MIN;
    for (const auto& p : in)
        for (const auto& q : p.outer)
        {
            minx = std::min(minx, q.x);
            miny = std::min(miny, q.y);
            maxx = std::max(maxx, q.x);
            maxy = std::max(maxy, q.y);
        }
    const double cx = (minx + maxx) / 2.0, cy = (miny + maxy) / 2.0;
    const double c = std::cos(angle_deg * PI / 180), s = std::sin(angle_deg * PI / 180);
    auto tr = [&](const Ring& r)
    {
        Ring o;
        o.reserve(r.size());
        for (const auto& q : r)
        {
            const double x = q.x - cx, y = q.y - cy;
            o.push_back(pt(cx + c * x - s * y + dx, cy + s * x + c * y + dy));
        }
        return o;
    };
    MPoly out;
    for (const auto& p : in)
    {
        Poly np{ tr(p.outer), {} };
        for (const auto& h : p.holes)
            np.holes.push_back(tr(h));
        out.push_back(std::move(np));
    }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------------
// Conversions (outside the timed region)
// ---------------------------------------------------------------------------------------------------------------------
ClipperLib::Paths toC1(const MPoly& mp)
{
    ClipperLib::Paths out;
    auto add = [&](const Ring& r)
    {
        ClipperLib::Path p;
        p.reserve(r.size());
        for (const auto& q : r)
            p.emplace_back(q.x, q.y);
        out.push_back(std::move(p));
    };
    for (const auto& p : mp)
    {
        add(p.outer);
        for (const auto& h : p.holes)
            add(h);
    }
    return out;
}

Clipper2Lib::Paths64 toC2(const MPoly& mp)
{
    Clipper2Lib::Paths64 out;
    auto add = [&](const Ring& r)
    {
        Clipper2Lib::Path64 p;
        p.reserve(r.size());
        for (const auto& q : r)
            p.emplace_back(q.x, q.y);
        out.push_back(std::move(p));
    };
    for (const auto& p : mp)
    {
        add(p.outer);
        for (const auto& h : p.holes)
            add(h);
    }
    return out;
}

using BPoint = bg::model::d2::point_xy<int64_t>;
using BPoly = bg::model::polygon<BPoint, false /*CCW*/, true /*closed*/>;
using BMPoly = bg::model::multi_polygon<BPoly>;

BMPoly toBG(const MPoly& mp)
{
    BMPoly out;
    auto conv = [](const Ring& r, auto& ring)
    {
        ring.reserve(r.size() + 1);
        for (const auto& q : r)
            ring.emplace_back(q.x, q.y);
        ring.emplace_back(r.front().x, r.front().y);
    };
    for (const auto& p : mp)
    {
        BPoly bp;
        conv(p.outer, bp.outer());
        for (const auto& h : p.holes)
        {
            bp.inners().emplace_back();
            conv(h, bp.inners().back());
        }
        out.push_back(std::move(bp));
    }
    return out;
}

// Normalise an arbitrary set of rings into a structured MPoly (used only to prepare real-world input data)
void polyTreeToMPoly(const Clipper2Lib::PolyPath64& node, MPoly& out)
{
    for (const auto& outer : node)
    {
        Poly p;
        for (const auto& q : outer->Polygon())
            p.outer.push_back({ q.x, q.y });
        for (const auto& hole : *outer)
        {
            Ring h;
            for (const auto& q : hole->Polygon())
                h.push_back({ q.x, q.y });
            p.holes.push_back(std::move(h));
            polyTreeToMPoly(*hole, out);
        }
        out.push_back(std::move(p));
    }
}

MPoly normalise(const std::vector<Ring>& rings)
{
    Clipper2Lib::Paths64 paths;
    for (const auto& r : rings)
    {
        Clipper2Lib::Path64 p;
        for (const auto& q : r)
            p.emplace_back(q.x, q.y);
        paths.push_back(std::move(p));
    }
    Clipper2Lib::Clipper64 c;
    c.AddSubject(paths);
    Clipper2Lib::PolyTree64 tree;
    c.Execute(Clipper2Lib::ClipType::Union, Clipper2Lib::FillRule::EvenOdd, tree);
    MPoly out;
    polyTreeToMPoly(tree, out);
    return out;
}

// ---------------------------------------------------------------------------------------------------------------------
// Data sets
// ---------------------------------------------------------------------------------------------------------------------
MPoly genEllipse(size_t n)
{
    return { Poly{ ellipse(0, 0, 100'000, 70'000, n), {} } };
}

MPoly genStar(size_t n)
{
    Ring r;
    for (size_t i = 0; i < n; ++i)
    {
        const double a = 2 * PI * i / n;
        const double rad = (i % 2 == 0) ? 100'000 : 60'000;
        r.push_back(pt(rad * std::cos(a), rad * std::sin(a)));
    }
    return { Poly{ r, {} } };
}

MPoly genComb(size_t n)
{
    // fixed tooth pitch of 200um (100um teeth), so the comb gets longer with n
    const size_t teeth = std::max<size_t>(1, n / 4);
    const int64_t pitch = 200, w = 100, h = 50'000, base = 10'000;
    Ring r;
    r.push_back({ 0, 0 });
    r.push_back({ static_cast<int64_t>(teeth) * pitch, 0 });
    for (size_t i = teeth; i-- > 0;)
    {
        const int64_t x = static_cast<int64_t>(i) * pitch;
        r.push_back({ x + w, base });
        r.push_back({ x + w, base + h });
        r.push_back({ x, base + h });
        r.push_back({ x, base });
    }
    r.pop_back(); // last tooth ends on the left edge
    return { Poly{ r, {} } };
}

MPoly genHoles(size_t n, int64_t& pitch_out)
{
    const size_t k = std::max<size_t>(1, std::llround(std::sqrt(n / 64.0)));
    const double size = 200'000;
    const double pitch = size / k;
    pitch_out = std::llround(pitch);
    Poly p;
    p.outer = { { 0, 0 }, { 200'000, 0 }, { 200'000, 200'000 }, { 0, 200'000 } };
    for (size_t i = 0; i < k; ++i)
        for (size_t j = 0; j < k; ++j)
            p.holes.push_back(ellipse((i + 0.5) * pitch, (j + 0.5) * pitch, 0.35 * pitch, 0.35 * pitch, 64, false));
    return { p };
}

MPoly genCircleGrid(size_t n)
{
    const size_t k = std::max<size_t>(1, std::llround(std::sqrt(n / 32.0)));
    MPoly mp;
    for (size_t i = 0; i < k; ++i)
        for (size_t j = 0; j < k; ++j)
            mp.push_back(Poly{ ellipse(i * 2000.0, j * 2000.0, 700, 700, 32), {} });
    return mp;
}

MPoly genJitterCircle(size_t n)
{
    std::mt19937_64 rng(1234);
    std::uniform_real_distribution<double> jitter(-2.0, 2.0);
    Ring r;
    for (size_t i = 0; i < n; ++i)
    {
        const double a = 2 * PI * i / n;
        const double rad = 100'000 + jitter(rng);
        r.push_back(pt(rad * std::cos(a), rad * std::sin(a)));
    }
    return { Poly{ r, {} } };
}

MPoly genSquares(size_t n)
{
    // k x k squares of 1mm at a 2mm pitch, 4 vertices each, axis aligned
    const size_t k = std::max<size_t>(1, std::llround(std::sqrt(n / 4.0)));
    MPoly mp;
    for (size_t i = 0; i < k; ++i)
        for (size_t j = 0; j < k; ++j)
        {
            const int64_t x = i * 2000, y = j * 2000;
            mp.push_back(Poly{ { { x, y }, { x + 1000, y }, { x + 1000, y + 1000 }, { x, y + 1000 } }, {} });
        }
    return mp;
}

MPoly genSelfIntersecting(size_t n)
{
    // closed curve winding 3 times around the origin with a varying radius: many self crossings, winding numbers up to 3
    Ring r;
    for (size_t i = 0; i < n; ++i)
    {
        const double t = 6 * PI * i / n;
        const double rad = 100'000 * (0.6 + 0.3 * std::sin(t * 7.0 / 3.0));
        r.push_back(pt(rad * std::cos(t), rad * std::sin(t)));
    }
    return { Poly{ r, {} } };
}

MPoly loadSliceTxt(const std::string& path)
{
    std::ifstream in(path);
    std::string line;
    std::vector<Ring> rings(1);
    while (std::getline(in, line))
    {
        if (line.starts_with("v "))
        {
            std::istringstream ss(line.substr(2));
            Pt p;
            ss >> p.x >> p.y;
            rings.back().push_back(p);
        }
        else if (line.starts_with("x") && ! rings.back().empty())
            rings.emplace_back();
    }
    if (rings.back().empty())
        rings.pop_back();
    return normalise(rings);
}

MPoly loadWkt(const std::string& path)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    bg::model::polygon<BPoint> p;
    bg::read_wkt(ss.str(), p);
    std::vector<Ring> rings;
    auto conv = [&](const auto& br)
    {
        Ring r;
        for (const auto& q : br)
            r.push_back({ q.x(), q.y() });
        if (r.size() > 1 && r.front().x == r.back().x && r.front().y == r.back().y)
            r.pop_back();
        rings.push_back(r);
    };
    conv(p.outer());
    for (const auto& h : p.inners())
        conv(h);
    return normalise(rings);
}

std::string sizeLabel(size_t n)
{
    if (n >= 1'000'000)
        return std::to_string(n / 1'000'000) + "M";
    if (n >= 1000)
        return std::to_string(n / 1000) + "k";
    return std::to_string(n);
}

std::vector<Case> buildCases(size_t max_size)
{
    std::vector<Case> cases;
    for (size_t n : { 1'000ul, 10'000ul, 100'000ul, 1'000'000ul })
    {
        if (n > max_size)
            continue;
        const std::string sl = sizeLabel(n);
        {
            auto a = genEllipse(n);
            cases.push_back({ "ellipse", sl, a, transform(a, 7, 15'000, 8'000) });
        }
        {
            auto a = genStar(n);
            // rotate by half a spike so that every edge crosses edges of the other operand
            cases.push_back({ "star", sl, a, transform(a, 360.0 / n, 1'000, 500) });
        }
        {
            auto a = genComb(n);
            cases.push_back({ "comb", sl, a, transform(a, 0, 50, 10'000) });
        }
        {
            int64_t pitch;
            auto a = genHoles(n, pitch);
            cases.push_back({ "disc_with_holes", sl, a, transform(a, 1, pitch * 0.5, pitch * 0.3) });
        }
        {
            auto a = genCircleGrid(n);
            cases.push_back({ "circle_grid", sl, a, transform(a, 0.5, 1'000, 500) });
        }
        {
            auto a = genJitterCircle(n);
            cases.push_back({ "jitter_circle", sl, a, transform(a, 3, 2'000, 1'000) });
        }
        {
            auto a = genSquares(n);
            cases.push_back({ "squares_coincident", sl, a, transform(a, 0, 500, 0) }); // overlapping, coincident edges
            cases.push_back({ "squares_touching", sl, a, transform(a, 0, 1'000, 0) }); // edge-to-edge contact only
        }
        {
            auto a = genSelfIntersecting(n);
            cases.push_back({ "self_intersecting", sl, a, transform(a, 5, 3'000, 2'000) });
        }
    }
    {
        auto a = loadSliceTxt(std::string(DATA_DIR) + "/tests/resources/slice_polygon_4.txt");
        cases.push_back({ "real_slice_polygon_4", "real", a, transform(a, 2, 300, 300) });
    }
    {
        auto a = loadWkt(std::string(DATA_DIR) + "/benchmark/holes.wkt");
        cases.push_back({ "real_holes_wkt", "real", a, transform(a, 2, 300, 300) });
    }
    return cases;
}

// ---------------------------------------------------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------------------------------------------------
struct Metrics
{
    double area = 0;
    size_t rings = 0;
    size_t vertices = 0;
    std::string valid = "n/a";
};

Metrics metrics(const ClipperLib::Paths& p)
{
    Metrics m;
    for (const auto& r : p)
    {
        m.area += ClipperLib::Area(r);
        m.vertices += r.size();
    }
    m.rings = p.size();
    return m;
}

Metrics metrics(const Clipper2Lib::Paths64& p)
{
    Metrics m;
    for (const auto& r : p)
    {
        m.area += Clipper2Lib::Area(r);
        m.vertices += r.size();
    }
    m.rings = p.size();
    return m;
}

Metrics metrics(const BMPoly& p)
{
    Metrics m;
    m.area = bg::area(p);
    for (const auto& q : p)
    {
        m.rings += 1 + q.inners().size();
        m.vertices += q.outer().size() - 1;
        for (const auto& h : q.inners())
            m.vertices += h.size() - 1;
    }
    std::string msg;
    m.valid = bg::is_valid(p, msg) ? "valid" : ("INVALID: " + msg);
    std::replace(m.valid.begin(), m.valid.end(), ',', ';');
    return m;
}

enum class Join
{
    Miter,
    Round,
    Square
};
const char* joinName(Join j)
{
    return j == Join::Miter ? "miter" : j == Join::Round ? "round" : "square";
}

constexpr double MITER_LIMIT = 1.2; // CuraEngine default (Shape::offset)
constexpr double ARC_TOLERANCE = 10.0; // CuraEngine value, in um

ClipperLib::JoinType c1Join(Join j)
{
    return j == Join::Miter ? ClipperLib::jtMiter : j == Join::Round ? ClipperLib::jtRound : ClipperLib::jtSquare;
}
Clipper2Lib::JoinType c2Join(Join j)
{
    return j == Join::Miter ? Clipper2Lib::JoinType::Miter : j == Join::Round ? Clipper2Lib::JoinType::Round : Clipper2Lib::JoinType::Square;
}

// Same number of segments per full circle that Clipper derives from the arc tolerance
int pointsPerCircle(double delta)
{
    const double d = std::abs(delta);
    return std::max(4, static_cast<int>(std::ceil(PI / std::acos(1.0 - std::min(ARC_TOLERANCE, d) / d))));
}

// ---------------------------------------------------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------------------------------------------------
double g_last_ms = 0; // duration of the last operation, excluding conversions and metrics

double elapsedMs(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

struct Timing
{
    double median_ms = 0;
    int reps = 0;
};

template<typename F>
Timing measure(F&& fn, double first_ms)
{
    // the warm-up run is only used as a sample when it is slow enough on its own
    std::vector<double> t;
    double total = 0;
    if (first_ms > 3000)
        return { first_ms, 1 };
    while (true)
    {
        fn();
        const double ms = g_last_ms;
        t.push_back(ms);
        total += ms;
        // at least 5 repetitions (or 1 if a single run takes > 3s), at most 31, stop after ~1.5s
        if (t.size() >= 31 || ms > 3000 || (t.size() >= 5 && total > 1500) || (t.size() >= 3 && total > 15000))
            break;
    }
    std::sort(t.begin(), t.end());
    return { t[t.size() / 2], static_cast<int>(t.size()) };
}

// ---------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    const std::string out_path = argc > 1 ? argv[1] : "results.csv";
    const size_t max_size = argc > 2 ? std::stoul(argv[2]) : 1'000'000;
    const std::string filter = argc > 3 ? argv[3] : "";
    constexpr double SKIP_AFTER_MS = 10'000; // don't run larger sizes for a (case, op, lib) once a run took this long

    std::ofstream csv(out_path);
    csv << "case,size,input_vertices,op,lib,status,median_ms,reps,out_rings,out_vertices,area,area_rel_diff_vs_clipper2,ring_diff_vs_clipper2,boost_validity\n";

    std::cerr << "Building data sets...\n";
    auto cases = buildCases(max_size);
    std::map<std::string, bool> skip; // key: case|op|lib

    for (const auto& c : cases)
    {
        if (! filter.empty() && c.name.find(filter) == std::string::npos)
            continue;
        const auto a1 = toC1(c.a), b1 = toC1(c.b);
        const auto a2 = toC2(c.a), b2 = toC2(c.b);
        auto ab = toBG(c.a), bb = toBG(c.b);
        std::string msg;
        const bool a_valid = bg::is_valid(ab, msg);
        std::string msg_b;
        const bool b_valid = bg::is_valid(bb, msg_b);
        const size_t nv = vertexCount(c.a);
        std::cerr << "== " << c.name << " " << c.size_label << " (" << nv << " vertices, " << c.a.size() << " polygons)  boost input validity A: "
                  << (a_valid ? "valid" : msg) << " / B: " << (b_valid ? "valid" : msg_b) << "\n";

        struct Op
        {
            std::string name;
            std::function<Metrics(bool)> c1, c2, b;
        };
        std::vector<Op> ops;

        ops.push_back({ "intersection",
                        [&](bool wm)
                        {
                            const auto t0 = std::chrono::steady_clock::now();
                            ClipperLib::Paths r;
                            ClipperLib::Clipper cl;
                            cl.AddPaths(a1, ClipperLib::ptSubject, true);
                            cl.AddPaths(b1, ClipperLib::ptClip, true);
                            cl.Execute(ClipperLib::ctIntersection, r, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
                            g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                        },
                        [&](bool wm)
                        {
                            const auto t0 = std::chrono::steady_clock::now();
                            auto r = Clipper2Lib::Intersect(a2, b2, Clipper2Lib::FillRule::NonZero);
                            g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                        },
                        [&](bool wm)
                        {
                            const auto t0 = std::chrono::steady_clock::now();
                            BMPoly r;
                            bg::intersection(ab, bb, r);
                            g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                        } });
        ops.push_back({ "union",
                        [&](bool wm)
                        {
                            const auto t0 = std::chrono::steady_clock::now();
                            ClipperLib::Paths r;
                            ClipperLib::Clipper cl;
                            cl.AddPaths(a1, ClipperLib::ptSubject, true);
                            cl.AddPaths(b1, ClipperLib::ptClip, true);
                            cl.Execute(ClipperLib::ctUnion, r, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
                            g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                        },
                        [&](bool wm)
                        {
                            const auto t0 = std::chrono::steady_clock::now();
                            auto r = Clipper2Lib::Union(a2, b2, Clipper2Lib::FillRule::NonZero);
                            g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                        },
                        [&](bool wm)
                        {
                            const auto t0 = std::chrono::steady_clock::now();
                            BMPoly r;
                            bg::union_(ab, bb, r);
                            g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                        } });
        for (Join j : { Join::Miter, Join::Round, Join::Square })
            for (double d : { 100.0, -100.0, 5000.0, -5000.0 })
            {
                Op op;
                op.name = std::string("offset_") + joinName(j) + "_" + (d > 0 ? "+" : "") + std::to_string(static_cast<int>(d));
                op.c1 = [&, j, d](bool wm)
                {
                    const auto t0 = std::chrono::steady_clock::now();
                    ClipperLib::Paths r;
                    ClipperLib::ClipperOffset co(MITER_LIMIT, ARC_TOLERANCE);
                    co.AddPaths(a1, c1Join(j), ClipperLib::etClosedPolygon);
                    co.Execute(r, d);
                    g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                };
                op.c2 = [&, j, d](bool wm)
                {
                    const auto t0 = std::chrono::steady_clock::now();
                    Clipper2Lib::Paths64 r;
                    Clipper2Lib::ClipperOffset co(MITER_LIMIT, ARC_TOLERANCE);
                    co.AddPaths(a2, c2Join(j), Clipper2Lib::EndType::Polygon);
                    co.Execute(d, r);
                    g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                };
                if (j != Join::Square) // boost::geometry has no square join for polygon buffers
                {
                    op.b = [&, j, d](bool wm)
                    {
                        const auto t0 = std::chrono::steady_clock::now();
                        BMPoly r;
                        bg::strategy::buffer::distance_symmetric<double> dist(d);
                        bg::strategy::buffer::side_straight side;
                        bg::strategy::buffer::end_flat end;
                        bg::strategy::buffer::point_circle point(pointsPerCircle(d));
                        if (j == Join::Miter)
                            bg::buffer(ab, r, dist, side, bg::strategy::buffer::join_miter(MITER_LIMIT), end, point);
                        else
                            bg::buffer(ab, r, dist, side, bg::strategy::buffer::join_round(pointsPerCircle(d)), end, point);
                        g_last_ms = elapsedMs(t0);
                    return wm ? metrics(r) : Metrics{};
                    };
                }
                ops.push_back(std::move(op));
            }

        for (const auto& op : ops)
        {
            std::optional<Metrics> ref;
            for (const char* lib : { "clipper2", "clipper1", "boost" })
            {
                const std::string libs(lib);
                const auto& fn = libs == "clipper1" ? op.c1 : libs == "clipper2" ? op.c2 : op.b;
                const std::string key = c.name + "|" + op.name + "|" + libs;
                csv << c.name << "," << c.size_label << "," << nv << "," << op.name << "," << libs << ",";
                if (! fn)
                {
                    csv << "unsupported,,,,,,,,\n";
                    continue;
                }
                if (skip[key])
                {
                    csv << "skipped_too_slow,,,,,,,,\n";
                    continue;
                }
                Metrics m;
                Timing t;
                try
                {
                    m = fn(true); // warm-up run, also gives the result for the correctness check
                    t = measure([&] { (void)fn(false); }, g_last_ms);
                }
                catch (const std::exception& e)
                {
                    std::string w = e.what();
                    std::replace(w.begin(), w.end(), ',', ';');
                    csv << "exception: " << w << ",,,,,,,,\n";
                    std::cerr << "   " << op.name << " " << libs << ": EXCEPTION " << w << "\n";
                    continue;
                }
                if (t.median_ms > SKIP_AFTER_MS)
                    skip[key] = true;
                if (libs == "clipper2")
                    ref = m;
                double rel = 0;
                long ring_diff = 0;
                if (ref)
                {
                    const double denom = std::max(std::abs(ref->area), 1.0);
                    rel = (m.area - ref->area) / denom;
                    ring_diff = static_cast<long>(m.rings) - static_cast<long>(ref->rings);
                }
                csv << "ok," << t.median_ms << "," << t.reps << "," << m.rings << "," << m.vertices << "," << std::llround(m.area) << "," << rel << "," << ring_diff << ","
                    << m.valid << "\n";
                csv.flush();
                std::cerr << "   " << op.name << " " << libs << ": " << t.median_ms << " ms (" << t.reps << " reps) rings=" << m.rings << " area_rel_diff=" << rel
                          << (libs == "boost" ? " " + m.valid : "") << "\n";
            }
        }
    }
    return 0;
}
