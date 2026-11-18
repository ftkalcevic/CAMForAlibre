// BoundaryShape.cpp : typed shapes and the Boundary settings' changes to an operation's areas.

#include "BoundaryShape.h"

#include "ProfileToolpath.h"

#include <algorithm>
#include <cmath>

namespace camcore
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kSmallest = 1e-6;      // narrower than this is no width

        double Cross(const Vec3& o, const Vec3& a, const Vec3& b)
        {
            return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
        }

        // The convex hull in X and Y, anticlockwise, by Andrew's monotone chain.
        Polyline ConvexHull(Polyline points)
        {
            std::sort(points.begin(), points.end(), [](const Vec3& a, const Vec3& b) {
                return a.x < b.x || (a.x == b.x && a.y < b.y);
            });
            if (points.size() < 3)
                return points;
            Polyline hull(2 * points.size());
            size_t k = 0;
            for (size_t i = 0; i < points.size(); ++i)
            {
                while (k >= 2 && Cross(hull[k - 2], hull[k - 1], points[i]) <= kSmallest * kSmallest)
                    --k;
                hull[k++] = points[i];
            }
            for (size_t i = points.size() - 1, lower = k + 1; i-- > 0;)
            {
                while (k >= lower && Cross(hull[k - 2], hull[k - 1], points[i]) <= kSmallest * kSmallest)
                    --k;
                hull[k++] = points[i];
            }
            hull.resize(k - 1);
            return hull;
        }

        // True if segments ab and cd cross, not counting shared ends.
        bool SegmentsCross(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
        {
            const double d1 = Cross(c, d, a), d2 = Cross(c, d, b);
            const double d3 = Cross(a, b, c), d4 = Cross(a, b, d);
            return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
                   ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
        }

        Path PolygonPath(const Polyline& corners, double level)
        {
            Polyline flat = corners;
            for (Vec3& p : flat)
                p.z = level;
            if (SignedArea(flat) < 0.0)
                std::reverse(flat.begin(), flat.end());
            return PathFromPoints(flat);
        }
    }

    std::string ValidateBoundary(const BoundarySettings& settings)
    {
        if (!std::isfinite(settings.offset))
            return "The boundary's offset must be a number.";
        return std::string();
    }

    Path ClosedPiece(const Path& path)
    {
        Path piece = path;
        if (!piece.empty())
        {
            PathVertex end;
            end.point = piece.front().point;
            piece.push_back(end);
        }
        return piece;
    }

    bool ApplyBoundary(const BoundarySettings& settings, AreaGeometry& areas, bool closeGaps, std::string& problem)
    {
        problem.clear();
        if (settings.IsDefault())
            return true;
        const std::string invalid = ValidateBoundary(settings);
        if (!invalid.empty())
        {
            problem = invalid;
            return false;
        }

        if (settings.shape != BoundaryShape::AsPicked)
        {
            Polyline points;
            for (const auto& area : areas.areas)
            {
                for (const std::vector<Path>& feature : area)
                {
                    for (const Path& piece : feature)
                    {
                        const Polyline sampled = PathPoints(piece, false);
                        points.insert(points.end(), sampled.begin(), sampled.end());
                    }
                }
            }

            Polyline outline;
            if (settings.shape == BoundaryShape::Rectangle && !points.empty())
            {
                double xLow = points.front().x, xHigh = xLow, yLow = points.front().y, yHigh = yLow;
                for (const Vec3& p : points)
                {
                    xLow = (std::min)(xLow, p.x);
                    xHigh = (std::max)(xHigh, p.x);
                    yLow = (std::min)(yLow, p.y);
                    yHigh = (std::max)(yHigh, p.y);
                }
                if (xHigh - xLow > kSmallest && yHigh - yLow > kSmallest)
                    outline = { { xLow, yLow, 0.0 }, { xHigh, yLow, 0.0 }, { xHigh, yHigh, 0.0 }, { xLow, yHigh, 0.0 } };
            }
            else if (settings.shape == BoundaryShape::ConvexHull)
            {
                outline = ConvexHull(points);
                if (outline.size() < 3 || std::fabs(SignedArea(outline)) < kSmallest)
                    outline.clear();
            }
            if (outline.empty())
            {
                problem = (settings.shape == BoundaryShape::Rectangle)
                              ? "A bounding rectangle needs features that reach across in both X and Y."
                              : "A convex hull needs features that aren't all in a line.";
                return false;
            }
            areas.areas = { { { ClosedPiece(PolygonPath(outline, areas.level)) } } };
        }

        if (settings.offset == 0.0)
            return true;

        std::vector<std::vector<std::vector<Path>>> changed;
        for (size_t n = 0; n < areas.areas.size(); ++n)
        {
            const std::string name =
                (areas.areas.size() > 1) ? "Area " + std::to_string(n + 1) + ": " : std::string();
            Path closed;
            const ProfileChain chain = ChainProfile(areas.areas[n], areas.level);
            if (chain.ok && chain.closed)
                closed = chain.path;
            else if (closeGaps)
                closed = ClosePath(AreaPieces(areas.areas[n]), areas.level);
            else
            {
                changed.push_back(areas.areas[n]);      // an open profile stays as it is
                continue;
            }
            if (closed.size() < 2)
            {
                problem = name + "the features don't enclose an area to offset.";
                return false;
            }

            // Outward is to the right of an anticlockwise path.
            const double distance = (SignedArea(closed) > 0.0) ? -settings.offset : settings.offset;
            size_t kept = 0;
            for (const ProfileRun& run : OffsetProfile(closed, true, distance))
            {
                if (!run.closed || run.path.size() < 2)
                    continue;
                Path path = run.path;
                for (PathVertex& vertex : path)
                {
                    vertex.point.z = areas.level;
                    vertex.centre.z = areas.level;
                }
                changed.push_back({ { ClosedPiece(path) } });
                ++kept;
            }
            if (kept == 0)
            {
                problem = name + "an offset of " + std::to_string(settings.offset) + " mm leaves nothing to cut.";
                return false;
            }
        }
        areas.areas = std::move(changed);
        return true;
    }

    std::string ValidateShape(const TypedShape& shape)
    {
        if (!std::isfinite(shape.x) || !std::isfinite(shape.y) || !std::isfinite(shape.z))
            return "The shape's position must be numbers.";
        switch (shape.kind)
        {
        case ShapeKind::Rectangle:
            if (!(shape.width > kSmallest) || !(shape.height > kSmallest))
                return "The rectangle's width and height must be more than 0.";
            if (!std::isfinite(shape.angle))
                return "The rectangle's angle must be a number.";
            return std::string();
        case ShapeKind::Circle:
            if (!(shape.diameter > kSmallest))
                return "The circle's diameter must be more than 0.";
            return std::string();
        case ShapeKind::Polygon:
        {
            if (shape.points.size() < 3)
                return "A polygon needs at least 3 points.";
            for (const Vec3& p : shape.points)
            {
                if (!std::isfinite(p.x) || !std::isfinite(p.y))
                    return "The polygon's points must be numbers.";
            }
            if (std::fabs(SignedArea(shape.points)) < kSmallest)
                return "The polygon's points must enclose an area.";
            const size_t n = shape.points.size();
            for (size_t i = 0; i < n; ++i)
            {
                for (size_t j = i + 1; j < n; ++j)
                {
                    if (SegmentsCross(shape.points[i], shape.points[(i + 1) % n], shape.points[j],
                                      shape.points[(j + 1) % n]))
                        return "The polygon's sides mustn't cross.";
                }
            }
            return std::string();
        }
        }
        return "Unknown shape.";
    }

    Path ShapePath(const TypedShape& shape, double level)
    {
        if (!ValidateShape(shape).empty())
            return Path();
        switch (shape.kind)
        {
        case ShapeKind::Rectangle:
        {
            const double a = shape.angle * kPi / 180.0;
            const Vec3 u{ std::cos(a) * shape.width / 2.0, std::sin(a) * shape.width / 2.0, 0.0 };
            const Vec3 v{ -std::sin(a) * shape.height / 2.0, std::cos(a) * shape.height / 2.0, 0.0 };
            const Vec3 c{ shape.x, shape.y, 0.0 };
            return PolygonPath({ c - u - v, c + u - v, c + u + v, c - u + v }, level);
        }
        case ShapeKind::Circle:
        {
            const double r = shape.diameter / 2.0;
            Path path(2);
            path[0].point = { shape.x + r, shape.y, level };
            path[1].point = { shape.x - r, shape.y, level };
            for (PathVertex& vertex : path)
            {
                vertex.arc = true;
                vertex.centre = { shape.x, shape.y, level };
                vertex.axis = { 0.0, 0.0, 1.0 };
            }
            return path;
        }
        case ShapeKind::Polygon:
            return PolygonPath(shape.points, level);
        }
        return Path();
    }

    const char* ShapeKindTitle(ShapeKind kind)
    {
        switch (kind)
        {
        case ShapeKind::Circle:
            return "Circle";
        case ShapeKind::Polygon:
            return "Polygon";
        default:
            return "Rectangle";
        }
    }

    const char* ShapeKindName(ShapeKind kind)
    {
        switch (kind)
        {
        case ShapeKind::Circle:
            return "circle";
        case ShapeKind::Polygon:
            return "polygon";
        default:
            return "rectangle";
        }
    }

    bool ShapeKindFromName(const std::string& name, ShapeKind& kind)
    {
        for (ShapeKind k : { ShapeKind::Rectangle, ShapeKind::Circle, ShapeKind::Polygon })
        {
            if (name == ShapeKindName(k))
            {
                kind = k;
                return true;
            }
        }
        return false;
    }

    const char* BoundaryShapeName(BoundaryShape shape)
    {
        switch (shape)
        {
        case BoundaryShape::Rectangle:
            return "rectangle";
        case BoundaryShape::ConvexHull:
            return "convexHull";
        default:
            return "asPicked";
        }
    }

    bool BoundaryShapeFromName(const std::string& name, BoundaryShape& shape)
    {
        for (BoundaryShape s : { BoundaryShape::AsPicked, BoundaryShape::Rectangle, BoundaryShape::ConvexHull })
        {
            if (name == BoundaryShapeName(s))
            {
                shape = s;
                return true;
            }
        }
        return false;
    }
}
