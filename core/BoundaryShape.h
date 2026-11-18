// BoundaryShape.h : changing the area an operation cuts beyond its picked geometry. No Alibre or MFC code belongs here.
//
// Two ways: shapes typed in the operation (a rectangle, circle or polygon in the
// active origin's X and Y), which join the picked features as areas of their
// own; and the Boundary settings, which turn the areas into their bounding
// rectangle or convex hull and offset them. Everything is in the origin's
// frame, in millimetres.

#pragma once

#include <string>
#include <vector>

#include "Boundary.h"

namespace camcore
{
    struct AreaGeometry;

    enum class BoundaryShape
    {
        AsPicked,       // the areas as the features make them
        Rectangle,      // one rectangle round everything, square to the origin's X and Y
        ConvexHull      // one convex outline round everything
    };

    struct BoundarySettings
    {
        BoundaryShape shape = BoundaryShape::AsPicked;
        double offset = 0.0;    // each closed area grown by this (shrunk if negative), corners rounded

        bool IsDefault() const { return shape == BoundaryShape::AsPicked && offset == 0.0; }
    };

    // Empty if the settings can be used, otherwise what's wrong.
    std::string ValidateBoundary(const BoundarySettings& settings);

    // The areas changed by the settings. Rectangle and Convex Hull make one area
    // from every piece, open or closed. Offset applies to each closed area, and to
    // the pieces ClosePath can close when closeGaps is set (as Facing does); open
    // profiles are left as they are. An inward offset can split an area into
    // several. Each changed area is a single closed piece, its first point repeated
    // at its end, at the areas' level. False, with a problem, if nothing is left.
    bool ApplyBoundary(const BoundarySettings& settings, AreaGeometry& areas, bool closeGaps, std::string& problem);

    // A closed path as one piece whose ends meet, as ChainProfile and ClosePath take it.
    Path ClosedPiece(const Path& path);

    enum class ShapeKind
    {
        Rectangle,
        Circle,
        Polygon
    };

    // A shape typed in the operation, in the active origin's frame.
    struct TypedShape
    {
        ShapeKind kind = ShapeKind::Rectangle;
        double x = 0.0, y = 0.0;    // rectangle and circle: the centre
        double z = 0.0;             // its height, used only when there are no picked features
        double width = 10.0;        // rectangle: along the angle
        double height = 10.0;       // rectangle: square to it
        double angle = 0.0;         // rectangle: degrees anticlockwise from X
        double diameter = 10.0;     // circle
        std::vector<Vec3> points;   // polygon: its corners in order (z unused)
    };

    // Empty if the shape can be used, otherwise what's wrong.
    std::string ValidateShape(const TypedShape& shape);

    // The shape as a closed path at level, anticlockwise from +Z, without its first
    // point repeated. A circle is two half arcs. Empty if the shape isn't valid.
    Path ShapePath(const TypedShape& shape, double level);

    // "Rectangle", "Circle" or "Polygon", and the name used in the .cam file.
    const char* ShapeKindTitle(ShapeKind kind);
    const char* ShapeKindName(ShapeKind kind);
    bool ShapeKindFromName(const std::string& name, ShapeKind& kind);
    const char* BoundaryShapeName(BoundaryShape shape);
    bool BoundaryShapeFromName(const std::string& name, BoundaryShape& shape);
}
