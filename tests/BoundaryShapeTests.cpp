// BoundaryShapeTests.cpp : tests for typed shapes and the Boundary settings.

#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>

#include "BoundaryShape.h"
#include "CamJob.h"
#include "Facing.h"
#include "Features.h"
#include "ProfileToolpath.h"

using namespace camcore;

namespace
{
    // The areas the features would make, from closed shapes as faces.
    AreaGeometry AreasOf(const std::vector<Path>& closedPaths)
    {
        std::vector<std::vector<Path>> outlines;
        for (const Path& path : closedPaths)
            outlines.push_back({ ClosedPiece(path) });
        return GatherAreas(outlines, std::vector<bool>(outlines.size(), true));
    }

    // An area's closed outline, as Profiling and Pocketing take it.
    Path Outline(const AreaGeometry& areas, size_t n)
    {
        const ProfileChain chain = ChainProfile(areas.areas[n], areas.level);
        REQUIRE(chain.ok);
        REQUIRE(chain.closed);
        return chain.path;
    }

    void Extent(const Path& path, double& xLow, double& xHigh, double& yLow, double& yHigh)
    {
        PathExtent(path, { 1, 0, 0 }, xLow, xHigh);
        PathExtent(path, { 0, 1, 0 }, yLow, yHigh);
    }

    TypedShape Circle(double x, double y, double diameter)
    {
        TypedShape shape;
        shape.kind = ShapeKind::Circle;
        shape.x = x;
        shape.y = y;
        shape.diameter = diameter;
        return shape;
    }
}

TEST_CASE("typed shapes make closed anticlockwise paths")
{
    TypedShape rectangle;
    rectangle.x = 10;
    rectangle.y = 5;
    rectangle.width = 20;
    rectangle.height = 10;
    rectangle.angle = 90;
    const Path r = ShapePath(rectangle, 3);
    REQUIRE(r.size() == 4);
    CHECK(SignedArea(r) == doctest::Approx(200));
    double xLow, xHigh, yLow, yHigh;
    Extent(r, xLow, xHigh, yLow, yHigh);
    CHECK(xLow == doctest::Approx(5));     // turned 90 degrees: 10 wide in X, 20 in Y
    CHECK(xHigh == doctest::Approx(15));
    CHECK(yLow == doctest::Approx(-5));
    CHECK(yHigh == doctest::Approx(15));
    CHECK(r[0].point.z == 3);

    const Path c = ShapePath(Circle(1, 2, 8), 0);
    REQUIRE(c.size() == 2);
    CHECK(c[0].arc);
    Extent(c, xLow, xHigh, yLow, yHigh);
    CHECK(xLow == doctest::Approx(-3));
    CHECK(yHigh == doctest::Approx(6));
    CHECK(SignedArea(c) == doctest::Approx(3.14159265358979 * 16).epsilon(1e-3));

    TypedShape polygon;
    polygon.kind = ShapeKind::Polygon;
    polygon.points = { { 0, 0, 0 }, { 0, 10, 0 }, { 10, 0, 0 } };     // clockwise
    const Path p = ShapePath(polygon, 0);
    REQUIRE(p.size() == 3);
    CHECK(SignedArea(p) == doctest::Approx(50));

    polygon.points = { { 0, 0, 0 }, { 10, 10, 0 }, { 10, 0, 0 }, { 0, 10, 0 } };   // a bow tie
    CHECK_FALSE(ValidateShape(polygon).empty());
    CHECK(ShapePath(polygon, 0).empty());
    rectangle.width = 0;
    CHECK_FALSE(ValidateShape(rectangle).empty());
    CHECK_FALSE(ValidateShape(Circle(0, 0, 0)).empty());
}

TEST_CASE("an offset grows a circle all round, keeping it round")
{
    AreaGeometry areas = AreasOf({ ShapePath(Circle(0, 0, 12), 0) });
    REQUIRE(areas.ok);
    BoundarySettings settings;
    settings.offset = 20;
    std::string problem;
    REQUIRE(ApplyBoundary(settings, areas, false, problem));
    REQUIRE(areas.areas.size() == 1);
    const Path outline = Outline(areas, 0);
    double xLow, xHigh, yLow, yHigh;
    Extent(outline, xLow, xHigh, yLow, yHigh);
    CHECK(xLow == doctest::Approx(-26));
    CHECK(xHigh == doctest::Approx(26));
    CHECK(yLow == doctest::Approx(-26));
    CHECK(yHigh == doctest::Approx(26));
    bool arcs = false;
    for (const PathVertex& vertex : outline)
        arcs = arcs || vertex.arc;
    CHECK(arcs);

    // And Facing closes it too.
    const Path closed = ClosePath(AreaPieces(areas.areas[0]), areas.level);
    CHECK(SignedArea(closed) == doctest::Approx(3.14159265358979 * 26 * 26).epsilon(1e-3));
}

TEST_CASE("an inward offset shrinks a square, and too much leaves nothing")
{
    TypedShape square;
    square.width = 10;
    square.height = 10;
    AreaGeometry areas = AreasOf({ ShapePath(square, 2) });
    BoundarySettings settings;
    settings.offset = -2;
    std::string problem;
    REQUIRE(ApplyBoundary(settings, areas, false, problem));
    const Path outline = Outline(areas, 0);
    double xLow, xHigh, yLow, yHigh;
    Extent(outline, xLow, xHigh, yLow, yHigh);
    CHECK(xLow == doctest::Approx(-3));
    CHECK(xHigh == doctest::Approx(3));
    CHECK(yHigh == doctest::Approx(3));
    CHECK(outline[0].point.z == doctest::Approx(2));

    AreaGeometry again = AreasOf({ ShapePath(square, 0) });
    settings.offset = -6;
    CHECK_FALSE(ApplyBoundary(settings, again, false, problem));
    CHECK_FALSE(problem.empty());
}

TEST_CASE("a bounding rectangle and a convex hull go round every area")
{
    // Three holes in an L.
    const std::vector<Path> holes = { ShapePath(Circle(0, 0, 2), 0), ShapePath(Circle(10, 0, 2), 0),
                                      ShapePath(Circle(0, 10, 2), 0) };
    AreaGeometry areas = AreasOf(holes);
    REQUIRE(areas.areas.size() == 3);
    BoundarySettings settings;
    settings.shape = BoundaryShape::Rectangle;
    std::string problem;
    REQUIRE(ApplyBoundary(settings, areas, false, problem));
    REQUIRE(areas.areas.size() == 1);
    Path outline = Outline(areas, 0);
    CHECK(outline.size() == 4);
    CHECK(SignedArea(outline) == doctest::Approx(12 * 12).epsilon(1e-3));

    areas = AreasOf(holes);
    settings.shape = BoundaryShape::ConvexHull;
    settings.offset = 1;
    REQUIRE(ApplyBoundary(settings, areas, false, problem));
    REQUIRE(areas.areas.size() == 1);
    outline = Outline(areas, 0);
    double xLow, xHigh, yLow, yHigh;
    Extent(outline, xLow, xHigh, yLow, yHigh);
    CHECK(xLow == doctest::Approx(-2).epsilon(1e-3));
    CHECK(xHigh == doctest::Approx(12).epsilon(1e-3));
    CHECK(yHigh == doctest::Approx(12).epsilon(1e-3));
    // Smaller than the rectangle: the corner at (10, 10) is cut off.
    CHECK(SignedArea(outline) < 14.0 * 14.0 - 20.0);

    // A single straight edge has no width for a rectangle.
    AreaGeometry line = GatherAreas({ { PathFromPoints({ { 0, 0, 0 }, { 10, 0, 0 } }) } }, { false });
    settings.shape = BoundaryShape::Rectangle;
    settings.offset = 0;
    CHECK_FALSE(ApplyBoundary(settings, line, false, problem));
}

TEST_CASE("an offset leaves an open profile alone unless gaps are closed")
{
    const Path open = PathFromPoints({ { 0, 0, 0 }, { 10, 0, 0 }, { 10, 10, 0 } });
    BoundarySettings settings;
    settings.offset = 2;
    std::string problem;

    AreaGeometry areas = GatherAreas({ { open } }, { false });
    REQUIRE(ApplyBoundary(settings, areas, false, problem));
    const ProfileChain chain = ChainProfile(areas.areas[0], areas.level);
    REQUIRE(chain.ok);
    CHECK_FALSE(chain.closed);

    // Facing bridges the gap first: a triangle grown by 2.
    areas = GatherAreas({ { open } }, { false });
    REQUIRE(ApplyBoundary(settings, areas, true, problem));
    const Path outline = Outline(areas, 0);
    double xLow, xHigh, yLow, yHigh;
    Extent(outline, xLow, xHigh, yLow, yHigh);
    CHECK(xHigh == doctest::Approx(12));
    CHECK(yLow == doctest::Approx(-2));
}

TEST_CASE("typed shapes and the boundary settings save and load")
{
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "CAMForAlibre-shapes-test.cam";
    Job job;
    job.assembly.name = "Plate";
    job.assembly.guid = "{guid}";
    SequenceStep origin;
    origin.type = StepType::SetOrigin;
    origin.id = NewStableId();
    origin.name = "Set Origin 1";
    job.sequence.push_back(origin);
    SequenceStep group;
    group.type = StepType::Group;
    group.id = NewStableId();
    group.name = "Folder 1";
    SequenceStep facing;
    facing.type = StepType::Facing;
    facing.id = NewStableId();
    facing.name = "Facing 1";
    TypedShape rectangle;
    rectangle.x = 1;
    rectangle.y = 2;
    rectangle.z = 3;
    rectangle.width = 40;
    rectangle.height = 30;
    rectangle.angle = 15;
    TypedShape polygon;
    polygon.kind = ShapeKind::Polygon;
    polygon.points = { { 0, 0, 0 }, { 5, 0, 0 }, { 0, 5, 0 } };
    facing.shapes = { rectangle, Circle(4, 5, 6), polygon };
    facing.boundary.shape = BoundaryShape::ConvexHull;
    facing.boundary.offset = -1.5;
    group.children.push_back(facing);
    job.sequence.push_back(group);
    REQUIRE(SaveJob(job, path).ok);

    const LoadResult loaded = LoadJob(path);
    REQUIRE(loaded.status == LoadStatus::Ok);
    const SequenceStep& back = loaded.job.sequence[1].children[0];
    CHECK(SameSettings(facing, back));
    REQUIRE(back.shapes.size() == 3);
    CHECK(back.shapes[0].angle == 15);
    CHECK(back.shapes[0].z == 3);
    CHECK(back.shapes[1].kind == ShapeKind::Circle);
    CHECK(back.shapes[1].diameter == 6);
    CHECK(back.shapes[2].points.size() == 3);
    CHECK(back.boundary.shape == BoundaryShape::ConvexHull);
    CHECK(back.boundary.offset == -1.5);

    SequenceStep changed = back;
    changed.boundary.offset = 0;
    CHECK_FALSE(SameSettings(changed, back));
    CHECK(TakesBoundary(StepType::Pocketing));
    CHECK_FALSE(TakesBoundary(StepType::Engrave));
    CHECK(TakesShapes(StepType::ParallelFinishing));
    CHECK_FALSE(TakesShapes(StepType::Drill));

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

TEST_CASE("a whole sketch is a feature for area operations, known by its name")
{
    PickedGeometry sketch;
    sketch.kind = PickKind::Sketch;
    sketch.targetName = "Sketch<1>";
    sketch.occurrenceName = "Plate<1>";
    sketch.point = { 1, 2, 3 };
    CHECK(FeatureAllowed(FeatureFilter::FacingArea, PickKind::Sketch));
    CHECK(FeatureAllowed(FeatureFilter::ProjectedArea, PickKind::Sketch));
    CHECK_FALSE(FeatureAllowed(FeatureFilter::Points, PickKind::Sketch));
    CHECK(CheckAreaFeatures({ sketch }, "Facing").ok);

    PickedGeometry other = sketch;
    other.targetKey.hex = "0102";       // an assembly sketch's key doesn't bind again; the name does
    CHECK(SameFeature(sketch, other));
    other.targetName = "Sketch<2>";
    CHECK_FALSE(SameFeature(sketch, other));

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "CAMForAlibre-sketch-test.cam";
    Job job;
    job.assembly.name = "Plate";
    job.assembly.guid = "{guid}";
    SequenceStep group;
    group.type = StepType::Group;
    group.id = NewStableId();
    group.name = "Folder 1";
    SequenceStep pocket;
    pocket.type = StepType::Pocketing;
    pocket.id = NewStableId();
    pocket.name = "Pocketing 1";
    pocket.features = { sketch };
    group.children.push_back(pocket);
    job.sequence.push_back(group);
    REQUIRE(SaveJob(job, path).ok);
    const LoadResult loaded = LoadJob(path);
    REQUIRE(loaded.status == LoadStatus::Ok);
    const PickedGeometry& back = loaded.job.sequence[0].children[0].features.at(0);
    CHECK(back.kind == PickKind::Sketch);
    CHECK(back.targetName == "Sketch<1>");
    CHECK(back.occurrenceName == "Plate<1>");
    CHECK(SameFeature(back, sketch));
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}
