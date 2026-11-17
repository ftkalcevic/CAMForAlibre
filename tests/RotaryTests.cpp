// RotaryTests.cpp : tests for Rotate steps, indexing the 4th axis, and 4 axis posts.

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "Rotary.h"
#include "ScriptedPost.h"
#include "Simulation.h"
#include "Stock.h"

using namespace camcore;
namespace fs = std::filesystem;

namespace
{
    bool Near(const Vec3& a, const Vec3& b, double tolerance = 1e-9)
    {
        return Length(a - b) <= tolerance;
    }

    // A Set Origin at the assembly's origin, along its axes: a circle in XY, its centre the position.
    SequenceStep WorkZero()
    {
        SequenceStep step;
        step.type = StepType::SetOrigin;
        step.id = NewStableId();
        step.name = "Work zero";
        step.origin.plane.kind = PickKind::CircleEdge;
        step.origin.plane.point = { 0, 0, 0 };
        step.origin.plane.direction = { 0, 0, 1 };
        step.origin.plane.radius = 10;
        return step;
    }

    SequenceStep RotateStep(double angle, OriginAfterRotate after = OriginAfterRotate::Keep)
    {
        SequenceStep step;
        step.type = StepType::Rotate;
        step.id = NewStableId();
        step.name = "Rotate";
        step.rotate.angle = angle;
        step.rotate.originAfter = after;
        return step;
    }

    OrientationContext FourAxes(RotaryAxis rotary = RotaryAxis::PlusY)
    {
        OrientationContext context;
        context.axes = 4;
        context.rotary = rotary;
        return context;
    }

    std::string ReadPost(const char* name)
    {
        const fs::path path = fs::path(__FILE__).parent_path().parent_path() / "posts" / name;
        std::ifstream in(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }

    ToolpathMove Move(MoveKind kind, Vec3 to, double feed = 0)
    {
        ToolpathMove move;
        move.kind = kind;
        move.to = to;
        move.feed = feed;
        return move;
    }

    SequenceStep Operation(const std::string& name, const std::vector<ToolpathMove>& moves)
    {
        SequenceStep step;
        step.type = StepType::Drill;
        step.name = name;
        step.feeds.spindleRpm = 10000;
        step.toolpath.moves = moves;
        return step;
    }

    bool Has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

    class TempFile
    {
    public:
        TempFile()
        {
            static std::atomic<int> counter{ 0 };
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            m_path = fs::temp_directory_path() /
                     ("camcore_rotary_" + std::to_string(stamp) + "_" + std::to_string(counter++) + ".cam");
        }
        ~TempFile()
        {
            std::error_code ignored;
            fs::remove(m_path, ignored);
        }
        const fs::path& Path() const { return m_path; }

    private:
        fs::path m_path;
    };
}

TEST_CASE("turning about an axis is right-handed")
{
    CHECK(Near(RotateAbout({ 0, 0, 1 }, { 0, 1, 0 }, 90), { 1, 0, 0 }, 1e-12));
    CHECK(Near(RotateAbout({ 0, 1, 0 }, { 1, 0, 0 }, 90), { 0, 0, 1 }, 1e-12));
    CHECK(Near(RotateAbout({ 0, 0, 1 }, { 0, -1, 0 }, 90), { -1, 0, 0 }, 1e-12));
    CHECK(Near(RotaryDirection(RotaryAxis::MinusX), { -1, 0, 0 }));
    CHECK(Near(RotaryDirection(RotaryAxis::PlusY), { 0, 1, 0 }));
}

TEST_CASE("a Rotate step turns the work zero's frame about the rotary axis")
{
    Job job;
    job.sequence = { WorkZero(), RotateStep(90) };
    const OrientationContext context = FourAxes();

    const FrameResult frame = OrientationFrame(job, 1, context);
    REQUIRE_MESSAGE(frame.ok, frame.message);
    CHECK(Near(frame.frame.origin, { 0, 0, 0 }));
    CHECK(Near(frame.frame.zAxis, { 1, 0, 0 }, 1e-12));
    CHECK(Near(frame.frame.xAxis, { 0, 0, -1 }, 1e-12));
    CHECK(Near(frame.frame.yAxis, { 0, 1, 0 }, 1e-12));

    // About X instead, Z goes to -Y.
    const FrameResult aboutX = OrientationFrame(job, 1, FourAxes(RotaryAxis::PlusX));
    REQUIRE(aboutX.ok);
    CHECK(Near(aboutX.frame.zAxis, { 0, -1, 0 }, 1e-12));

    // Steps below it run under it, as under a Set Origin.
    SequenceStep folder;
    folder.type = StepType::Group;
    folder.id = NewStableId();
    job.sequence.push_back(folder);
    CHECK(ActiveOriginIndex(job, 2) == 1);
    const FrameResult below = FrameAt(job, 2, context);
    REQUIRE(below.ok);
    CHECK(Near(below.frame.zAxis, { 1, 0, 0 }, 1e-12));

    // A suppressed Rotate is skipped by the steps below.
    job.sequence[1].suppressed = true;
    CHECK(ActiveOriginIndex(job, 2) == 0);
}

TEST_CASE("a Rotate can put the origin on top of the stock")
{
    Job job;
    job.sequence = { WorkZero(), RotateStep(90, OriginAfterRotate::TopOfStock) };
    const std::vector<Vec3> stock = BoxCorners({ { -50, -10, -12 }, { 50, 10, 8 } });
    OrientationContext context = FourAxes(RotaryAxis::PlusX);
    context.stockPoints = &stock;

    // About +X by 90, Z points along -Y, so the top is the stock's -Y side.
    const FrameResult frame = OrientationFrame(job, 1, context);
    REQUIRE_MESSAGE(frame.ok, frame.message);
    CHECK(Near(frame.frame.origin, { 0, -10, 0 }, 1e-12));

    // With no stock it can't.
    context.stockPoints = nullptr;
    CHECK_FALSE(OrientationFrame(job, 1, context).ok);
}

TEST_CASE("a Rotate needs a rotary axis and a work zero above it")
{
    Job job;
    job.sequence = { WorkZero(), RotateStep(45) };
    OrientationContext threeAxes;
    const FrameResult refused = OrientationFrame(job, 1, threeAxes);
    CHECK_FALSE(refused.ok);
    CHECK_FALSE(refused.message.empty());

    job.sequence[0].suppressed = true;
    CHECK_FALSE(OrientationFrame(job, 1, FourAxes()).ok);
}

TEST_CASE("the rotary angle that reaches a frame")
{
    const Frame zero;
    Frame frame;
    frame.zAxis = { 1, 0, 0 };
    frame.xAxis = { 0, 0, -1 };
    frame.yAxis = { 0, 1, 0 };

    IndexSolution solution = SolveIndex(zero, FourAxes(), frame);
    REQUIRE(solution.ok);
    CHECK(solution.angle == doctest::Approx(90));

    frame.zAxis = { -1, 0, 0 };
    solution = SolveIndex(zero, FourAxes(), frame);
    REQUIRE(solution.ok);
    CHECK(solution.angle == doctest::Approx(270));

    // Along the work zero's Z it's 0, whatever its X.
    Frame upright;
    upright.xAxis = { 0, 1, 0 };
    upright.yAxis = { -1, 0, 0 };
    solution = SolveIndex(zero, FourAxes(), upright);
    REQUIRE(solution.ok);
    CHECK(solution.angle == doctest::Approx(0));

    // A Z along the axis, or tilted towards it, can't be reached.
    frame.zAxis = { 0, 1, 0 };
    CHECK_FALSE(SolveIndex(zero, FourAxes(), frame).ok);
    frame.zAxis = { std::sqrt(0.5), std::sqrt(0.5), 0 };
    CHECK_FALSE(SolveIndex(zero, FourAxes(), frame).ok);

    // A 3 axis machine reaches only the work zero's Z.
    CHECK(SolveIndex(zero, OrientationContext(), upright).ok);
    frame.zAxis = { 1, 0, 0 };
    CHECK_FALSE(SolveIndex(zero, OrientationContext(), frame).ok);
}

TEST_CASE("the rotary angle of the simulated tool, for turning the view")
{
    const Frame zero;
    // About +Y, right-handed: Z turns towards +X.
    for (double angle : { 0.0, 2.0, 90.0, 180.0, 270.0, 358.0 })
    {
        const Vec3 axis = RotateAbout({ 0, 0, 1 }, { 0, 1, 0 }, angle);
        const IndexSolution solution = RotaryAngleOf(zero, RotaryAxis::PlusY, axis * 3.0);
        REQUIRE(solution.ok);
        CHECK(solution.angle == doctest::Approx(angle).epsilon(1e-9));
    }
    // About +X.
    const IndexSolution aboutX = RotaryAngleOf(zero, RotaryAxis::PlusX, RotateAbout({ 0, 0, 1 }, { 1, 0, 0 }, 90));
    REQUIRE(aboutX.ok);
    CHECK(aboutX.angle == doctest::Approx(90));
    // A swing's steps grow.
    double last = -1;
    for (int s = 0; s <= 45; ++s)
    {
        const IndexSolution step = RotaryAngleOf(zero, RotaryAxis::PlusY, RotateAbout({ 0, 0, 1 }, { 0, 1, 0 }, s * 2.0));
        REQUIRE(step.ok);
        CHECK(step.angle > last);
        last = step.angle;
    }
    CHECK_FALSE(RotaryAngleOf(zero, RotaryAxis::PlusY, { 0, 1, 0 }).ok);
    CHECK_FALSE(RotaryAngleOf(zero, RotaryAxis::PlusY, { 0, 1, 1 }).ok);

    // Turning about a line keeps its points and turns the rest.
    const RowMatrix turn = TurnAboutLine({ 5, 0, 2 }, { 0, 1, 0 }, 90);
    CHECK(Near(TransformPoint({ 5, 7, 2 }, turn), { 5, 7, 2 }, 1e-12));
    CHECK(Near(TransformPoint({ 5, 0, 3 }, turn), { 6, 0, 2 }, 1e-12));
    const RowMatrix back = MultiplyRows(turn, TurnAboutLine({ 5, 0, 2 }, { 0, 1, 0 }, -90));
    CHECK(Near(TransformPoint({ 1, 2, 3 }, back), { 1, 2, 3 }, 1e-12));
}

TEST_CASE("machine points are turned back by the rotary angle")
{
    Frame zero;
    zero.origin = { 5, 0, 0 };
    // A point 20 out along the turned frame's Z (assembly +X) is 20 up on the machine at 90.
    CHECK(Near(MachinePoint(zero, RotaryAxis::PlusY, 90, { 25, 3, 0 }), { 0, 3, 20 }, 1e-12));
    CHECK(Near(WorldFromMachine(zero, RotaryAxis::PlusY, 90, { 0, 3, 20 }), { 25, 3, 0 }, 1e-12));
    CHECK(Near(MachineDirection(zero, RotaryAxis::PlusY, 90, { 1, 0, 0 }), { 0, 0, 1 }, 1e-12));
    CHECK(FurthestFromAxis(zero, RotaryAxis::PlusY, { { 5, 100, 4 }, { 8, 0, 4 } }) == doctest::Approx(5));
}

TEST_CASE("the next index angle turns the way asked")
{
    CHECK(NextIndexAngle(0, 270, IndexDirection::Shortest) == doctest::Approx(-90));
    CHECK(NextIndexAngle(0, 270, IndexDirection::Positive) == doctest::Approx(270));
    CHECK(NextIndexAngle(0, 90, IndexDirection::Negative) == doctest::Approx(-270));
    CHECK(NextIndexAngle(350, 10, IndexDirection::Shortest) == doctest::Approx(370));
    CHECK(NextIndexAngle(-90, 270, IndexDirection::Positive) == doctest::Approx(-90));
    CHECK(NextIndexAngle(0, 180, IndexDirection::Shortest) == doctest::Approx(180));
    CHECK(NextIndexAngle(720, 0, IndexDirection::Negative) == doctest::Approx(720));
}

TEST_CASE("a cylindrical face gives a position on its axis")
{
    PickedGeometry cylinder;
    cylinder.kind = PickKind::CylindricalFace;
    cylinder.point = { 0, 0, 30 };
    cylinder.direction = { 0, 1, 0 };
    cylinder.radius = 15;
    cylinder.clicked = { 12, 40, 39 };
    Vec3 position;
    REQUIRE(PositionFromPick(cylinder, position));
    CHECK(Near(position, { 0, 40, 30 }));

    // As the position of a Set Origin, the work zero sits on the bar's axis.
    OriginDefinition definition;
    definition.plane.kind = PickKind::DesignPlane;
    definition.plane.direction = { 0, 0, 1 };
    definition.position = cylinder;
    const FrameResult frame = BuildFrame(definition);
    REQUIRE(frame.ok);
    CHECK(Near(frame.frame.origin, { 0, 40, 30 }));

    // A cylindrical face picked for the plane alone still needs a position.
    OriginDefinition alone;
    alone.plane = cylinder;
    CHECK_FALSE(BuildFrame(alone).ok);
}

TEST_CASE("Rotate steps and index settings save and load")
{
    TempFile file;
    Job job;
    job.assembly.name = "Bar";
    job.sequence = { WorkZero(), RotateStep(-45, OriginAfterRotate::TopOfStock) };
    job.sequence[0].index.direction = IndexDirection::Negative;
    job.sequence[1].index.automaticRetract = false;
    job.sequence[1].index.retractHeight = 42.5;
    job.sequence[1].index.direction = IndexDirection::Positive;

    REQUIRE(SaveJob(job, file.Path()).ok);
    const LoadResult loaded = LoadJob(file.Path());
    REQUIRE_MESSAGE(loaded.status == LoadStatus::Ok, loaded.message);
    REQUIRE(loaded.job.sequence.size() == 2);
    const SequenceStep& rotate = loaded.job.sequence[1];
    CHECK(rotate.type == StepType::Rotate);
    CHECK(rotate.rotate.angle == -45);
    CHECK(rotate.rotate.originAfter == OriginAfterRotate::TopOfStock);
    CHECK_FALSE(rotate.index.automaticRetract);
    CHECK(rotate.index.retractHeight == 42.5);
    CHECK(rotate.index.direction == IndexDirection::Positive);
    CHECK(loaded.job.sequence[0].index.direction == IndexDirection::Negative);
    CHECK(SameSettings(rotate, job.sequence[1]));

    // A Set Origin saved before version 18 has no index settings, and gets the defaults.
    {
        std::ofstream out(file.Path(), std::ios::binary);
        out << R"({"format":"AlibreCAM","formatVersion":17,"assembly":{"name":"a","guid":"b"},"sequence":[)"
            << R"({"type":"setOrigin","id":"x","name":"n","plane":null,"position":null,)"
            << R"("xDirection":null,"options":{"flipZ":false,"xQuarterTurns":0,"offset":[0,0,0]}}]})";
    }
    const LoadResult old = LoadJob(file.Path());
    REQUIRE_MESSAGE(old.status == LoadStatus::Ok, old.message);
    CHECK(old.job.sequence[0].index.automaticRetract);
    CHECK(old.job.sequence[0].index.direction == IndexDirection::Shortest);
}

TEST_CASE("the 4th axis post retracts, turns, then cuts from the turned work zero")
{
    const PostScriptRead read = ReadPostScript(ReadPost("LinuxCNC 4th axis.post"));
    REQUIRE_MESSAGE(read.ok, read.problem);
    CHECK(read.script.axes == 4);
    CHECK(read.script.rotaryWord == "A");

    // The first operation along the work zero; the second under a Rotate of 90 about +Y.
    SequenceStep zero = WorkZero();
    SequenceStep rotate = RotateStep(90);
    const SequenceStep first = Operation("Top", { Move(MoveKind::Rapid, { 10, 0, 20 }) });
    const SequenceStep second = Operation("Side", { Move(MoveKind::Rapid, { 0, 0, 20 }),
                                                    Move(MoveKind::Feed, { 0, 0, 12 }, 100) });
    Frame turned;
    turned.zAxis = { 1, 0, 0 };
    turned.xAxis = { 0, 0, -1 };
    turned.yAxis = { 0, 1, 0 };
    std::vector<PostOperation> entries(2);
    entries[0].origin = &zero;
    entries[0].operation = &first;
    entries[1].origin = &rotate;
    entries[1].operation = &second;
    entries[1].frame = turned;

    const std::vector<Vec3> stock = BoxCorners({ { -10, -50, -10 }, { 10, 50, 10 } });
    OrientationContext context = FourAxes();
    context.stockPoints = &stock;
    const PostOutput out = WriteScriptedPost(read.script, "Bar", Frame(), entries, "", ArcFitting(), context);
    REQUIRE_MESSAGE(out.ok, out.problem);

    // The furthest from the axis is the first operation's rapid, sqrt(10^2 + 20^2), plus 5.
    CHECK(Has(out.text, "G0 Z27.3607\nG0 A90\n"));
    CHECK(Has(out.text, "(Side)\nS10000 M3\nG0 X0 Y0 Z20\nG1 Z12 F100\n"));
    CHECK(out.text.find("G0 A") == out.text.rfind("G0 A"));     // the first operation needs no turn

    // A set retract height, turning the negative way.
    rotate.index.automaticRetract = false;
    rotate.index.retractHeight = 60;
    rotate.index.direction = IndexDirection::Negative;
    const PostOutput negative = WriteScriptedPost(read.script, "Bar", Frame(), entries, "", ArcFitting(), context);
    REQUIRE(negative.ok);
    CHECK(Has(negative.text, "G0 Z60\nG0 A-270\n"));

    // A machine with no rotary axis can't cut the turned operation.
    const PostOutput noAxis = WriteScriptedPost(read.script, "Bar", Frame(), entries, "", ArcFitting(),
                                                OrientationContext());
    CHECK_FALSE(noAxis.ok);
    CHECK(Has(noAxis.problem, "Side"));

    // Nor can the 3 axis post.
    const PostScriptRead three = ReadPostScript(ReadPost("LinuxCNC.post"));
    REQUIRE(three.ok);
    const PostOutput threeOut = WriteScriptedPost(three.script, "Bar", Frame(), entries, "", ArcFitting(), context);
    CHECK_FALSE(threeOut.ok);
    CHECK(Has(threeOut.problem, "Side"));
}

TEST_CASE("4 axis post scripts are checked when they're read")
{
    CHECK_FALSE(ReadPostScript("[settings]\nname = A\naxes = 4\n").ok);                 // no [index]
    CHECK(ReadPostScript("[settings]\nname = A\naxes = 4\n[index]\nG0{ Z}\nG0{ A}\n").ok);
    CHECK_FALSE(ReadPostScript("[settings]\nname = A\naxes = 5\n").ok);
    CHECK_FALSE(ReadPostScript("[settings]\nname = A\naxes = 4\nrotary = D\n[index]\n").ok);
    const PostScriptRead b = ReadPostScript("[settings]\nname = A\naxes = 4\nrotary = B\n[index]\nG0{ B}\n");
    REQUIRE(b.ok);
    CHECK(b.script.rotaryWord == "B");
}

namespace
{
    // A bar 20 x 60 x 20 round the rotary axis (+Y through the origin), and two
    // operations: one along Z leaving the tool just above the bar, one turned 90.
    std::vector<SimOperation> TurnJob(double retract)
    {
        ToolProfile flat;
        flat.type = ToolType::FlatEndMill;
        flat.radius = 3;
        flat.length = 30;

        SimOperation top;
        top.name = "Top";
        top.profile = flat;
        top.toolpath.moves = { Move(MoveKind::Rapid, { 0, 0, 20 }), Move(MoveKind::Rapid, { 0, 0, 11 }) };

        SimOperation side = top;
        side.name = "Side";
        side.frame.zAxis = { 1, 0, 0 };
        side.frame.xAxis = { 0, 0, -1 };
        side.frame.yAxis = { 0, 1, 0 };
        side.toolpath.moves = { Move(MoveKind::Rapid, { 0, 0, 25 }) };
        side.index.present = true;
        side.index.axisDirection = { 0, 1, 0 };
        side.index.from = 0;
        side.index.to = 90;
        side.index.retract = retract;
        return { top, side };
    }
}

TEST_CASE("the simulation swings the tool round the rotary axis between operations")
{
    const StockBox bar{ { -10, -30, -10 }, { 10, 30, 10 } };
    for (const StockModel model : { StockModel::Voxels, StockModel::TriDexel })
    {
        CAPTURE(static_cast<int>(model));
        SimResolution resolution;
        resolution.cell = 0.5;
        resolution.model = model;

        // Retracting clear of the bar's corners (14.1 from the axis), it turns without touching.
        Simulation clear;
        clear.Start(bar, {}, TurnJob(20), resolution);
        size_t swing = 0;
        for (const SimMove& move : clear.Moves())
            swing += move.index ? 1 : 0;
        CHECK(swing == 1 + 45);         // up from 11 to 20, then 2 degrees a step
        clear.RunToEnd();
        CHECK_FALSE(clear.HasWarning());
        CHECK(Near(clear.Moves().back().to, { 25, 0, 0 }, 1e-9));
        // At the end of the swing the tool is on +X, along the turned frame's Z.
        size_t last = 0;
        for (size_t m = 0; m < clear.Moves().size(); ++m)
            last = clear.Moves()[m].index ? m : last;
        CHECK(Near(clear.Moves()[last].to, { 20, 0, 0 }, 1e-9));
        CHECK(Near(clear.Moves()[last].axis, { 1, 0, 0 }, 1e-9));

        // Staying at 11 above the axis, the tool hits the bar's corner on the way round.
        Simulation low;
        low.Start(bar, {}, TurnJob(11), resolution);
        low.RunToEnd();
        REQUIRE(low.HasWarning());
        CHECK(low.Warning() == SimWarning::RapidIntoStock);
        CHECK(low.Moves()[low.WarningMove()].index);
    }
}

TEST_CASE("an arc about a sideways axis is drawn in its own plane")
{
    // A circle on a side face, facing +X, as two half arcs: picked features are drawn
    // in the assembly, where such an arc isn't in XY.
    Path circle(2);
    circle[0].point = { 12, 5, 0 };
    circle[0].arc = true;
    circle[0].centre = { 12, 0, 0 };
    circle[0].axis = { 1, 0, 0 };
    circle[1].point = { 12, -5, 0 };
    circle[1].arc = true;
    circle[1].centre = { 12, 0, 0 };
    circle[1].axis = { 1, 0, 0 };
    const Polyline points = PathPoints(circle, true);
    REQUIRE(points.size() > 8);
    bool passedPlusZ = false;
    for (const Vec3& point : points)
    {
        CHECK(point.x == doctest::Approx(12));
        CHECK(Length(point - Vec3{ 12, 0, 0 }) == doctest::Approx(5));
        passedPlusZ = passedPlusZ || point.z > 4.9;
    }
    // Anticlockwise about +X from +Y goes through +Z first.
    CHECK(points[1].z > 0);
    CHECK(passedPlusZ);
    CHECK(Near(points.back(), { 12, 5, 0 }, 1e-9));
}

TEST_CASE("Generate's hint names features edge-on to the origin's Z")
{
    PickedGeometry side;
    side.kind = PickKind::CircleEdge;
    side.direction = { -1, 0, 0 };
    Frame down;
    down.zAxis = { 0, 0, -1 };
    down.yAxis = { 0, -1, 0 };
    CHECK_FALSE(EdgeOnFeaturesText({ side }, down).empty());
    Frame facingIt;
    facingIt.zAxis = { -1, 0, 0 };
    CHECK(EdgeOnFeaturesText({ side }, facingIt).empty());
}
