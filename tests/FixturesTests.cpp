// FixturesTests.cpp : tests for fixtures (work holding): saving, checks, trimming, the simulation and 3D operations.

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>

#include "CamJob.h"
#include "Clearance.h"
#include "FixtureCheck.h"
#include "Simulation.h"
#include "Stock.h"
#include "WaterlineRoughToolpath.h"

using namespace camcore;
namespace fs = std::filesystem;

namespace
{
    // The 12 triangles of a box, facing out.
    std::vector<Vec3> BoxMesh(const Vec3& a, const Vec3& b)
    {
        const Vec3 c[8] = {
            { a.x, a.y, a.z }, { b.x, a.y, a.z }, { b.x, b.y, a.z }, { a.x, b.y, a.z },
            { a.x, a.y, b.z }, { b.x, a.y, b.z }, { b.x, b.y, b.z }, { a.x, b.y, b.z },
        };
        const int faces[6][4] = { { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
                                  { 1, 2, 6, 5 }, { 2, 3, 7, 6 }, { 3, 0, 4, 7 } };
        std::vector<Vec3> mesh;
        for (const auto& f : faces)
        {
            for (int i : { f[0], f[1], f[2], f[0], f[2], f[3] })
                mesh.push_back(c[i]);
        }
        return mesh;
    }

    FixtureMesh Clamp(const Vec3& a, const Vec3& b, double clearance, const std::string& name = "Clamp<1>")
    {
        FixtureMesh mesh;
        mesh.name = name;
        mesh.triangles = BoxMesh(a, b);
        mesh.clearance = clearance;
        return mesh;
    }

    ToolpathMove Move(MoveKind kind, const Vec3& to, double feed = 0.0)
    {
        ToolpathMove move;
        move.kind = kind;
        move.to = to;
        move.feed = feed;
        return move;
    }

    // A straight pass along X at height z and y, from x0 to x1, from above at 30.
    Toolpath Pass(double y, double z, double x0, double x1)
    {
        Toolpath path;
        path.moves.push_back(Move(MoveKind::Rapid, { x0, y, 30 }));
        path.moves.push_back(Move(MoveKind::Feed, { x0, y, z }, 100));
        path.moves.push_back(Move(MoveKind::Feed, { x1, y, z }, 500));
        path.moves.push_back(Move(MoveKind::Rapid, { x1, y, 30 }));
        return path;
    }

    FixtureTool SixMm(double length = 30.0)
    {
        FixtureTool tool;
        tool.radius = 3.0;
        tool.length = length;
        return tool;
    }

    class TempFile
    {
    public:
        TempFile()
        {
            static std::atomic<int> counter{ 0 };
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            m_path = fs::temp_directory_path() /
                     ("camcore_fixtures_" + std::to_string(stamp) + "_" + std::to_string(counter++) + ".cam");
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

TEST_CASE("a fixtures step saves and loads, and operations below it use it")
{
    Job job;
    SequenceStep origin;
    origin.type = StepType::SetOrigin;
    origin.id = NewStableId();
    origin.name = "Set Origin";
    job.sequence.push_back(origin);

    SequenceStep fixtures;
    fixtures.type = StepType::Fixtures;
    fixtures.id = NewStableId();
    fixtures.name = "Fixtures";
    fixtures.fixtures.trim = true;
    fixtures.fixtures.defaultClearance = 3.0;
    FixturePart part;
    part.occurrenceKey.elementType = 17;
    part.occurrenceKey.hex = "0a0b0c";
    part.name = "Vice<1>";
    part.clearance = 1.5;
    fixtures.fixtures.parts.push_back(part);
    job.sequence.push_back(fixtures);

    SequenceStep folder;
    folder.type = StepType::Group;
    folder.id = NewStableId();
    folder.name = "Folder";
    job.sequence.push_back(folder);

    SequenceStep moved = fixtures;
    moved.id = NewStableId();
    moved.name = "Clamps moved";
    moved.suppressed = true;
    job.sequence.push_back(moved);
    job.sequence.push_back(folder);
    job.sequence.back().id = NewStableId();

    TempFile file;
    REQUIRE(SaveJob(job, file.Path()).ok);
    const LoadResult loaded = LoadJob(file.Path());
    REQUIRE(loaded.status == LoadStatus::Ok);
    REQUIRE(loaded.job.sequence.size() == 5);
    const SequenceStep& back = loaded.job.sequence[1];
    CHECK(back.type == StepType::Fixtures);
    CHECK(back.fixtures.trim);
    CHECK(back.fixtures.defaultClearance == 3.0);
    REQUIRE(back.fixtures.parts.size() == 1);
    CHECK(back.fixtures.parts[0].name == "Vice<1>");
    CHECK(back.fixtures.parts[0].occurrenceKey.hex == "0a0b0c");
    CHECK(back.fixtures.parts[0].occurrenceKey.elementType == 17);
    CHECK(back.fixtures.parts[0].clearance == 1.5);
    CHECK(SameSettings(back, fixtures));
    CHECK(loaded.job.sequence[3].suppressed);

    // The folder under the first step uses it; the second is suppressed, so the
    // last folder still uses the first; the step itself is its own.
    CHECK(ActiveFixturesIndex(loaded.job, 0) == kNoStepIndex);
    CHECK(ActiveFixturesIndex(loaded.job, 2) == 1);
    CHECK(ActiveFixturesIndex(loaded.job, 3) == 3);
    CHECK(ActiveFixturesIndex(loaded.job, 4) == 1);
    Job unsuppressed = loaded.job;
    unsuppressed.sequence[3].suppressed = false;
    CHECK(ActiveFixturesIndex(unsuppressed, 4) == 3);

    // Templates keep it.
    const Job copy = JobAsTemplate(job);
    CHECK(copy.sequence[1].fixtures.parts.size() == 1);

    // A negative clearance is refused.
    FixtureSettings bad = fixtures.fixtures;
    bad.parts[0].clearance = -1.0;
    CHECK_FALSE(ValidateFixtures(bad).empty());
    CHECK(ValidateFixtures(fixtures.fixtures).empty());
}

TEST_CASE("a toolpath through a clamp is found, and one outside its clearance isn't")
{
    const std::vector<FixtureGrid> fixtures{ MakeFixtureGrid(Clamp({ 40, 0, 0 }, { 60, 20, 15 }, 2.0)) };
    const FixtureTool tool = SixMm();

    // Straight through it.
    const std::vector<FixtureHit> through = CheckFixtures(Pass(10, 5, 0, 100), Frame(), tool, fixtures);
    REQUIRE(through.size() == 1);
    CHECK(through[0].move == 2);
    CHECK_FALSE(through[0].holder);
    CHECK(FixtureHitText(through[0], fixtures).find("Clamp<1>") != std::string::npos);

    // Beside it: a 3 mm radius tool with 2 mm clearance must stay 5 mm from its side (at y = 20).
    CHECK(CheckFixtures(Pass(24.5, 5, 0, 100), Frame(), tool, fixtures).size() == 1);
    CHECK(CheckFixtures(Pass(25.6, 5, 0, 100), Frame(), tool, fixtures).empty());
    CHECK(CheckFixtures(Pass(30, 5, 0, 100), Frame(), tool, fixtures).empty());

    // Over the top by more than the clearance.
    CHECK(CheckFixtures(Pass(10, 17.6, 0, 100), Frame(), tool, fixtures).empty());
    CHECK(CheckFixtures(Pass(10, 16.0, 0, 100), Frame(), tool, fixtures).size() == 1);
}

TEST_CASE("the tool holder hitting a tall clamp is found")
{
    // A short tool cutting beside a tall clamp: the cutter clears it, the 20 mm holder doesn't.
    const std::vector<FixtureGrid> fixtures{ MakeFixtureGrid(Clamp({ 40, 0, 0 }, { 60, 20, 40 }, 1.0)) };
    FixtureTool tool = SixMm(15.0);
    tool.holder.start = 15.0;
    tool.holder.radius = 10.0;
    tool.holder.length = 30.0;
    const std::vector<FixtureHit> hits = CheckFixtures(Pass(28, 5, 0, 100), Frame(), tool, fixtures);
    REQUIRE(hits.size() == 1);
    CHECK(hits[0].holder);
}

TEST_CASE("trimming lifts over a clamp, and the result is clear")
{
    const std::vector<FixtureGrid> fixtures{ MakeFixtureGrid(Clamp({ 40, 0, 0 }, { 60, 20, 15 }, 2.0)) };
    const FixtureTool tool = SixMm();
    Toolpath path = Pass(10, 5, 0, 100);
    int trimmed = 0;
    std::string problem;
    REQUIRE(TrimAroundFixtures(path, Frame(), tool, fixtures, 25.0, 100.0, trimmed, problem));
    CHECK(trimmed == 1);
    CHECK(CheckFixtures(path, Frame(), tool, fixtures).empty());

    // It still cuts up to the clamp from both sides, at the cut's height.
    double lowestBefore = 1e9, highestAfter = -1e9;
    for (const ToolpathMove& move : path.moves)
    {
        if (move.kind == MoveKind::Feed && std::fabs(move.to.z - 5.0) < 1e-9)
        {
            if (move.to.x < 50)
                highestAfter = (std::max)(highestAfter, move.to.x);
            else
                lowestBefore = (std::min)(lowestBefore, move.to.x);
        }
    }
    CHECK(highestAfter > 34.0);
    CHECK(highestAfter < 35.5);
    CHECK(lowestBefore > 64.5);
    CHECK(lowestBefore < 66.0);
    // The lift reaches the clearance plane.
    bool reached = false;
    for (const ToolpathMove& move : path.moves)
        reached = reached || (move.kind == MoveKind::Rapid && std::fabs(move.to.z - 25.0) < 1e-9);
    CHECK(reached);

    // A clear toolpath is left alone.
    Toolpath clear = Pass(40, 5, 0, 100);
    const size_t before = clear.moves.size();
    REQUIRE(TrimAroundFixtures(clear, Frame(), tool, fixtures, 25.0, 100.0, trimmed, problem));
    CHECK(trimmed == 0);
    CHECK(clear.moves.size() == before);

    // With the clearance plane below the clamp's top, it can't lift over.
    Toolpath low = Pass(10, 5, 0, 100);
    CHECK_FALSE(TrimAroundFixtures(low, Frame(), tool, fixtures, 12.0, 100.0, trimmed, problem));
    CHECK(problem.find("too low") != std::string::npos);
    CHECK(low.moves.size() == 4);
}

TEST_CASE("the simulation stops where the tool or holder hits a fixture, with voxels and tri-dexels")
{
    // The clamp sits outside the stock's box.
    const FixtureMesh clamp = Clamp({ 25, 0, 0 }, { 35, 20, 20 }, 0.0, "Clamp<2>");
    auto grids = MakeFixtureGrids({ clamp });

    Tool tool;
    tool.diameter = 4.0;
    tool.fluteLength = 10.0;
    tool.overallLength = 30.0;
    SimOperation operation;
    operation.name = "Op";
    operation.profile = ProfileOf(tool);
    operation.holder = HolderOf(tool, 20.0, 30.0);
    operation.fixtures = grids;

    for (StockModel model : { StockModel::Voxels, StockModel::TriDexel })
    {
        // Into the clamp's side.
        operation.toolpath = Pass(10, 5, 0, 40);
        Simulation simulation;
        simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { operation }, SimResolution{ false, 0.5, model });
        simulation.RunToEnd();
        CHECK(simulation.Warning() == SimWarning::ToolHitsFixture);
        CHECK(simulation.WarningFixture() == "Clamp<2>");

        // Ignored when asked.
        simulation.SetIgnoreCollisions(true);
        simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { operation }, SimResolution{ false, 0.5, model });
        simulation.RunToEnd();
        CHECK_FALSE(simulation.HasWarning());
        simulation.SetIgnoreCollisions(false);

        // Beside it, 8 mm off: the 4 mm tool clears it, the 20 mm holder above 30 doesn't when the clamp's taller.
        SimOperation tall = operation;
        tall.fixtures = MakeFixtureGrids({ Clamp({ 25, 0, 0 }, { 35, 20, 60 }, 0.0, "Tall<1>") });
        tall.toolpath = Pass(10, 5, 0, 18);
        simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { tall }, SimResolution{ false, 0.5, model });
        simulation.RunToEnd();
        CHECK(simulation.Warning() == SimWarning::HolderHitsFixture);
        CHECK(simulation.WarningFixture() == "Tall<1>");

        // An operation with no fixtures (under no Fixtures step) runs through.
        SimOperation none = operation;
        none.fixtures.reset();
        simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { none }, SimResolution{ false, 0.5, model });
        simulation.RunToEnd();
        CHECK_FALSE(simulation.HasWarning());
    }
}

TEST_CASE("automatic clearance goes above the fixtures")
{
    ClearanceSettings settings;
    double height = 0.0;
    std::string problem;
    REQUIRE(ClearanceHeight(settings, true, 10.0, height, problem, true, 12.0, true, 32.0));
    CHECK(height == doctest::Approx(37.0));
    REQUIRE(ClearanceHeight(settings, true, 10.0, height, problem, true, 12.0, false, 32.0));
    CHECK(height == doctest::Approx(17.0));

    double top = 0.0;
    REQUIRE(FixtureTopAlongZ(Frame(), { Clamp({ 0, 0, 0 }, { 10, 10, 30 }, 2.0) }, top));
    CHECK(top == doctest::Approx(32.0));
    CHECK_FALSE(FixtureTopAlongZ(Frame(), {}, top));
}

namespace
{
    // A 60 x 60 x 20 stock with its footprint the area, a 20 mm block in its
    // middle, and a clamp at its left edge.
    WaterlineRoughToolpathInput BlockInput(const SequenceStep& step, const Tool& tool)
    {
        WaterlineRoughToolpathInput input;
        input.operation = &step;
        input.tool = &tool;
        ProfileChain footprint;
        footprint.ok = true;
        footprint.closed = true;
        footprint.path = StockFootprint(Frame(), { { 0, 0, 0 }, { 60, 60, 20 } });
        input.outlines = { footprint };
        input.partTriangles = BoxMesh({ 20, 20, 0 }, { 40, 40, 10 });
        input.havePartMaxZ = true;
        input.partMaxZ = 10;
        input.havePartMinZ = true;
        input.partMinZ = 0;
        input.haveStockMaxZ = true;
        input.stockMaxZ = 20;
        input.haveStockMinZ = true;
        input.stockMinZ = 0;
        input.fixtures = { Clamp({ 0, 22, 0 }, { 8, 38, 12 }, 2.0) };
        return input;
    }
}

TEST_CASE("waterline roughing and parallel finishing keep clear of a fixture when trimming")
{
    const std::vector<FixtureGrid> check{ MakeFixtureGrid(Clamp({ 0, 22, 0 }, { 8, 38, 12 }, 1.5)) };

    Tool flat;
    flat.diameter = 6;
    flat.overallLength = 40;
    SequenceStep rough;
    rough.type = StepType::WaterlineRoughing;
    rough.clearance.mode = ClearanceMode::AbsoluteZ;
    rough.clearance.absoluteZ = 30;
    rough.feeds.plunge = 150;
    rough.feeds.cut = 1200;
    rough.waterlineRoughing.stepDown = 3.0;
    rough.waterlineRoughing.clearFlats = true;
    const ToolpathResult roughed = GenerateWaterlineRoughToolpath(BlockInput(rough, flat));
    REQUIRE(roughed.ok);
    CHECK(CheckFixtures(roughed.toolpath, Frame(), FixtureToolOf(flat, 0.0, 0.0), check).empty());
    // The clamp's top (12) isn't taken as a flat to clear.
    for (const ToolpathMove& move : roughed.toolpath.moves)
    {
        if (move.kind == MoveKind::Feed)
            CHECK(std::fabs(move.to.z - 12.0) > 1e-6);
    }
    // Without the fixture, it cuts into the clamp's space.
    WaterlineRoughToolpathInput without = BlockInput(rough, flat);
    without.fixtures.clear();
    const ToolpathResult plain = GenerateWaterlineRoughToolpath(without);
    REQUIRE(plain.ok);
    CHECK_FALSE(CheckFixtures(plain.toolpath, Frame(), FixtureToolOf(flat, 0.0, 0.0), check).empty());

    Tool ball;
    ball.type = ToolType::BallMill;
    ball.diameter = 6;
    ball.overallLength = 40;
    SequenceStep finish;
    finish.type = StepType::ParallelFinishing;
    finish.parallelFinishing = ParallelFinishingDefaults();
    finish.clearance.mode = ClearanceMode::AbsoluteZ;
    finish.clearance.absoluteZ = 30;
    finish.feeds.plunge = 150;
    finish.feeds.cut = 1200;
    finish.parallelFinishing.stepoverMode = WaterlineStepoverMode::Distance;
    finish.parallelFinishing.stepoverDistance = 1.0;
    const ToolpathResult finished = GenerateParallelFinishToolpath(BlockInput(finish, ball));
    REQUIRE(finished.ok);
    const std::vector<FixtureHit> hits = CheckFixtures(finished.toolpath, Frame(), FixtureToolOf(ball, 0.0, 0.0), check);
    for (const FixtureHit& hit : hits)
    {
        for (size_t m = (hit.move > 3 ? hit.move - 3 : 0); m <= hit.move + 1 && m < finished.toolpath.moves.size(); ++m)
        {
            const ToolpathMove& move = finished.toolpath.moves[m];
            MESSAGE(m << " kind " << int(move.kind) << " " << move.to.x << " " << move.to.y << " " << move.to.z);
        }
    }
    CHECK(hits.empty());
}
