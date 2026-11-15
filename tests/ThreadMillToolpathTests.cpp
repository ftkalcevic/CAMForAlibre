// ThreadMillToolpathTests.cpp : tests for thread milling: settings, saving, toolpaths and the hole check.

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>

#include "CamJob.h"
#include "Simulation.h"
#include "ThreadMillToolpath.h"
#include "ToolLibrary.h"

using namespace camcore;
namespace fs = std::filesystem;

namespace
{
    const double kPi = 3.14159265358979323846;

    PickedGeometry Point(double x, double y, double z)
    {
        PickedGeometry pick;
        pick.kind = PickKind::CircleEdge;
        pick.point = { x, y, z };
        pick.direction = { 0, 0, 1 };
        pick.radius = 2.5;
        return pick;
    }

    // An M6 x 1 thread 10 long at the origin, its top at 0.
    SequenceStep ThreadStep()
    {
        SequenceStep step;
        step.type = StepType::ThreadMilling;
        step.id = NewStableId();
        step.name = "Thread Milling";
        step.clearance.mode = ClearanceMode::AbsoluteZ;
        step.clearance.absoluteZ = 10;
        step.features.push_back(Point(0, 0, 0));
        return step;
    }

    // A single point thread mill with 4 mm tips.
    Tool ThreadMill(int teeth = 1, double pitch = 1.0)
    {
        Tool tool;
        tool.id = "tm";
        tool.number = 1;
        tool.name = "Thread mill";
        tool.type = ToolType::ThreadMill;
        tool.diameter = 4;
        tool.fluteLength = 12;
        tool.overallLength = 50;
        tool.pitch = pitch;
        tool.threadAngle = 60;
        tool.threadTeeth = teeth;
        return tool;
    }

    ToolpathResult Generate(const SequenceStep& step, const Tool& tool)
    {
        ThreadMillToolpathInput input;
        input.operation = &step;
        input.tool = &tool;
        return GenerateThreadMillToolpath(input);
    }

    struct HelixArc
    {
        Vec3 from;
        ToolpathMove move;
    };

    // The arcs about the hole's centre at a radius: the helix, not the leads.
    std::vector<HelixArc> HelixArcs(const Toolpath& toolpath, double radius)
    {
        std::vector<HelixArc> arcs;
        Vec3 at;
        for (const ToolpathMove& move : toolpath.moves)
        {
            if (move.arc && std::hypot(move.centre.x, move.centre.y) < 1e-9 &&
                std::fabs(std::hypot(move.to.x, move.to.y) - radius) < 1e-9)
                arcs.push_back({ at, move });
            at = move.to;
        }
        return arcs;
    }

    class TempFile
    {
    public:
        explicit TempFile(const char* extension)
        {
            static std::atomic<int> counter{ 0 };
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            m_path = fs::temp_directory_path() /
                     ("camcore_thread_" + std::to_string(stamp) + "_" + std::to_string(counter++) + extension);
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

TEST_CASE("thread depth and minor diameter")
{
    CHECK(ThreadDepth(1.0, 60.0) == doctest::Approx(0.54127).epsilon(1e-4));
    ThreadMillingSettings settings;
    CHECK(ThreadMinorDiameter(settings, 60.0) == doctest::Approx(4.9175).epsilon(1e-4));
    CHECK(ValidateThreadMilling(settings).empty());

    ThreadMillingSettings bad;
    bad.length = 0;
    bad.majorDiameter = -1;
    bad.pitch = 0;
    bad.radialPasses = 0;
    bad.direction = CutDirection::Mixed;
    bad.approachDistance = -1;
    CHECK(ValidateThreadMilling(bad).size() == 6);
}

TEST_CASE("a thread mill's teeth are checked, saved and loaded")
{
    Tool tool = ThreadMill(5, 1.0);
    CHECK(ValidateTool(tool).empty());
    tool.threadTeeth = 0;
    CHECK(!ValidateTool(tool).empty());
    tool.threadTeeth = 13;      // 12 pitches between the outer teeth, in 12 of flute
    CHECK(!ValidateTool(tool).empty());

    ToolLibrary library;
    library.tools.push_back(ThreadMill(5, 1.25));
    library.tools.push_back(ThreadMill(1, 1.0));
    library.tools.back().id = "single";
    library.tools.back().number = 2;
    TempFile file(".json");
    REQUIRE(SaveToolLibrary(library, file.Path()).ok);
    const ToolLibraryLoadResult loaded = LoadToolLibrary(file.Path());
    REQUIRE(loaded.status == LoadStatus::Ok);
    REQUIRE(loaded.library.tools.size() == 2);
    CHECK(loaded.library.tools[0].threadTeeth == 5);
    CHECK(loaded.library.tools[1].threadTeeth == 1);
}

TEST_CASE("a thread milling step saves and loads")
{
    Job job;
    SequenceStep origin;
    origin.type = StepType::SetOrigin;
    origin.id = NewStableId();
    job.sequence.push_back(origin);
    SequenceStep folder;
    folder.type = StepType::Group;
    folder.id = NewStableId();
    folder.name = "Folder";
    SequenceStep step = ThreadStep();
    step.threadMilling.location = CutGeometryLocation::PickTop;
    step.threadMilling.topZ = 3;
    step.threadMilling.length = 12;
    step.threadMilling.majorDiameter = 8;
    step.threadMilling.pitch = 1.25;
    step.threadMilling.hand = ThreadHand::Left;
    step.threadMilling.direction = CutDirection::Conventional;
    step.threadMilling.radialPasses = 3;
    step.threadMilling.springPass = true;
    step.threadMilling.approachDistance = 2;
    folder.children.push_back(step);
    job.sequence.push_back(folder);

    TempFile file(".cam");
    REQUIRE(SaveJob(job, file.Path()).ok);
    const LoadResult loaded = LoadJob(file.Path());
    REQUIRE(loaded.status == LoadStatus::Ok);
    CHECK(loaded.fileVersion == 17);
    REQUIRE(loaded.job.sequence.size() == 2);
    REQUIRE(loaded.job.sequence[1].children.size() == 1);
    const SequenceStep& back = loaded.job.sequence[1].children[0];
    CHECK(back.type == StepType::ThreadMilling);
    CHECK(IsOperation(back.type));
    CHECK(SameSettings(back, step));
    CHECK(back.threadMilling.hand == ThreadHand::Left);
    CHECK(back.threadMilling.direction == CutDirection::Conventional);
    CHECK(back.threadMilling.radialPasses == 3);
    CHECK(back.threadMilling.springPass);
    CHECK(back.threadMilling.pitch == 1.25);
}

TEST_CASE("a right hand climb thread rises anticlockwise from the bottom, in from the centre and back")
{
    const SequenceStep step = ThreadStep();
    const Tool tool = ThreadMill();
    const ToolpathResult result = Generate(step, tool);
    REQUIRE(result.ok);
    CHECK(result.warnings.empty());

    const double radius = 3.0 - 2.0;     // the major radius less the tool's
    const std::vector<HelixArc> arcs = HelixArcs(result.toolpath, radius);
    REQUIRE(arcs.size() == 20);          // 10 turns, in half turns
    double total = 0.0;
    for (const HelixArc& arc : arcs)
    {
        CHECK(!arc.move.clockwise);
        CHECK(arc.move.to.z > arc.from.z);
        CHECK(arc.move.to.z - arc.from.z == doctest::Approx(0.5));
        total += arc.move.to.z - arc.from.z;
    }
    CHECK(total == doctest::Approx(10.0));
    CHECK(arcs.front().from.z == doctest::Approx(-10.0));
    CHECK(arcs.back().move.to.z == doctest::Approx(0.0));

    // The lead in starts at the centre a quarter pitch below and the lead out
    // ends there a quarter pitch above; it finishes at the clearance plane.
    const std::vector<ToolpathMove>& moves = result.toolpath.moves;
    for (size_t i = 0; i + 1 < moves.size(); ++i)
    {
        if (moves[i + 1].arc && !moves[i].arc)
        {
            CHECK(std::hypot(moves[i].to.x, moves[i].to.y) < 1e-9);
            CHECK(moves[i].to.z == doctest::Approx(-10.25));
            break;
        }
    }
    const ToolpathMove& out = moves[moves.size() - 2];
    CHECK(out.arc);
    CHECK(std::hypot(out.to.x, out.to.y) < 1e-9);
    CHECK(out.to.z == doctest::Approx(0.25));
    CHECK(moves.back().to.z == doctest::Approx(10.0));
}

TEST_CASE("conventional and left hand threads turn and go the other ways")
{
    const Tool tool = ThreadMill();
    const auto check = [&](ThreadHand hand, CutDirection direction, bool clockwise, bool up) {
        SequenceStep step = ThreadStep();
        step.threadMilling.hand = hand;
        step.threadMilling.direction = direction;
        const ToolpathResult result = Generate(step, tool);
        REQUIRE(result.ok);
        const std::vector<HelixArc> arcs = HelixArcs(result.toolpath, 1.0);
        REQUIRE(!arcs.empty());
        for (const HelixArc& arc : arcs)
        {
            CHECK(arc.move.clockwise == clockwise);
            CHECK((arc.move.to.z > arc.from.z) == up);
        }
    };
    check(ThreadHand::Right, CutDirection::Climb, false, true);
    check(ThreadHand::Right, CutDirection::Conventional, true, false);
    check(ThreadHand::Left, CutDirection::Climb, false, false);
    check(ThreadHand::Left, CutDirection::Conventional, true, true);
}

TEST_CASE("a multi-row thread mill cuts single turns whole pitches apart")
{
    SequenceStep step = ThreadStep();
    step.threadMilling.length = 12;
    const Tool tool = ThreadMill(5, 1.0);
    const ToolpathResult result = Generate(step, tool);
    REQUIRE(result.ok);
    const std::vector<HelixArc> arcs = HelixArcs(result.toolpath, 1.0);
    REQUIRE(arcs.size() == 6);   // 3 turns of two halves
    // Starting at the bottom, 5 up, then no higher than the top needs: 12 - 5 = 7.
    CHECK(arcs[0].from.z == doctest::Approx(-12.0));
    CHECK(arcs[2].from.z == doctest::Approx(-7.0));
    CHECK(arcs[4].from.z == doctest::Approx(-5.0));
    CHECK(arcs[5].move.to.z == doctest::Approx(-4.0));

    // A tool whose teeth are another pitch is still used, with a warning.
    const ToolpathResult other = Generate(step, ThreadMill(5, 1.5));
    REQUIRE(other.ok);
    CHECK(other.warnings.size() == 1);
}

TEST_CASE("radial passes step out to the major radius, and a spring pass repeats it")
{
    SequenceStep step = ThreadStep();
    step.threadMilling.radialPasses = 3;
    step.threadMilling.springPass = true;
    const Tool tool = ThreadMill();
    const ToolpathResult result = Generate(step, tool);
    REQUIRE(result.ok);
    const double first = ThreadMinorDiameter(step.threadMilling, 60.0) / 2.0 - 2.0;
    const double last = 1.0;
    for (int k = 1; k <= 3; ++k)
        CHECK(HelixArcs(result.toolpath, first + (last - first) * k / 3.0).size() == (k == 3 ? 40u : 20u));
}

TEST_CASE("thread milling refuses tools that can't cut the thread")
{
    const SequenceStep step = ThreadStep();
    Tool wide = ThreadMill();
    wide.diameter = 5;      // wider than the 4.92 minor diameter
    CHECK(!Generate(step, wide).ok);
    Tool mill = ThreadMill();
    mill.type = ToolType::FlatEndMill;
    CHECK(!Generate(step, mill).ok);
}

TEST_CASE("the hole check sees whether a hole is drilled open")
{
    // A 40 x 40 x 10 stock with a 5 mm hole drilled 8 deep at (20, 20).
    Tool drill;
    drill.type = ToolType::FlatEndMill;
    drill.diameter = 5;
    drill.fluteLength = 20;
    drill.overallLength = 40;
    Toolpath path;
    ToolpathMove move;
    move.kind = MoveKind::Rapid;
    move.to = { 20, 20, 15 };
    path.moves.push_back(move);
    move.kind = MoveKind::Feed;
    move.feed = 100;
    move.to = { 20, 20, 2 };
    path.moves.push_back(move);
    move.kind = MoveKind::Rapid;
    move.to = { 20, 20, 15 };
    path.moves.push_back(move);

    SimOperation op;
    op.name = "Drill";
    op.profile = ProfileOf(drill);
    op.toolpath = path;
    Simulation simulation;
    simulation.Start(StockBox{ { 0, 0, 0 }, { 40, 40, 10 } }, {}, { op }, SimResolution{ true, 0.1 });
    simulation.RunToEnd();
    REQUIRE(simulation.Finished());
    const RemainingStock remaining = RemainingStockHeights(simulation.Stock(), Frame());

    // Open to 4.9 down to 2, but not to 6, and not below 2.
    const double inside = 4.9 / 2.0 - 1.5 * remaining.cell;
    CHECK(remaining.HighestInCircle(20, 20, inside) < 2.0);
    CHECK(remaining.HighestInCircle(20, 20, 3.0) > 9.0);
    CHECK(remaining.HighestInCircle(5, 5, 1.0) > 9.0);
}

namespace
{
    // An M6 x 1 thread 6 long at (10, 10), its top at the stock's top (10), cut
    // after a 4.9 mm hole drilled 7 deep, in a 20 x 20 x 10 stock, simulated.
    struct ThreadedStock
    {
        Simulation simulation;
        bool dexels = false;

        bool Filled(double x, double y, double z) const
        {
            const double cell = simulation.Spacing();
            const Vec3 lower = dexels ? simulation.Dexels().Lower() : simulation.Stock().Lower();
            const int i = static_cast<int>(std::floor((x - lower.x) / cell));
            const int j = static_cast<int>(std::floor((y - lower.y) / cell));
            const int k = static_cast<int>(std::floor((z - lower.z) / cell));
            return dexels ? simulation.Dexels().Inside(i, j, k) : simulation.Stock().Filled(i, j, k);
        }

        // Along Z at a radius and angle round the hole, from 5 to 9: the filled samples, 0.01 apart.
        std::vector<bool> Column(double radius, double angle) const
        {
            std::vector<bool> filled;
            for (int n = 0; n <= 400; ++n)
                filled.push_back(Filled(10.0 + radius * std::cos(angle), 10.0 + radius * std::sin(angle), 5.0 + n * 0.01));
            return filled;
        }
    };

    void CutThread(ThreadedStock& stock, const Tool& threadMill, StockModel model, double cell)
    {
        Tool drill;
        drill.type = ToolType::FlatEndMill;
        drill.diameter = 4.9;
        drill.fluteLength = 20;
        drill.overallLength = 40;
        Toolpath hole;
        ToolpathMove move;
        move.kind = MoveKind::Rapid;
        move.to = { 10, 10, 15 };
        hole.moves.push_back(move);
        move.kind = MoveKind::Feed;
        move.feed = 100;
        move.to = { 10, 10, 3 };
        hole.moves.push_back(move);
        move.kind = MoveKind::Rapid;
        move.to = { 10, 10, 15 };
        hole.moves.push_back(move);
        SimOperation drilling;
        drilling.name = "Drill";
        drilling.profile = ProfileOf(drill);
        drilling.toolpath = hole;

        SequenceStep step = ThreadStep();
        step.clearance.absoluteZ = 20;
        step.features = { Point(10, 10, 10) };
        step.threadMilling.length = 6;
        const ToolpathResult result = Generate(step, threadMill);
        REQUIRE(result.ok);
        SimOperation threading;
        threading.name = "Thread";
        threading.profile = CuttingProfileOf(threadMill);
        threading.parts = CuttingPartsOf(threadMill);
        threading.toolpath = result.toolpath;

        stock.dexels = model == StockModel::TriDexel;
        stock.simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { drilling, threading },
                               SimResolution{ true, cell, model });
        while (!stock.simulation.Finished())
        {
            const size_t moved = stock.simulation.RunToEnd();
            stock.simulation.ClearWarning();
            if (moved == 0 && !stock.simulation.Finished())
                break;
        }
        REQUIRE(stock.simulation.Finished());
    }

    // The runs of filled samples: how many start, and where the first starts.
    size_t Runs(const std::vector<bool>& filled, size_t& first)
    {
        size_t runs = 0;
        first = filled.size();
        for (size_t n = 1; n < filled.size(); ++n)
        {
            if (filled[n] && !filled[n - 1])
            {
                if (runs == 0)
                    first = n;
                ++runs;
            }
        }
        return runs;
    }

    double Fraction(const std::vector<bool>& filled)
    {
        size_t count = 0;
        for (bool f : filled)
            count += f ? 1 : 0;
        return static_cast<double>(count) / filled.size();
    }

    void CheckThread(const ThreadedStock& stock)
    {
        const double minor = 4.9175, major = 6.0;
        const double middle = (minor + major) / 4.0;

        // Halfway into the thread: a band of metal a pitch apart, a little over half of it.
        const std::vector<bool> at0 = stock.Column(middle, 0.0);
        size_t first0 = 0;
        const size_t runs = Runs(at0, first0);
        CHECK(runs >= 3);
        CHECK(runs <= 5);
        CHECK(Fraction(at0) > 0.4);
        CHECK(Fraction(at0) < 0.75);

        // A quarter turn on (anticlockwise), a right-hand thread is a quarter pitch higher.
        const std::vector<bool> at90 = stock.Column(middle, 3.14159265358979323846 / 2.0);
        size_t first90 = 0;
        Runs(at90, first90);
        const double shift = std::fmod((static_cast<double>(first90) - static_cast<double>(first0)) * 0.01 + 10.0, 1.0);
        CHECK(std::fabs(shift - 0.25) < 0.08);

        // Inside the minor diameter it's open; outside the major, solid.
        CHECK(Fraction(stock.Column(minor / 2.0 - 0.15, 0.7)) < 0.01);
        CHECK(Fraction(stock.Column(major / 2.0 + 0.15, 0.7)) > 0.99);
    }
}

TEST_CASE("a thread mill's teeth as the simulation cuts with them")
{
    const Tool tool = ThreadMill();
    const std::vector<ToolPart> parts = CuttingPartsOf(tool);
    REQUIRE(parts.size() == 3);
    const double depth = ThreadDepth(1.0, 60.0);
    CHECK(parts[0].profile.type == ToolType::FlatEndMill);
    CHECK(parts[0].profile.radius == doctest::Approx(2.0 - depth));
    CHECK(parts[0].offset == 0.0);
    const double slope = 1.0 / std::tan(30.0 * 3.14159265358979323846 / 180.0);
    const double length = depth / slope + 1.0 / 16.0;
    CHECK(parts[1].profile.type == ToolType::VMill);
    CHECK(parts[1].profile.slope == doctest::Approx(slope));
    CHECK(parts[1].profile.tipRadius == doctest::Approx(2.0 - depth));
    CHECK(parts[1].profile.radius == 2.0);
    CHECK(parts[1].profile.length == doctest::Approx(length));
    CHECK(parts[1].offset == doctest::Approx(-length));
    CHECK_FALSE(parts[1].reversed);
    CHECK(parts[2].offset == doctest::Approx(length));
    CHECK(parts[2].reversed);
    CHECK(CuttingPartsOf(ThreadMill(5)).size() == 11);

    // Any other tool is one part, as before.
    Tool mill;
    mill.diameter = 6;
    mill.fluteLength = 20;
    mill.overallLength = 50;
    const std::vector<ToolPart> one = CuttingPartsOf(mill);
    REQUIRE(one.size() == 1);
    CHECK(one[0].profile.radius == 3.0);
    CHECK(one[0].profile.length == 50.0);
    CHECK(one[0].offset == 0.0);
}

TEST_CASE("simulating thread milling leaves a thread, in voxels")
{
    ThreadedStock stock;
    CutThread(stock, ThreadMill(), StockModel::Voxels, 0.05);
    CheckThread(stock);
}

TEST_CASE("simulating thread milling leaves a thread, in tri-dexels")
{
    ThreadedStock stock;
    CutThread(stock, ThreadMill(), StockModel::TriDexel, 0.05);
    CheckThread(stock);
}

TEST_CASE("a multi-row thread mill leaves the same thread as a single point one")
{
    ThreadedStock single, multi;
    CutThread(single, ThreadMill(), StockModel::Voxels, 0.05);
    CutThread(multi, ThreadMill(5), StockModel::Voxels, 0.05);
    CheckThread(multi);
    size_t same = 0, total = 0;
    for (double angle : { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 })
    {
        const std::vector<bool> a = single.Column(2.73, angle), b = multi.Column(2.73, angle);
        for (size_t n = 0; n < a.size(); ++n)
        {
            same += (a[n] == b[n]) ? 1 : 0;
            ++total;
        }
    }
    CHECK(static_cast<double>(same) / total > 0.95);
}

TEST_CASE("thread mill simulation timing" * doctest::skip())
{
    const std::pair<StockModel, double> runs[] = { { StockModel::Voxels, 0.05 }, { StockModel::TriDexel, 0.05 },
                                                   { StockModel::TriDexel, 0.1 } };
    for (const auto& run : runs)
    {
        for (int teeth : { 1, 5 })
        {
            const auto start = std::chrono::steady_clock::now();
            ThreadedStock stock;
            CutThread(stock, ThreadMill(teeth), run.first, run.second);
            const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            const std::string model = (run.first == StockModel::Voxels) ? "voxels " : "tri-dexel ";
            MESSAGE(model << run.second << " mm, " << teeth << " teeth: " << seconds << " s");
        }
    }
}
