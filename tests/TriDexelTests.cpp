// TriDexelTests.cpp : tests for the tri-dexel stock: filling, exact cuts, meshing and simulating with it.

#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <map>
#include <tuple>

#include "Simulation.h"
#include "TriDexel.h"

using namespace camcore;

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    const Vec3 kUp{ 0, 0, 1 };

    TriDexelStock DexelBlock(double spacing = 0.25)
    {
        TriDexelStock stock;
        stock.Reset({ { 0, 0, 0 }, { 20, 20, 10 } }, spacing);
        stock.FillAll();
        return stock;
    }

    ToolProfile FlatTool(double diameter)
    {
        Tool tool;
        tool.diameter = diameter;
        tool.fluteLength = 20;
        tool.overallLength = 50;
        return ProfileOf(tool);
    }

    ToolProfile BallTool(double diameter)
    {
        Tool tool;
        tool.type = ToolType::BallMill;
        tool.diameter = diameter;
        tool.fluteLength = 20;
        tool.overallLength = 50;
        return ProfileOf(tool);
    }

    // A UV sphere's triangles, facing out.
    std::vector<Vec3> SphereMesh(const Vec3& centre, double radius, int around, int up)
    {
        const auto point = [&](int a, int b) {
            const double theta = 2.0 * kPi * a / around, phi = kPi * b / up;
            return Vec3{ centre.x + radius * std::sin(phi) * std::cos(theta),
                         centre.y + radius * std::sin(phi) * std::sin(theta), centre.z + radius * std::cos(phi) };
        };
        std::vector<Vec3> mesh;
        for (int b = 0; b < up; ++b)
        {
            for (int a = 0; a < around; ++a)
            {
                const Vec3 p = point(a, b), q = point(a + 1, b), r = point(a + 1, b + 1), s = point(a, b + 1);
                if (b > 0)
                    mesh.insert(mesh.end(), { p, s, q });
                if (b < up - 1)
                    mesh.insert(mesh.end(), { q, s, r });
            }
        }
        return mesh;
    }

    double MeshVolume(const std::vector<Vec3>& triangles)
    {
        double volume = 0.0;
        for (size_t t = 0; t + 2 < triangles.size(); t += 3)
            volume += Dot(triangles[t], Cross(triangles[t + 1], triangles[t + 2])) / 6.0;
        return volume;
    }

    double MeshVolume(const StockMesh& mesh)
    {
        double volume = 0.0;
        const auto at = [&](int v) {
            const size_t i = static_cast<size_t>(v) * 3;
            return Vec3{ mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2] };
        };
        for (size_t f = 0; f + 3 < mesh.faces.size(); f += 4)
            volume += Dot(at(mesh.faces[f + 1]), Cross(at(mesh.faces[f + 2]), at(mesh.faces[f + 3]))) / 6.0;
        return volume;
    }

    // Each directed edge has its reverse: the mesh is closed and consistently facing.
    bool Closed(const StockMesh& mesh)
    {
        std::map<std::tuple<float, float, float, float, float, float>, int> edges;
        const auto key = [&](int a, int b) {
            const size_t i = static_cast<size_t>(a) * 3, j = static_cast<size_t>(b) * 3;
            return std::make_tuple(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2], mesh.vertices[j],
                                   mesh.vertices[j + 1], mesh.vertices[j + 2]);
        };
        for (size_t f = 0; f + 3 < mesh.faces.size(); f += 4)
        {
            for (int e = 0; e < 3; ++e)
            {
                const int a = mesh.faces[f + 1 + e], b = mesh.faces[f + 1 + (e + 1) % 3];
                ++edges[key(a, b)];
                --edges[key(b, a)];
            }
        }
        for (const auto& edge : edges)
        {
            if (edge.second != 0)
                return false;
        }
        return true;
    }

    // The ray's last stretch's end (its top, for a Z ray).
    double TopOf(const TriDexelStock& stock, int axis, int iu, int iv)
    {
        std::vector<TriDexelStock::Interval> intervals;
        stock.Intervals(axis, stock.RayIndex(axis, iu, iv), intervals);
        return intervals.empty() ? -1.0 : intervals.back().exit;
    }

    // The tool's lowest point over (x, y) as its tip moves from a to b (along Z), by brute force.
    double BruteLowest(const ToolProfile& profile, const Vec3& a, const Vec3& b, double x, double y)
    {
        double lowest = 1e300;
        for (int n = 0; n <= 200000; ++n)
        {
            const Vec3 c = a + (b - a) * (n / 200000.0);
            const double h = profile.HeightForRadius(std::hypot(x - c.x, y - c.y));
            if (h >= 0.0)
                lowest = (std::min)(lowest, c.z + h);
        }
        return lowest;
    }

    // Where the swept tool covers the line along X at (y, z), by brute force.
    bool BruteAcross(const ToolProfile& profile, const Vec3& a, const Vec3& b, double y, double z, double& x0, double& x1)
    {
        x0 = 1e300;
        x1 = -1e300;
        for (int n = 0; n <= 200000; ++n)
        {
            const Vec3 c = a + (b - a) * (n / 200000.0);
            const double rho = profile.RadiusAt(z - c.z);
            const double dy = y - c.y;
            if (rho < 0.0 || std::fabs(dy) > rho)
                continue;
            const double half = std::sqrt(rho * rho - dy * dy);
            x0 = (std::min)(x0, c.x - half);
            x1 = (std::max)(x1, c.x + half);
        }
        return x0 <= x1;
    }

    // The stretch of a ray covering from a to b's middle, or none.
    bool StretchAround(const TriDexelStock& stock, int axis, int iu, int iv, double t, double& gapStart, double& gapEnd)
    {
        // The gap the cut left around t: the end of the stretch before and the start of the one after.
        std::vector<TriDexelStock::Interval> intervals;
        stock.Intervals(axis, stock.RayIndex(axis, iu, iv), intervals);
        gapStart = 0.0;
        gapEnd = stock.Length(axis);
        for (const TriDexelStock::Interval& interval : intervals)
        {
            if (interval.exit <= t)
                gapStart = interval.exit;
            else if (interval.enter >= t)
            {
                gapEnd = interval.enter;
                break;
            }
            else
                return false;   // t is still in stock
        }
        return true;
    }

    SimOperation DexelOperation(const ToolProfile& profile, std::vector<ToolpathMove> moves)
    {
        SimOperation operation;
        operation.name = "Op";
        operation.profile = profile;
        operation.toolpath.moves = std::move(moves);
        return operation;
    }

    ToolpathMove DexelMove(MoveKind kind, Vec3 to)
    {
        ToolpathMove move;
        move.kind = kind;
        move.to = to;
        move.feed = 100;
        return move;
    }

    const SimResolution kDexels{ false, 0.25, StockModel::TriDexel };
}

TEST_CASE("tri-dexel stops keep the largest grid within its rays, fine to coarse")
{
    const StockBox box{ { 0, 0, 0 }, { 136, 150, 6 } };
    const std::vector<double> stops = DexelResolutionStops(box);
    REQUIRE(!stops.empty());
    CHECK(stops.size() <= 7);
    CHECK(stops.front() == doctest::Approx(0.1));
    for (size_t n = 1; n < stops.size(); ++n)
        CHECK(stops[n] > stops[n - 1]);
}

TEST_CASE("a full tri-dexel box has the box's volume, and a sphere mesh the sphere's")
{
    const TriDexelStock block = DexelBlock(0.5);
    CHECK(block.Volume() == doctest::Approx(4000.0));
    CHECK(block.Inside(0, 0, 0));
    CHECK(block.Inside(39, 39, 19));
    CHECK_FALSE(block.Inside(40, 0, 0));

    TriDexelStock sphere;
    sphere.Reset({ { 0, 0, 0 }, { 20, 20, 20 } }, 0.1);
    const std::vector<Vec3> mesh = SphereMesh({ 10, 10, 10 }, 8.0, 64, 32);
    sphere.FillMesh(mesh);
    CHECK(sphere.Volume() == doctest::Approx(MeshVolume(mesh)).epsilon(0.005));
    // Every ray's ends are on the sphere's facets, within their sag.
    std::vector<TriDexelStock::Interval> intervals;
    sphere.Intervals(0, sphere.RayIndex(0, 100, 100), intervals);    // along X through (10.05, 10.05)
    REQUIRE(intervals.size() == 1);
    CHECK(intervals[0].enter == doctest::Approx(2.0).epsilon(0.002));
    CHECK(intervals[0].exit == doctest::Approx(18.0).epsilon(0.002));
}

TEST_CASE("a level flat end mill slot cuts every ray exactly")
{
    TriDexelStock stock = DexelBlock(0.1);
    const ToolProfile flat = FlatTool(4);
    const double removed = CutDexels(stock, flat, { 5, 10, 8 }, { 15, 10, 8 }, kUp);
    CHECK(removed > 0.0);

    // Z rays: down to the slot's floor, or untouched.
    CHECK(TopOf(stock, 2, 100, 100) == doctest::Approx(8.0).epsilon(1e-5));    // (10.05, 10.05)
    CHECK(TopOf(stock, 2, 100, 125) == doctest::Approx(10.0));                 // (10.05, 12.55): outside the slot
    CHECK(TopOf(stock, 2, 152, 100) == doctest::Approx(8.0).epsilon(1e-5));    // (15.25, 10.05): in the end's round

    // X rays at heights in the slot: a gap of the slot's length plus the tool's round ends.
    for (int j : { 100, 110, 115 })
    {
        const double y = (j + 0.5) * 0.1;
        const double half = std::sqrt(4.0 - (y - 10.0) * (y - 10.0));
        double gapStart = 0.0, gapEnd = 0.0;
        REQUIRE(StretchAround(stock, 0, j, 85, 10.0, gapStart, gapEnd));     // z = 8.55
        CHECK(gapStart == doctest::Approx(5.0 - half).epsilon(1e-5));
        CHECK(gapEnd == doctest::Approx(15.0 + half).epsilon(1e-5));
    }

    // The volume: the slot is a 4 x 10 rectangle and a 4 mm circle, 2 deep, sampled at the rays.
    const double slot = (40.0 + kPi * 4.0) * 2.0;
    CHECK(4000.0 - stock.Volume() == doctest::Approx(slot).epsilon(0.01));
}

TEST_CASE("a sloping ball mill move cuts rays along and across the axis as its whole sweep does")
{
    TriDexelStock stock = DexelBlock(0.1);
    const ToolProfile ball = BallTool(4);
    const Vec3 a{ 5, 10, 9 }, b{ 15, 10, 7 };
    CutDexels(stock, ball, a, b, kUp);

    // Z rays: the tool's lowest point over them along the move.
    for (int i : { 30, 60, 100, 140, 160 })
    {
        for (int j : { 90, 100, 110 })
        {
            const double x = (i + 0.5) * 0.1, y = (j + 0.5) * 0.1;
            const double lowest = (std::min)(10.0, BruteLowest(ball, a, b, x, y));
            CHECK(TopOf(stock, 2, i, j) == doctest::Approx(lowest).epsilon(2e-5));
        }
    }

    // X rays: the gap the sweep leaves at each height.
    for (int j : { 95, 100, 112 })
    {
        for (int k : { 72, 80, 90, 98 })
        {
            const double y = (j + 0.5) * 0.1, z = (k + 0.5) * 0.1;
            double x0 = 0.0, x1 = 0.0;
            if (!BruteAcross(ball, a, b, y, z, x0, x1))
                continue;
            double gapStart = 0.0, gapEnd = 0.0;
            REQUIRE(StretchAround(stock, 0, j, k, 0.5 * (x0 + x1), gapStart, gapEnd));
            CHECK(gapStart == doctest::Approx((std::max)(0.0, x0)).epsilon(2e-5));
            CHECK(gapEnd == doctest::Approx((std::min)(20.0, x1)).epsilon(2e-5));
        }
    }
}

TEST_CASE("tri-dexel cuts remove what voxel cuts do, flat, ball and tilted")
{
    struct Case
    {
        ToolProfile profile;
        Vec3 from;
        Vec3 to;
        Vec3 axis;
    };
    const double c = std::cos(30.0 * kPi / 180.0), s = std::sin(30.0 * kPi / 180.0);
    const Case cases[] = {
        { FlatTool(4), { 3, 4, 7 }, { 17, 15, 8.5 }, kUp },
        { BallTool(5), { 4, 10, 9 }, { 16, 12, 6 }, kUp },
        { FlatTool(4), { 4, 6, 8 }, { 16, 6, 8 }, { 0, -s, c } },       // tilted 30 degrees round X
        { BallTool(4), { 10, 4, 9 }, { 10, 16, 8 }, { s, 0, c } },      // tilted round Y, moving along Y
    };
    for (const Case& test : cases)
    {
        TriDexelStock dexels = DexelBlock(0.1);
        CutDexels(dexels, test.profile, test.from, test.to, test.axis);
        VoxelStock voxels;
        voxels.Reset({ { 0, 0, 0 }, { 20, 20, 10 } }, 0.1, false);
        voxels.FillAll();
        const long long cells = CutMove(voxels, test.profile, test.from, test.to, test.axis);
        const double voxelVolume = static_cast<double>(cells) * 0.001;
        // Voxels cut a sloping step with the tool at its lower tip all along it, so a little more.
        CHECK(4000.0 - dexels.Volume() == doctest::Approx(voxelVolume).epsilon(0.03));

        // Every lattice node agrees with the voxel cell at it, but for those within a
        // spacing of the cut's surface.
        int disagree = 0, cut = 0;
        for (int k = 0; k < 100; ++k)
        {
            for (int j = 0; j < 200; ++j)
            {
                for (int i = 0; i < 200; ++i)
                {
                    const bool dexel = dexels.Inside(i, j, k), voxel = voxels.Filled(i, j, k);
                    disagree += (dexel != voxel) ? 1 : 0;
                    cut += voxel ? 0 : 1;
                }
            }
        }
        CHECK(disagree < cut / 20);
    }
}

TEST_CASE("a tilted cut's three grids agree at the lattice nodes")
{
    // Each lattice node lies on a ray of each grid: they should say the same.
    const double c = std::cos(30.0 * kPi / 180.0), s = std::sin(30.0 * kPi / 180.0);
    TriDexelStock stock = DexelBlock(0.2);
    CutDexels(stock, FlatTool(6), { 4, 6, 8 }, { 16, 9, 7 }, { 0, -s, c });
    std::vector<TriDexelStock::Interval> intervals;
    const auto insideRay = [&](int axis, int iu, int iv, int along) {
        stock.Intervals(axis, stock.RayIndex(axis, iu, iv), intervals);
        const double t = (along + 0.5) * 0.2;
        for (const auto& interval : intervals)
        {
            if (t >= interval.enter && t < interval.exit)
                return true;
        }
        return false;
    };
    int disagree = 0, nodes = 0;
    for (int k = 0; k < 50; ++k)
    {
        for (int j = 0; j < 100; ++j)
        {
            for (int i = 0; i < 100; ++i)
            {
                const bool z = stock.Inside(i, j, k);
                const bool x = insideRay(0, j, k, i);
                const bool y = insideRay(1, k, i, j);
                disagree += (x != z || y != z) ? 1 : 0;
                ++nodes;
            }
        }
    }
    CHECK(disagree < nodes / 1000);
}

TEST_CASE("a cut tri-dexel stock meshes closed, in chunks or whole, holding its volume")
{
    TriDexelStock stock = DexelBlock(0.25);     // 80 x 80 x 40 nodes: 3 x 3 x 2 chunks
    REQUIRE(stock.ChunkCount() == 18);
    const StockMesh full = MeshDexelStock(stock, false);
    CHECK(Closed(full));
    CHECK(MeshVolume(full) == doctest::Approx(stock.Volume()).epsilon(0.01));
    // Merged, the box's faces are rectangles but for a ring round each, where the
    // surface rounds its edges, holding the same volume.
    const StockMesh merged = MeshDexelStock(stock);
    CHECK(merged.TriangleCount() < full.TriangleCount() / 10);
    CHECK(MeshVolume(merged) == doctest::Approx(stock.Volume()).epsilon(0.01));

    for (int c = 0; c < stock.ChunkCount(); ++c)
        stock.SetChunkClean(c);
    CutDexels(stock, BallTool(6), { 4, 5, 8 }, { 7, 8, 6.5 }, kUp);
    const std::vector<int> dirty = DirtyDexelChunks(stock);
    CHECK(!dirty.empty());
    CHECK(dirty.size() < 18);

    CutDexels(stock, FlatTool(4), { 3, 15, 7 }, { 18, 4, 9 }, kUp);
    const StockMesh cut = MeshDexelStock(stock, false);
    CHECK(Closed(cut));
    CHECK(MeshVolume(cut) == doctest::Approx(stock.Volume()).epsilon(0.01));
    const StockMesh cutMerged = MeshDexelStock(stock);
    CHECK(cutMerged.TriangleCount() < cut.TriangleCount() / 3);
    CHECK(MeshVolume(cutMerged) == doctest::Approx(stock.Volume()).epsilon(0.01));

    // Meshing only the changed chunks gives the same as meshing them all.
    std::vector<StockMesh> meshes;
    MeshDexelChunks(stock, DirtyDexelChunks(stock), meshes);
    std::vector<StockMesh> all;
    std::vector<int> every(18);
    for (int n = 0; n < 18; ++n)
        every[static_cast<size_t>(n)] = n;
    MeshDexelChunks(stock, every, all);
    for (int n : DirtyDexelChunks(stock))
        CHECK(meshes[static_cast<size_t>(n)].vertices == all[static_cast<size_t>(n)].vertices);
}

TEST_CASE("touching reads the stock without cutting it")
{
    TriDexelStock stock = DexelBlock(0.25);
    const double before = stock.Volume();
    CHECK(DexelsTouch(stock, FlatTool(4), { 5, 5, 9 }, { 15, 5, 9 }, kUp));
    CHECK_FALSE(DexelsTouch(stock, FlatTool(4), { 5, 5, 10.5 }, { 15, 5, 10.5 }, kUp));
    CHECK(stock.Volume() == before);
    // Skimming the top it doesn't count.
    CHECK_FALSE(DexelsTouch(stock, FlatTool(4), { 5, 5, 10 }, { 15, 5, 10 }, kUp));
}

TEST_CASE("a tri-dexel simulation steps, seeks and rewinds, and warns as the voxel one does")
{
    const std::vector<ToolpathMove> moves = {
        DexelMove(MoveKind::Rapid, { 5, 10, 15 }), DexelMove(MoveKind::Feed, { 5, 10, 8 }),
        DexelMove(MoveKind::Feed, { 15, 10, 8 }), DexelMove(MoveKind::Rapid, { 15, 10, 15 }),
    };
    Simulation simulation;
    simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { DexelOperation(FlatTool(4), moves) }, kDexels);
    CHECK(simulation.Model() == StockModel::TriDexel);
    CHECK(simulation.Spacing() == 0.25);
    CHECK(simulation.Stock().SizeX() == 0);
    const double whole = simulation.Dexels().Volume();
    simulation.Step(2);
    const double afterPlunge = simulation.Dexels().Volume();
    CHECK(afterPlunge < whole);
    simulation.RunToEnd();
    CHECK_FALSE(simulation.HasWarning());
    const double done = simulation.Dexels().Volume();
    CHECK(done < afterPlunge);
    simulation.SeekTo(2);
    CHECK(simulation.Dexels().Volume() == afterPlunge);
    simulation.Rewind();
    CHECK(simulation.Dexels().Volume() == whole);
    simulation.SeekTo(4);
    CHECK(simulation.Dexels().Volume() == done);
    CHECK(simulation.ChunkCount() == simulation.Dexels().ChunkCount());
    CHECK(simulation.MeshWhole().TriangleCount() > 0);

    // A rapid into stock.
    const std::vector<ToolpathMove> rapid = {
        DexelMove(MoveKind::Rapid, { 5, 10, 15 }), DexelMove(MoveKind::Rapid, { 5, 10, 5 }),
        DexelMove(MoveKind::Feed, { 15, 10, 5 }),
    };
    simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { DexelOperation(FlatTool(4), rapid) }, kDexels);
    CHECK(simulation.RunToEnd() == 2);
    CHECK(simulation.Warning() == SimWarning::RapidIntoStock);

    // The shank, then the holder of a short tool.
    const std::vector<ToolpathMove> deep = {
        DexelMove(MoveKind::Rapid, { 5, 10, 15 }), DexelMove(MoveKind::Feed, { 5, 10, -2 }),
        DexelMove(MoveKind::Feed, { 15, 10, -2 }),
    };
    Tool tool;
    tool.diameter = 4.0;
    tool.fluteLength = 4.0;
    tool.overallLength = 30.0;
    SimOperation operation = DexelOperation(ProfileOf(tool), deep);
    operation.shank = ShankOf(tool);
    operation.holder = HolderOf(tool, 20.0, 30.0);
    simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { operation }, kDexels);
    CHECK(simulation.RunToEnd() == 2);
    CHECK(simulation.Warning() == SimWarning::ShankHitsStock);
    tool.overallLength = 8.0;
    operation.profile = ProfileOf(tool);
    operation.shank = ShankOf(tool);
    operation.holder = HolderOf(tool, 20.0, 30.0);
    simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { operation }, kDexels);
    simulation.RunToEnd();
    CHECK(simulation.Warning() == SimWarning::HolderHitsStock);

    // Cutting within the flutes, and rapids just over the cut, are fine.
    const std::vector<ToolpathMove> shallow = {
        DexelMove(MoveKind::Rapid, { 5, 10, 15 }), DexelMove(MoveKind::Feed, { 5, 10, 7 }),
        DexelMove(MoveKind::Feed, { 15, 10, 7 }), DexelMove(MoveKind::Rapid, { 15, 10, 50 }),
    };
    tool.overallLength = 30.0;
    SimOperation fine = DexelOperation(ProfileOf(tool), shallow);
    fine.shank = ShankOf(tool);
    fine.holder = HolderOf(tool, 20.0, 30.0);
    simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { fine }, kDexels);
    simulation.RunToEnd();
    CHECK(simulation.Finished());
    CHECK_FALSE(simulation.HasWarning());
}

TEST_CASE("tri-dexel stock compares with the parts through voxels at its spacing")
{
    Simulation simulation;
    simulation.Start({ { 0, 0, 0 }, { 20, 20, 10 } }, {}, { DexelOperation(FlatTool(4), {
                                                                DexelMove(MoveKind::Rapid, { 5, 10, 15 }) }) },
                     kDexels);
    VoxelStock part;
    simulation.ResetPartGrid(part);
    CHECK(part.Cell() == 0.25);
    CHECK(part.SizeX() == 80);
    // A part the lower 8 mm of the box: 2 mm of stock left over it, less a cell's tolerance.
    std::vector<Vec3> box;
    {
        const Vec3 a{ 0, 0, 0 }, b{ 20, 20, 8 };
        const Vec3 corner[8] = {
            { a.x, a.y, a.z }, { b.x, a.y, a.z }, { b.x, b.y, a.z }, { a.x, b.y, a.z },
            { a.x, a.y, b.z }, { b.x, a.y, b.z }, { b.x, b.y, b.z }, { a.x, b.y, b.z },
        };
        const int faces[6][4] = { { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
                                  { 1, 2, 6, 5 }, { 2, 3, 7, 6 }, { 3, 0, 4, 7 } };
        for (const auto& f : faces)
        {
            for (int i : { f[0], f[1], f[2], f[0], f[2], f[3] })
                box.push_back(corner[i]);
        }
    }
    part.FillMesh(box);
    const StockDifference difference = simulation.CompareTo(part);
    CHECK(difference.notEnoughVolume == doctest::Approx(20.0 * 20.0 * 1.75));
    CHECK(difference.tooMuchVolume == 0.0);
}

TEST_CASE("tri-dexel timing on a 136 x 150 x 6 stock" * doctest::skip())
{
    std::vector<ToolpathMove> moves = { DexelMove(MoveKind::Rapid, { -5, 0, 10 }) };
    for (double depth : { 5.5, 5.0 })
    {
        for (double y = 0; y <= 150; y += 4)
        {
            moves.push_back(DexelMove(MoveKind::Rapid, { -5, y, 10 }));
            moves.push_back(DexelMove(MoveKind::Feed, { -5, y, depth }));
            moves.push_back(DexelMove(MoveKind::Feed, { 141, y, depth }));
            moves.push_back(DexelMove(MoveKind::Rapid, { 141, y, 10 }));
        }
    }
    // Then a ball mill zigzagging over a slope, as parallel finishing does.
    std::vector<ToolpathMove> finish = { DexelMove(MoveKind::Rapid, { 0, 0, 10 }) };
    for (double y = 0; y <= 150; y += 0.5)
    {
        for (double x = 0; x <= 136; x += 2)
        {
            const double xx = (static_cast<int>(y * 2) % 2 == 0) ? x : 136 - x;
            finish.push_back(DexelMove(MoveKind::Feed, { xx, y, 3.0 + std::sin(xx / 10.0) + std::cos(y / 13.0) }));
        }
    }
    const auto ms = [](auto a, auto b) { return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count(); };
    const StockBox box{ { 0, 0, 0 }, { 136, 150, 6 } };
    {
        // The voxels on the same moves, for comparison.
        for (double cell : { 0.05, 0.1 })
        {
            const auto t0 = std::chrono::steady_clock::now();
            Simulation simulation;
            simulation.Start(box, {}, { DexelOperation(FlatTool(6), moves), DexelOperation(BallTool(3), finish) },
                             SimResolution{ true, cell });
            simulation.RunToOperationEnd();
            const auto t1 = std::chrono::steady_clock::now();
            simulation.RunToEnd();
            const auto t2 = std::chrono::steady_clock::now();
            const StockMesh finished = simulation.MeshWhole();
            const auto t3 = std::chrono::steady_clock::now();
            MESSAGE("voxels " << cell << ": facing in " << ms(t0, t1) << " ms, finishing in " << ms(t1, t2)
                    << " ms, mesh " << finished.TriangleCount() << " in " << ms(t2, t3) << " ms");
        }
    }
    for (double spacing : DexelResolutionStops(box))
    {
        if (spacing > 0.25)
            break;
        const auto t0 = std::chrono::steady_clock::now();
        Simulation simulation;
        simulation.Start(box, {}, { DexelOperation(FlatTool(6), moves), DexelOperation(BallTool(3), finish) },
                         SimResolution{ false, spacing, StockModel::TriDexel });
        const auto t1 = std::chrono::steady_clock::now();
        simulation.RunToOperationEnd();
        const auto t2 = std::chrono::steady_clock::now();
        const StockMesh faced = simulation.MeshWhole();
        const auto t3 = std::chrono::steady_clock::now();
        simulation.RunToEnd();
        const auto t4 = std::chrono::steady_clock::now();
        const StockMesh finished = simulation.MeshWhole();
        const auto t5 = std::chrono::steady_clock::now();
        VoxelStock part;
        simulation.ResetPartGrid(part);
        part.FillAll();
        const StockDifference difference = simulation.CompareTo(part);
        const auto t6 = std::chrono::steady_clock::now();
        MESSAGE("spacing " << spacing << ": start " << ms(t0, t1) << " ms, facing " << (moves.size() - 1)
                << " moves in " << ms(t1, t2) << " ms, mesh " << faced.TriangleCount() << " in " << ms(t2, t3)
                << " ms, finishing " << finish.size() << " moves in " << ms(t3, t4) << " ms, mesh "
                << finished.TriangleCount() << " in " << ms(t4, t5) << " ms, compare " << ms(t5, t6) << " ms ("
                << difference.tooMuchVolume << " mm3), memory "
                << simulation.Dexels().MemoryBytes() / (1024 * 1024) << " MB");
        MESSAGE("  of the cuts: cutting " << simulation.Timing().cutSeconds * 1000.0 << " ms, checks "
                << simulation.Timing().checkSeconds * 1000.0 << " ms, " << simulation.Timing().pieces << " pieces");
    }
}
