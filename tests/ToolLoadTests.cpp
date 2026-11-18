// ToolLoadTests.cpp : the tool's load in the simulation.

#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>

#include "Machine.h"
#include "Simulation.h"
#include "ToolLoad.h"

using namespace camcore;

namespace
{
    // A 6 mm two flute end mill (or ball mill) at 10,000 rpm, its chip load 0.05 mm.
    SimOperation Operation(ToolType type = ToolType::FlatEndMill)
    {
        Tool tool;
        tool.type = type;
        tool.diameter = 6.0;
        tool.fluteLength = 15.0;
        tool.overallLength = 40.0;
        tool.fluteCount = 2;
        SimOperation operation;
        operation.name = "Op";
        operation.profile = ProfileOf(tool);
        operation.load.present = true;
        operation.load.rpm = 10000.0;
        operation.load.teeth = 2;
        operation.load.chipLoadTarget = 0.05;
        return operation;
    }

    void Add(Toolpath& path, MoveKind kind, double x, double y, double z, double feed = 0.0)
    {
        ToolpathMove move;
        move.kind = kind;
        move.to = { x, y, z };
        move.feed = feed;
        path.moves.push_back(move);
    }

    // The stock 40 x 20 x 10 from the origin.
    const StockBox kBox{ { 0, 0, 0 }, { 40, 20, 10 } };
    const double kCell = 0.1;

    Simulation Run(const SimOperation& operation, StockModel model)
    {
        Simulation simulation;
        simulation.SetMeasureLoad(true);
        simulation.Start(kBox, {}, { operation }, SimResolution{ false, kCell, model });
        simulation.RunToEnd();
        return simulation;
    }
}

TEST_CASE("slotting at the chip load: 180 degrees, chips the full feed per tooth, removal rate from the cut")
{
    for (StockModel model : { StockModel::Voxels, StockModel::TriDexel })
    {
        CAPTURE(static_cast<int>(model));
        SimOperation operation = Operation();
        operation.load.cuttingForce = 700.0;
        // In from outside, then along the middle 3 mm deep.
        Add(operation.toolpath, MoveKind::Rapid, -5, 10, 7);
        Add(operation.toolpath, MoveKind::Feed, 5, 10, 7, 1000.0);
        Add(operation.toolpath, MoveKind::Feed, 30, 10, 7, 1000.0);
        Simulation simulation = Run(operation, model);
        CHECK_FALSE(simulation.HasWarning());

        const MoveLoad& load = simulation.MoveLoads()[2];
        CHECK(load.measured);
        CHECK(load.engaged);
        CHECK_FALSE(load.plunge);
        CHECK(load.feedPerTooth == doctest::Approx(0.05));
        CHECK(load.chip == doctest::Approx(0.05));
        CHECK(load.percent == doctest::Approx(100.0));
        CHECK(load.angle == doctest::Approx(180.0));
        CHECK(load.depth == doctest::Approx(3.0).epsilon(0.05));
        CHECK(load.width == doctest::Approx(6.0).epsilon(0.05));
        // 6 wide, 3 deep, at 1000 mm/min.
        CHECK(load.removalRate == doctest::Approx(18000.0).epsilon(0.05));
        // Power: 700 N/mm^2 x 300 mm^3/s = 210 W; force over the cutting speed, pi x 6 mm x 10,000 rpm.
        CHECK(load.power == doctest::Approx(700.0 * load.removalRate / 60000.0));
        CHECK(load.force == doctest::Approx(load.power / (3.14159265358979 * 0.006 * 10000.0 / 60.0)));

        const OperationLoad& total = simulation.OperationLoads()[0];
        CHECK(total.any);
        CHECK(total.maxPercent == doctest::Approx(100.0));
        CHECK(total.AverageChip() == doctest::Approx(0.05).epsilon(0.02));
        CHECK(MoveLoadText(load, operation.load).find("Chip 0.05 of 0.05 mm/tooth (100%)") == 0);
    }
}

TEST_CASE("a side cut a quarter of the diameter wide engages 60 degrees and thins the chip to sin 60")
{
    for (StockModel model : { StockModel::Voxels, StockModel::TriDexel })
    {
        CAPTURE(static_cast<int>(model));
        SimOperation operation = Operation();
        // The tool's edge 1.5 mm into the stock's +Y side, 2 mm deep, along +X: the stock on its right.
        Add(operation.toolpath, MoveKind::Rapid, -5, 21.5, 8);
        Add(operation.toolpath, MoveKind::Feed, 30, 21.5, 8, 1000.0);
        Simulation simulation = Run(operation, model);
        const MoveLoad& load = simulation.MoveLoads()[1];
        CHECK(load.engaged);
        CHECK(load.angle == doctest::Approx(60.0).epsilon(0.1));
        CHECK(load.chip == doctest::Approx(0.05 * std::sin(60.0 * 3.14159265358979 / 180.0)).epsilon(0.05));
        CHECK(load.width == doctest::Approx(1.5).epsilon(0.15));
        CHECK(load.depth == doctest::Approx(2.0).epsilon(0.05));
    }
}

TEST_CASE("cutting air and plunging")
{
    for (StockModel model : { StockModel::Voxels, StockModel::TriDexel })
    {
        CAPTURE(static_cast<int>(model));
        SimOperation operation = Operation();
        Add(operation.toolpath, MoveKind::Rapid, 10, 10, 15);
        Add(operation.toolpath, MoveKind::Feed, 20, 10, 15, 1000.0);    // above the stock
        Add(operation.toolpath, MoveKind::Feed, 20, 10, 7, 300.0);      // down into it
        Add(operation.toolpath, MoveKind::Feed, 20, 10, 15, 1000.0);    // back up its own hole
        Simulation simulation = Run(operation, model);
        const auto& loads = simulation.MoveLoads();
        CHECK_FALSE(loads[0].measured);    // the rapid
        CHECK(loads[1].measured);
        CHECK_FALSE(loads[1].engaged);
        CHECK(MoveLoadText(loads[1], operation.load) == "Air");
        CHECK(loads[2].engaged);
        CHECK(loads[2].plunge);
        CHECK(loads[2].chip == doctest::Approx(300.0 / (10000.0 * 2.0)));
        // 3 mm of the 8 mm move is in the stock.
        CHECK(loads[2].removalRate == doctest::Approx(3.14159265358979 * 9.0 * 3.0 * 300.0 / 8.0).epsilon(0.08));
        CHECK_FALSE(loads[3].engaged);
    }
}

TEST_CASE("a ball mill cutting shallow is measured at its smaller diameter there")
{
    SimOperation operation = Operation(ToolType::BallMill);
    Add(operation.toolpath, MoveKind::Rapid, -5, 10, 9.5);
    Add(operation.toolpath, MoveKind::Feed, 30, 10, 9.5, 1000.0);
    Simulation simulation = Run(operation, StockModel::Voxels);
    const MoveLoad& load = simulation.MoveLoads()[1];
    CHECK(load.engaged);
    // 0.5 mm deep in a 3 mm radius ball: 2 x sqrt(9 - 2.5^2).
    CHECK(load.effectiveDiameter == doctest::Approx(2.0 * std::sqrt(2.75)).epsilon(0.05));
    CHECK(load.depth == doctest::Approx(0.5).epsilon(0.15));
}

TEST_CASE("stopping on overload stops after the first move over the limit")
{
    SimOperation operation = Operation();
    operation.load.chipLoadTarget = 0.04;   // the 0.05 chips are 125%
    Add(operation.toolpath, MoveKind::Rapid, -5, 10, 15);
    Add(operation.toolpath, MoveKind::Feed, -5, 10, 7, 300.0);     // down beside the stock
    Add(operation.toolpath, MoveKind::Feed, 5, 10, 7, 1000.0);     // in
    Add(operation.toolpath, MoveKind::Feed, 30, 10, 7, 1000.0);
    for (StockModel model : { StockModel::Voxels, StockModel::TriDexel })
    {
        CAPTURE(static_cast<int>(model));
        Simulation simulation;
        simulation.SetMeasureLoad(true);
        simulation.SetStopOnOverload(true, 110.0);
        simulation.Start(kBox, {}, { operation }, SimResolution{ false, kCell, model });
        simulation.RunToEnd();
        CHECK(simulation.Warning() == SimWarning::ToolOverloaded);
        CHECK(simulation.WarningMove() == 2);
        CHECK(simulation.OperationLoads()[0].maxPercent == doctest::Approx(125.0));

        // Under the limit, and not measured at all unless asked.
        simulation.SetStopOnOverload(true, 150.0);
        simulation.Start(kBox, {}, { operation }, SimResolution{ false, kCell, model });
        simulation.RunToEnd();
        CHECK_FALSE(simulation.HasWarning());
        simulation.SetMeasureLoad(false);
        simulation.SetStopOnOverload(true, 110.0);
        simulation.Start(kBox, {}, { operation }, SimResolution{ false, kCell, model });
        simulation.RunToEnd();
        CHECK_FALSE(simulation.HasWarning());
        CHECK_FALSE(simulation.MoveLoads()[2].measured);
    }
}

TEST_CASE("rewinding forgets the loads")
{
    SimOperation operation = Operation();
    Add(operation.toolpath, MoveKind::Rapid, -5, 10, 7);
    Add(operation.toolpath, MoveKind::Feed, 30, 10, 7, 1000.0);
    Simulation simulation = Run(operation, StockModel::Voxels);
    CHECK(simulation.OperationLoads()[0].any);
    simulation.Rewind();
    CHECK_FALSE(simulation.OperationLoads()[0].any);
    CHECK_FALSE(simulation.MoveLoads()[1].measured);
}

TEST_CASE("the load setup takes the tool's chip load times the material's factor and the stickout")
{
    Tool tool;
    tool.type = ToolType::FlatEndMill;
    tool.diameter = 6.0;
    tool.overallLength = 15.0;
    tool.fluteCount = 3;
    tool.chipLoad = 0.05;
    tool.toolMaterial = ToolMaterial::Carbide;
    Material material;
    material.CuttingFor(ToolMaterial::Carbide) = { true, 300.0, 80.0 };
    material.cuttingForce = 700.0;

    ToolLoadSetup setup = LoadSetupFor(tool, 12000.0, &material);
    CHECK(setup.present);
    CHECK(setup.teeth == 3);
    CHECK(setup.rpm == 12000.0);
    CHECK(setup.chipLoadTarget == doctest::Approx(0.04));
    CHECK(setup.cuttingForce == 700.0);

    // 36 long is 6 diameters: (18 / 36)^3 = 1/8, under the floor of a quarter.
    tool.overallLength = 36.0;
    CHECK(LoadSetupFor(tool, 12000.0, &material).chipLoadTarget == doctest::Approx(0.01));

    // No material: the tool's own; no chip load: no target. A drill has one "tooth".
    tool.overallLength = 15.0;
    CHECK(LoadSetupFor(tool, 12000.0, nullptr).chipLoadTarget == doctest::Approx(0.05));
    CHECK(LoadSetupFor(tool, 12000.0, nullptr).cuttingForce == 0.0);
    tool.chipLoad = 0.0;
    CHECK(LoadSetupFor(tool, 12000.0, &material).chipLoadTarget == 0.0);
    // Without one, the programmed feed per tooth: 1800 mm/min at 12,000 rpm with 3 flutes is 0.05.
    const ToolLoadSetup programmed = LoadSetupFor(tool, 12000.0, &material, 0.0, 1800.0);
    CHECK(programmed.chipLoadTarget == doctest::Approx(0.05));
    CHECK(programmed.targetProgrammed);
    CHECK_FALSE(setup.targetProgrammed);
    MoveLoad slot;
    slot.measured = slot.engaged = true;
    slot.chip = 0.05;
    slot.percent = 100.0;
    CHECK(MoveLoadText(slot, programmed).find("of 0.05 mm/tooth programmed (100%)") != std::string::npos);
    tool.type = ToolType::Drill;
    CHECK(LoadSetupFor(tool, 12000.0, &material).teeth == 1);
}

TEST_CASE("load bands")
{
    MoveLoad load;
    CHECK(BandOf(load, 150.0) == LoadBand::NotCutting);
    load.measured = true;
    CHECK(BandOf(load, 150.0) == LoadBand::Air);
    load.engaged = true;
    CHECK(BandOf(load, 150.0) == LoadBand::NoTarget);
    load.percent = 90.0;
    CHECK(BandOf(load, 150.0) == LoadBand::Under);
    load.percent = 120.0;
    CHECK(BandOf(load, 150.0) == LoadBand::Over);
    load.percent = 160.0;
    CHECK(BandOf(load, 150.0) == LoadBand::Overloaded);
}

TEST_CASE("a material's specific cutting force is saved and loaded")
{
    MaterialList list;
    Material material;
    material.id = "m1";
    material.name = "Aluminium";
    material.cuttingForce = 700.0;
    list.materials.push_back(material);
    Material other;
    other.id = "m2";
    other.name = "Mystery";
    list.materials.push_back(other);
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "CamCoreTests-materials-kc.json";
    REQUIRE(SaveMaterials(list, path).ok);
    const MaterialsLoadResult loaded = LoadMaterials(path);
    std::filesystem::remove(path);
    REQUIRE(loaded.status == LoadStatus::Ok);
    CHECK(loaded.list.materials[0].cuttingForce == 700.0);
    CHECK(loaded.list.materials[1].cuttingForce == 0.0);

    list.materials[0].cuttingForce = -1.0;
    CHECK_FALSE(ValidateMaterials(list).empty());
}

TEST_CASE("power as a percent of the spindle's, and red when it's over")
{
    SimOperation operation = Operation();
    operation.load.cuttingForce = 700.0;
    operation.load.spindlePower = 150.0;   // W: the 210 W slot is 140%
    Add(operation.toolpath, MoveKind::Rapid, -5, 10, 7);
    Add(operation.toolpath, MoveKind::Feed, 5, 10, 7, 1000.0);
    Add(operation.toolpath, MoveKind::Feed, 30, 10, 7, 1000.0);
    Simulation simulation = Run(operation, StockModel::Voxels);
    const MoveLoad& load = simulation.MoveLoads()[2];
    CHECK(load.powerPercent == doctest::Approx(100.0 * load.power / 150.0));
    CHECK(load.powerPercent == doctest::Approx(140.0).epsilon(0.05));
    CHECK(load.LoadPercent() == load.powerPercent);
    // The chip is at 100%, but the power is over: red.
    CHECK(BandOf(load, 150.0) == LoadBand::Overloaded);
    CHECK(simulation.OperationLoads()[0].maxPowerPercent == doctest::Approx(load.powerPercent));
    CHECK(MoveLoadText(load, operation.load).find("% of 0.15 kW)") != std::string::npos);

    // No spindle power: no percent.
    operation.load.spindlePower = 0.0;
    Simulation unknown = Run(operation, StockModel::Voxels);
    CHECK(unknown.MoveLoads()[2].powerPercent < 0.0);
    CHECK(BandOf(unknown.MoveLoads()[2], 150.0) == LoadBand::Under);
}

TEST_CASE("the machine's spindle power is saved when set, and goes into the load setup in watts")
{
    Machine machine;
    machine.id = "power-test";
    machine.name = "Power test";
    machine.spindlePower = 2.2;
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "CamCoreTests-spindle-power.json";
    REQUIRE(SaveMachine(machine, path).ok);
    MachineLoadResult loaded = LoadMachine(path);
    REQUIRE(loaded.status == LoadStatus::Ok);
    CHECK(loaded.machine.spindlePower == 2.2);

    machine.spindlePower = 0.0;
    REQUIRE(SaveMachine(machine, path).ok);
    loaded = LoadMachine(path);
    std::filesystem::remove(path);
    REQUIRE(loaded.status == LoadStatus::Ok);
    CHECK(loaded.machine.spindlePower == 0.0);

    machine.spindlePower = -1.0;
    CHECK_FALSE(ValidateMachine(machine).empty());

    Tool tool;
    tool.diameter = 6.0;
    CHECK(LoadSetupFor(tool, 10000.0, nullptr, 2.2).spindlePower == doctest::Approx(2200.0));
}

TEST_CASE("overlapping parts fill as their union when each is a solid of its own")
{
    // Two 10 mm cubes overlapping by half; counted as one mesh the overlap reads as outside.
    const auto box = [](const Vec3& a, const Vec3& b) {
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
    };
    std::vector<Vec3> triangles = box({ 0, 0, 0 }, { 10, 10, 10 });
    const size_t second = triangles.size();
    const std::vector<Vec3> other = box({ 0, 0, 5 }, { 10, 10, 15 });     // stacked, overlapping 5 to 10
    triangles.insert(triangles.end(), other.begin(), other.end());

    VoxelStock together, apart;
    for (VoxelStock* grid : { &together, &apart })
        grid->Reset({ { 0, 0, 0 }, { 10, 10, 15 } }, 0.5, true);
    together.FillMesh(triangles);
    apart.FillMesh(triangles, { 0, second });
    // 20 x 20 columns: one mesh fills 0-5 and 10-15 (20 cells a column), each apart all 30.
    CHECK(together.Count() == 20LL * 20 * 20);
    CHECK(apart.Count() == 20LL * 20 * 30);
    // Without the list the mesh is one solid, as before.
    VoxelStock single;
    single.Reset({ { 0, 0, 0 }, { 10, 10, 15 } }, 0.5, true);
    single.FillMesh(box({ 0, 0, 0 }, { 10, 10, 10 }));
    CHECK(single.Count() == 20LL * 20 * 20);
}
