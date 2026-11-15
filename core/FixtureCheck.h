// FixtureCheck.h : checking toolpaths against fixtures, and trimming them round. No Alibre or MFC code belongs here.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Fixtures.h"
#include "Simulation.h"
#include "ToolLibrary.h"
#include "Toolpath.h"

namespace camcore
{
    // The cell for a fixture's grid: half its clearance, from 0.05 to 0.25 mm,
    // coarser if the grid's top would have more than maxColumns columns.
    double FixtureCell(const FixtureMesh& mesh, double maxColumns = 4.0e6);

    // A fixture's grid (its cells filled from its facets, at FixtureCell).
    FixtureGrid MakeFixtureGrid(const FixtureMesh& mesh);

    // The fixtures, shared by every operation under one Fixtures step.
    using FixtureGrids = std::shared_ptr<const std::vector<FixtureGrid>>;
    FixtureGrids MakeFixtureGrids(const std::vector<FixtureMesh>& meshes);

    // The highest point of the fixtures plus each one's clearance, along a frame's Z.
    // False if there are none.
    bool FixtureTopAlongZ(const Frame& frame, const std::vector<FixtureMesh>& meshes, double& top);

    // What's checked against fixtures for a tool (in mm): its cutting diameter from
    // the tip to its overall length (with a ball or corner radius end, the others as
    // flat), its shank, and the holder. Each is grown by the fixture's clearance all
    // round, which is exact for flat, ball and corner radius ends.
    struct FixtureTool
    {
        ToolType type = ToolType::FlatEndMill;
        double radius = 0.0;
        double cornerRadius = 0.0;      // corner radius mills only
        double length = 0.0;
        SimCylinder shank;
        SimCylinder holder;
    };
    FixtureTool FixtureToolOf(const Tool& toolMm, double holderDiameter, double holderDepth);

    // One straight piece of a toolpath, in the operation's frame.
    struct ToolpathChord
    {
        Vec3 from;
        Vec3 to;
        size_t move = 0;        // which toolpath move it's part of
        bool rapid = false;
    };

    // A toolpath as straight pieces: arcs as chords within the tolerance; dwells left out.
    std::vector<ToolpathChord> ToolpathChords(const Toolpath& toolpath, double tolerance = 0.01);

    // Whether a straight move of the tool's tip, in the frame, comes within a
    // fixture's clearance. The fixture's cells are filled by their centres, so the
    // answer is good to about a cell.
    bool ChordHitsFixture(const FixtureGrid& fixture, const FixtureTool& tool, const Frame& frame, const Vec3& from,
                          const Vec3& to, bool* pHolder = nullptr);

    // The first place a toolpath comes within a fixture's clearance.
    struct FixtureHit
    {
        size_t fixture = 0;     // which of the fixtures
        size_t move = 0;        // which toolpath move
        bool holder = false;    // the holder, not the tool or shank
    };

    // The first hit for each fixture the toolpath comes too close to, by fixture.
    std::vector<FixtureHit> CheckFixtures(const Toolpath& toolpath, const Frame& frame, const FixtureTool& tool,
                                          const std::vector<FixtureGrid>& fixtures);

    // A phrase for a hit, such as "comes within 2 mm of Clamp<2> at move 143".
    std::string FixtureHitText(const FixtureHit& hit, const std::vector<FixtureGrid>& fixtures);

    // Makes a toolpath go round the fixtures: each stretch that comes within a
    // fixture's clearance is replaced by a lift straight up to the clearance height,
    // a rapid across, a rapid down to 2 mm above where the cut starts again, and the
    // plunge feed into it. Moves that come too close are split into straight
    // pieces; the rest stay as they were. Returns false, leaving the toolpath alone,
    // if a lift or rapid would itself come too close. trimmed counts the stretches.
    bool TrimAroundFixtures(Toolpath& toolpath, const Frame& frame, const FixtureTool& tool,
                            const std::vector<FixtureGrid>& fixtures, double clearanceHeight, double plungeFeed,
                            int& trimmed, std::string& problem);
}
