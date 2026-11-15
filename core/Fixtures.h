// Fixtures.h : work holding (vices, clamps, screws) the tool must keep clear of. No Alibre or MFC code belongs here.
//
// A Fixtures step in the sequence lists assembly parts that hold the stock. The
// operations below it, up to the next Fixtures step, watch out for them: by
// default a toolpath that comes too close is a warning; with Trim, toolpaths are
// made to go round them. Lengths are in millimetres.

#pragma once

#include <string>
#include <vector>

#include "Origin.h"

namespace camcore
{
    constexpr double kDefaultFixtureClearance = 2.0;

    // A part of the assembly that holds the stock, found again by its occurrence's key.
    struct FixturePart
    {
        PersistentKey occurrenceKey;
        std::string name;
        double clearance = kDefaultFixtureClearance;   // how close the tool and holder may come to it
    };

    // A Fixtures step's settings.
    struct FixtureSettings
    {
        std::vector<FixturePart> parts;
        bool trim = false;                                  // make toolpaths go round them, not just warn
        double defaultClearance = kDefaultFixtureClearance; // given to parts as they're added
    };

    // Returns a sentence for each problem. Empty means the settings are valid.
    std::vector<std::string> ValidateFixtures(const FixtureSettings& settings);

    // A fixture part's facets, in assembly mm, closed.
    struct FixtureMesh
    {
        std::string name;
        std::vector<Vec3> triangles;
        double clearance = kDefaultFixtureClearance;
    };

}
