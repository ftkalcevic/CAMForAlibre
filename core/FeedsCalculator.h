// FeedsCalculator.h : works out spindle speed and feeds from the tool, the material and the cut.
// No Alibre or MFC code belongs here.
//
// The surface speed and a chip load factor come from the material, for the tool's
// tool material; the chip load itself is the tool's. The calculator adds the depth
// and width of cut and an aggression from 50 to 150%.

#pragma once

#include <string>
#include <vector>

#include "Feeds.h"
#include "Materials.h"
#include "ToolLibrary.h"

namespace camcore
{
    struct FeedsCalcInput
    {
        Tool tool;                      // in millimetres (ToolInMillimetres)
        MaterialCutting cutting;        // the material's values for the tool's tool material
        double depth = 0.0;             // mm, along the tool
        double width = 0.0;             // mm, across it (the stepover)
        double aggression = 1.0;        // 0.5 to 1.5, scaling the chip load
        double chipLoad = 0.0;          // mm per tooth (per revolution for drills); 0 takes the tool's
        double maxSpindleRpm = 0.0;     // the machine's limit; 0 is none
        FeedSettings start;             // the feeds being replaced: transfer is kept from here
    };

    struct FeedsCalcResult
    {
        FeedSettings feeds;

        double effectiveDiameter = 0.0; // the tool's diameter at the depth of cut
        double rpmWanted = 0.0;         // from the surface speed
        double rpm = 0.0;               // after the machine's limit
        bool rpmLimited = false;

        double toolChipLoad = 0.0;      // the tool's (or the one given)
        double materialFactor = 1.0;
        double aggression = 1.0;
        double thinningFactor = 1.0;    // radial chip thinning, at most kMaxThinning
        double stickoutFactor = 1.0;    // under 1 for tools longer than 3 diameters
        double chipLoad = 0.0;          // per tooth, as cut

        double removalRate = 0.0;       // mm^3 per minute (cm^3 per minute x 1000)

        std::vector<std::string> problems;  // empty if the feeds were worked out
    };

    constexpr double kMaxThinning = 3.0;
    constexpr double kMinStickoutFactor = 0.25;
    constexpr double kPlungeRatio = 0.3;
    constexpr double kEngageRatio = 0.5;

    FeedsCalcResult CalculateFeeds(const FeedsCalcInput& input);

    // Typical starting values for a few materials, for the material editor.
    struct TypicalMaterial
    {
        std::string name;
        double surfaceSpeed[kToolMaterialCount];    // m/min, by ToolMaterial
        double chipLoadFactor;                      // percent
        double cuttingForce;                        // N/mm^2, specific cutting force
    };
    const std::vector<TypicalMaterial>& TypicalMaterials();
}
