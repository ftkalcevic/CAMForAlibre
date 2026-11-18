// ToolLoad.h : how hard the tool works in the simulation: chip load per tooth,
// engagement, removal rate, power and force. No Alibre or MFC code belongs here.
//
// Each feed move is measured against the stock as it is before the move cuts:
// points just outside the tool's front half, at heights up its flutes, are tested
// for stock. From those come the engagement angle, the depth and width of cut, and
// the radial chip thinning; the removal rate comes from what the move actually cut.

#pragma once

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "Materials.h"
#include "Origin.h"     // Vec3
#include "ToolLibrary.h"

namespace camcore
{
    struct ToolProfile;

    // What the load is worked out from, for one operation.
    struct ToolLoadSetup
    {
        bool present = false;       // false: the operation isn't measured
        double rpm = 0.0;           // the spindle speed
        int teeth = 1;              // flutes; 1 for a drill, whose chip load is per revolution
        bool drill = false;
        double chipLoadTarget = 0.0;    // mm per tooth (per revolution for drills) the tool should cut; 0 isn't set
        bool targetProgrammed = false;  // the target is the operation's own feed per tooth, the tool having no chip load
        double cuttingForce = 0.0;  // the material's specific cutting force, N/mm^2; 0 isn't known
        double spindlePower = 0.0;  // the machine's spindle power, W; 0 isn't known
    };

    // How the tool meets the stock at one place.
    struct ToolEngagement
    {
        bool engaged = false;
        bool plunge = false;            // moving mostly down the tool's axis, cutting with its tip
        double angle = 0.0;             // degrees round the tool in contact (up to 180; a plunge is 360)
        double depth = 0.0;             // mm along the axis in contact
        double width = 0.0;             // mm across the move in contact
        double thinning = 0.0;          // the chip's thickness over the feed per tooth, 0 to 1
        double effectiveDiameter = 0.0; // the tool's widest diameter in contact
    };

    // Whether a point (assembly mm) is in the stock.
    using StockTest = std::function<bool(const Vec3&)>;

    constexpr double kLoadSampleStep = 5.0;         // degrees between the points round the tool
    constexpr int kLoadHeightSamples = 64;          // at most this many heights up the flutes
    constexpr double kLoadPositionSpacing = 2.0;    // mm between places measured along a long move

    // Measures a move from `from` to `to`, the tool pointing along axis (unit, tip up),
    // against the stock before the move. cell is the stock's cell size or dexel spacing.
    // The heaviest of the places measured along the move is returned.
    ToolEngagement MeasureEngagement(const StockTest& inStock, const ToolProfile& profile, const Vec3& from,
                                     const Vec3& to, const Vec3& axis, double cell);

    // The setup for a tool in mm, at a spindle speed, cutting a material (or none):
    // the target is the tool's chip load times the material's chip load factor for
    // the tool's tool material, and the feeds calculator's stickout factor. A tool
    // with no chip load is compared with the programmed feed per tooth instead: the
    // operation's cut feed (its plunge feed for a drill) over rpm x flutes (Frank's
    // choice), so 100% is the full chip it was programmed for. No feed, no target.
    // spindlePower is the machine's, in kW (0 isn't known).
    ToolLoadSetup LoadSetupFor(const Tool& tool, double spindleRpm, const Material* material,
                               double spindlePower = 0.0, double programmedFeed = 0.0);

    // A move's load.
    struct MoveLoad
    {
        bool measured = false;      // a feed move that was measured
        bool engaged = false;
        bool plunge = false;
        double feed = 0.0;          // mm/min
        double feedPerTooth = 0.0;  // mm (per revolution for drills); along the axis for a plunge
        double chip = 0.0;          // the thickest chip, mm
        double percent = -1.0;      // chip over the target, in percent; negative when there's no target
        double angle = 0.0;         // degrees
        double depth = 0.0;         // mm
        double width = 0.0;         // mm
        double effectiveDiameter = 0.0;
        double removalRate = 0.0;   // mm^3 per minute
        double power = 0.0;         // W; 0 when the material's cutting force isn't known
        double force = 0.0;         // N, tangential
        double powerPercent = -1.0; // power over the spindle's, in percent; negative when either isn't known

        // What's drawn and graphed: the higher of the chip's and the power's percent,
        // or negative when neither is known.
        double LoadPercent() const { return (std::max)(percent, powerPercent); }
    };

    // The load from an engagement, the setup, the move's feed, and the volume (mm^3) the
    // piece of the move `length` mm long removed.
    MoveLoad LoadOf(const ToolEngagement& engagement, const ToolLoadSetup& setup, double feed, double removed,
                    double length);

    // Keeps the heavier of two loads on one move (its pieces, when the simulation
    // stops part way along it). An engaged load is heavier than air.
    void CombineLoad(MoveLoad& into, const MoveLoad& piece);

    // An operation's loads so far.
    struct OperationLoad
    {
        bool any = false;           // some move was engaged
        double maxPercent = -1.0;
        double maxChip = 0.0;
        size_t worstMove = 0;       // the simulation's move with the highest load (percent, or chip without a target)
        double chipSeconds = 0.0;   // chip x seconds, over the engaged moves
        double engagedSeconds = 0.0;
        double maxAngle = 0.0;
        double maxDepth = 0.0;
        double maxRemovalRate = 0.0;
        double maxPower = 0.0;
        double maxForce = 0.0;
        double maxPowerPercent = -1.0;

        double AverageChip() const { return engagedSeconds > 0.0 ? chipSeconds / engagedSeconds : 0.0; }
    };
    void AddLoad(OperationLoad& operation, const MoveLoad& load, double seconds, size_t move);

    // How a move's load is drawn: not a measured feed, cutting air, at most the
    // target, over it, over the stop limit (or needing more than the spindle's power),
    // or engaged with no target to judge by.
    enum class LoadBand
    {
        NotCutting,
        Air,
        Under,
        Over,
        Overloaded,
        NoTarget,
    };
    LoadBand BandOf(const MoveLoad& load, double limitPercent);

    // For the dialog and the log: one move's load, and an operation's.
    // Such as "Chip 0.032 of 0.050 mm/tooth (64%), engaged 72 deg, 3.00 deep x 1.50 wide, 2.1 cm3/min, 0.35 kW, 120 N".
    std::string MoveLoadText(const MoveLoad& load, const ToolLoadSetup& setup);
    std::string OperationLoadText(const OperationLoad& load, const ToolLoadSetup& setup);
}
