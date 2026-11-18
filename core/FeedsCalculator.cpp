// FeedsCalculator.cpp : works out spindle speed and feeds from the tool, the material and the cut.

#include "FeedsCalculator.h"

#include <algorithm>
#include <cmath>

#include "Simulation.h"     // ProfileOf

namespace camcore
{
    namespace
    {
        const double kPi = 3.14159265358979323846;

        // The tool's cutting diameter at the depth of cut: less than the diameter
        // for ball, corner radius, V and chamfer mills cutting shallower than their curve or point.
        double EffectiveDiameter(const Tool& tool, double depth)
        {
            if (tool.type == ToolType::Drill || tool.type == ToolType::FlatEndMill || tool.type == ToolType::ThreadMill)
                return tool.diameter;
            const ToolProfile profile = ProfileOf(tool);
            const double radius = profile.RadiusAt(depth);
            if (!(radius > 0.0))
                return tool.diameter;
            return std::min(tool.diameter, 2.0 * radius);
        }
    }

    FeedsCalcResult CalculateFeeds(const FeedsCalcInput& input)
    {
        FeedsCalcResult result;
        result.feeds = input.start;
        const Tool& tool = input.tool;
        const bool drill = tool.type == ToolType::Drill;

        if (!(tool.diameter > 0.0))
            result.problems.push_back("The tool has no diameter.");
        if (!input.cutting.set)
        {
            result.problems.push_back("The material has no surface speed for " + ToolMaterialTitle(tool.toolMaterial) +
                                      " tools.");
        }
        result.toolChipLoad = input.chipLoad > 0.0 ? input.chipLoad : tool.chipLoad;
        if (!(result.toolChipLoad > 0.0))
            result.problems.push_back("There's no chip load.");
        if (!(input.depth > 0.0))
            result.problems.push_back("The depth of cut must be more than 0.");
        if (!drill && !(input.width > 0.0))
            result.problems.push_back("The width of cut must be more than 0.");
        if (!(input.aggression >= 0.5 && input.aggression <= 1.5))
            result.problems.push_back("The aggression must be from 50 to 150%.");
        if (!result.problems.empty())
            return result;

        const double d = tool.diameter;
        result.effectiveDiameter = drill ? d : EffectiveDiameter(tool, input.depth);
        result.rpmWanted = input.cutting.surfaceSpeed * 1000.0 / (kPi * result.effectiveDiameter);
        result.rpm = result.rpmWanted;
        if (input.maxSpindleRpm > 0.0 && result.rpm > input.maxSpindleRpm)
        {
            result.rpm = input.maxSpindleRpm;
            result.rpmLimited = true;
        }

        result.materialFactor = input.cutting.chipLoadFactor / 100.0;
        result.aggression = input.aggression;

        // A cut narrower than half the diameter makes thinner chips than the
        // chip load, so the feed goes up to make them as thick.
        const double width = std::min(input.width, d);
        if (!drill && width < d / 2.0)
            result.thinningFactor = std::min(kMaxThinning, d / (2.0 * std::sqrt(width * (d - width))));

        // A long tool bends: its feed comes down with the cube of the length past 3 diameters.
        const double stickout = tool.overallLength;
        if (stickout > 3.0 * d)
            result.stickoutFactor = std::max(kMinStickoutFactor, std::pow(3.0 * d / stickout, 3.0));

        result.chipLoad = result.toolChipLoad * result.materialFactor * result.aggression * result.thinningFactor *
                          result.stickoutFactor;

        const double teeth = drill ? 1.0 : std::max(1, tool.fluteCount);
        const double cut = result.rpm * teeth * result.chipLoad;

        FeedSettings& feeds = result.feeds;
        feeds.spindleRpm = std::round(result.rpm);
        feeds.cut = std::round(cut);
        feeds.plunge = std::round(drill ? cut : cut * kPlungeRatio);
        feeds.engage = std::round(cut * kEngageRatio);
        feeds.approach = feeds.cut;
        feeds.retract = feeds.cut;
        feeds.departure = feeds.cut;

        result.removalRate = drill ? kPi * d * d / 4.0 * cut : input.depth * width * cut;
        return result;
    }

    const std::vector<TypicalMaterial>& TypicalMaterials()
    {
        // Surface speeds for HSS, cobalt HSS, carbide and coated carbide: conservative starting points.
        static const std::vector<TypicalMaterial> materials =
        {
            { "Aluminium", { 90, 110, 300, 400 }, 100, 700 },
            { "Brass", { 60, 75, 200, 250 }, 80, 800 },
            { "Mild steel", { 25, 30, 100, 150 }, 50, 2000 },
            { "Stainless steel", { 12, 15, 60, 90 }, 40, 2500 },
            { "Acrylic and plastics", { 100, 100, 300, 300 }, 100, 150 },
            { "Hardwood", { 300, 300, 500, 500 }, 100, 60 },
            { "MDF", { 300, 300, 600, 600 }, 120, 40 },
        };
        return materials;
    }
}
