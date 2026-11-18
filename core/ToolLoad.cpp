// ToolLoad.cpp : how hard the tool works in the simulation. See ToolLoad.h.

#include "ToolLoad.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "FeedsCalculator.h"    // kMinStickoutFactor
#include "Simulation.h"     // ToolProfile

namespace camcore
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        std::string Format(const char* format, double a, double b = 0.0, double c = 0.0)
        {
            char text[96];
            std::snprintf(text, sizeof(text), format, a, b, c);
            return text;
        }

        // Whether one engagement is heavier than another: thicker chips, then more in contact.
        bool Heavier(const ToolEngagement& a, const ToolEngagement& b)
        {
            if (a.engaged != b.engaged)
                return a.engaged;
            if (std::fabs(a.thinning - b.thinning) > 1e-9)
                return a.thinning > b.thinning;
            return a.depth * a.width > b.depth * b.width;
        }
    }

    ToolEngagement MeasureEngagement(const StockTest& inStock, const ToolProfile& profile, const Vec3& from,
                                     const Vec3& to, const Vec3& axis, double cell)
    {
        ToolEngagement best;
        Vec3 w;
        if (!Normalise(axis, w))
            return best;
        const Vec3 d = to - from;
        const double moveLength = Length(d);
        if (moveLength < 1e-9)
            return best;
        const double along = Dot(d, w);
        const Vec3 side = d - w * along;
        const double sideLength = Length(side);

        // Points this far outside the tool: in a cell whose centre is outside it, so
        // not cut by the tool where it's been, nor by the floor under its tip.
        const double c = (std::max)(cell, 1e-4);
        const double out = 0.75 * c;
        const int positions = (std::max)(1, static_cast<int>(std::ceil(moveLength / kLoadPositionSpacing)));
        const auto positionAt = [&](int i) { return from + d * ((i + 0.5) / positions); };

        if (sideLength <= std::fabs(along))
        {
            // Mostly along the axis. Going up, the tool leaves the cut; going down, its
            // tip cuts whatever is just under it.
            if (along >= 0.0)
                return best;
            const double r = profile.radius;
            for (int i = 0; i < positions && !best.engaged; ++i)
            {
                const Vec3 below = positionAt(i) - w * out;
                Vec3 u0;
                if (!Normalise(Cross(w, std::fabs(w.x) < 0.9 ? Vec3{ 1.0, 0.0, 0.0 } : Vec3{ 0.0, 1.0, 0.0 }), u0))
                    return best;
                const Vec3 v0 = Cross(w, u0);
                for (int ring = 0; ring <= 4 && !best.engaged; ++ring)
                {
                    const double rr = r * ring / 4.0;
                    const int count = (ring == 0) ? 1 : 24;
                    for (int a = 0; a < count; ++a)
                    {
                        const double t = 2.0 * kPi * a / count;
                        if (inStock(below + u0 * (rr * std::cos(t)) + v0 * (rr * std::sin(t))))
                        {
                            best.engaged = true;
                            break;
                        }
                    }
                }
            }
            if (best.engaged)
            {
                best.plunge = true;
                best.angle = 360.0;
                best.width = 2.0 * r;
                best.thinning = 1.0;
                best.effectiveDiameter = 2.0 * r;
            }
            return best;
        }

        // Across the axis: the front half of the tool, from its left (-90 degrees) to
        // its right (+90), at heights up the flutes.
        const Vec3 u = side * (1.0 / sideLength);   // forward
        const Vec3 s = Cross(w, u);                 // to the left
        const int angles = static_cast<int>(std::lround(180.0 / kLoadSampleStep)) + 1;
        std::vector<double> cosines(angles), sines(angles);
        for (int a = 0; a < angles; ++a)
        {
            const double theta = (-90.0 + a * kLoadSampleStep) * kPi / 180.0;
            cosines[a] = std::cos(theta);
            sines[a] = std::sin(theta);
        }
        const double top = (std::max)(out, profile.length);
        const int heights =
            (std::min)(kLoadHeightSamples, (std::max)(1, static_cast<int>(std::floor((top - out) / c)) + 1));
        const double step = (heights > 1) ? (top - out) / (heights - 1) : 0.0;
        const auto heightAt = [&](int k) { return out + k * step; };

        for (int i = 0; i < positions; ++i)
        {
            const Vec3 p = positionAt(i);
            // The angles in contact at a height, as bits.
            const auto engagedAt = [&](double h) {
                std::uint64_t mask = 0;
                const double radius = profile.RadiusAt(h);
                if (radius < 0.0)
                    return mask;
                const double reach = radius + out;
                const Vec3 centre = p + w * h;
                for (int a = 0; a < angles; ++a)
                {
                    if (inStock(centre + u * (reach * cosines[a]) + s * (reach * sines[a])))
                        mask |= std::uint64_t(1) << a;
                }
                return mask;
            };

            std::uint64_t all = 0;
            int low = -1, high = -1;
            double widest = 0.0;
            for (int k = 0; k < heights; ++k)
            {
                const double h = heightAt(k);
                const std::uint64_t mask = engagedAt(h);
                if (mask == 0)
                    continue;
                all |= mask;
                if (low < 0)
                    low = k;
                high = k;
                widest = (std::max)(widest, profile.RadiusAt(h));
            }
            if (all == 0)
                continue;

            // The top and bottom of the contact, between the heights tried.
            double topH = heightAt(high);
            if (high + 1 < heights)
            {
                double lo = topH, hi = heightAt(high + 1);
                for (int b = 0; b < 8; ++b)
                {
                    const double mid = 0.5 * (lo + hi);
                    (engagedAt(mid) != 0 ? lo : hi) = mid;
                }
                topH = 0.5 * (lo + hi);
            }
            widest = (std::max)(widest, profile.RadiusAt(topH));
            double bottomH = 0.0;
            if (low > 0)
            {
                double lo = heightAt(low - 1), hi = heightAt(low);
                for (int b = 0; b < 8; ++b)
                {
                    const double mid = 0.5 * (lo + hi);
                    (engagedAt(mid) != 0 ? hi : lo) = mid;
                }
                bottomH = 0.5 * (lo + hi);
            }

            int first = 0, last = angles - 1;
            while (((all >> first) & 1) == 0)
                ++first;
            while (((all >> last) & 1) == 0)
                --last;
            ToolEngagement here;
            here.engaged = true;
            here.depth = (std::max)(0.0, topH - bottomH);
            here.angle = (last > first) ? (last - first) * kLoadSampleStep : kLoadSampleStep / 2.0;
            here.width = widest * (sines[last] - sines[first]);
            if (here.width <= 0.0)
                here.width = widest * (1.0 - std::cos(kLoadSampleStep / 2.0 * kPi / 180.0));
            // The chip is thickest where the tooth moves most nearly along the feed.
            here.thinning = (sines[first] <= 0.0 && sines[last] >= 0.0)
                                ? 1.0
                                : (std::max)(cosines[first], cosines[last]);
            here.effectiveDiameter = 2.0 * widest;
            if (Heavier(here, best))
                best = here;
        }
        return best;
    }

    ToolLoadSetup LoadSetupFor(const Tool& tool, double spindleRpm, const Material* material, double spindlePower,
                               double programmedFeed)
    {
        ToolLoadSetup setup;
        setup.present = true;
        setup.spindlePower = (std::max)(0.0, spindlePower) * 1000.0;
        setup.rpm = spindleRpm;
        setup.drill = tool.type == ToolType::Drill;
        setup.teeth = setup.drill ? 1 : (std::max)(1, tool.fluteCount);
        double factor = 1.0;
        if (material != nullptr)
        {
            const MaterialCutting& cutting = material->CuttingFor(tool.toolMaterial);
            if (cutting.set)
                factor = cutting.chipLoadFactor / 100.0;
            setup.cuttingForce = material->cuttingForce;
        }
        // A long tool bends: as the feeds calculator, less past 3 diameters.
        const double d = tool.diameter;
        if (d > 0.0 && tool.overallLength > 3.0 * d)
            factor *= (std::max)(kMinStickoutFactor, std::pow(3.0 * d / tool.overallLength, 3.0));
        if (tool.chipLoad > 0.0)
        {
            setup.chipLoadTarget = tool.chipLoad * factor;
        }
        else if (programmedFeed > 0.0 && spindleRpm > 0.0)
        {
            setup.chipLoadTarget = programmedFeed / (spindleRpm * setup.teeth);
            setup.targetProgrammed = true;
        }
        return setup;
    }

    LoadBand BandOf(const MoveLoad& load, double limitPercent)
    {
        if (!load.measured)
            return LoadBand::NotCutting;
        if (!load.engaged)
            return LoadBand::Air;
        if (load.powerPercent > 100.0)
            return LoadBand::Overloaded;
        if (load.percent < 0.0)
            return LoadBand::NoTarget;
        if (load.percent > limitPercent)
            return LoadBand::Overloaded;
        return (load.percent > 100.0) ? LoadBand::Over : LoadBand::Under;
    }

    MoveLoad LoadOf(const ToolEngagement& engagement, const ToolLoadSetup& setup, double feed, double removed,
                    double length)
    {
        MoveLoad load;
        load.measured = true;
        load.feed = feed;
        load.engaged = engagement.engaged;
        load.plunge = engagement.plunge;
        load.angle = engagement.angle;
        load.depth = engagement.depth;
        load.width = engagement.width;
        load.effectiveDiameter = engagement.effectiveDiameter;
        load.removalRate = (length > 1e-9 && removed > 0.0) ? removed * feed / length : 0.0;
        if (!engagement.engaged)
            return load;

        const int teeth = setup.drill ? 1 : (std::max)(1, setup.teeth);
        if (setup.rpm > 0.0)
            load.feedPerTooth = feed / (setup.rpm * teeth);
        load.chip = load.feedPerTooth * engagement.thinning;
        if (setup.chipLoadTarget > 0.0)
            load.percent = 100.0 * load.chip / setup.chipLoadTarget;

        if (setup.cuttingForce > 0.0)
        {
            load.power = setup.cuttingForce * load.removalRate / 60.0 / 1000.0;     // N/mm^2 x mm^3/s is mW
            const double speed = kPi * engagement.effectiveDiameter / 1000.0 * setup.rpm / 60.0;     // m/s
            if (speed > 0.0)
                load.force = load.power / speed;
            if (setup.spindlePower > 0.0)
                load.powerPercent = 100.0 * load.power / setup.spindlePower;
        }
        return load;
    }

    void CombineLoad(MoveLoad& into, const MoveLoad& piece)
    {
        if (!piece.measured)
            return;
        if (!into.measured)
        {
            into = piece;
            return;
        }
        bool heavier;
        if (piece.engaged != into.engaged)
            heavier = piece.engaged;
        else if (std::fabs(piece.chip - into.chip) > 1e-12)
            heavier = piece.chip > into.chip;
        else
            heavier = piece.removalRate > into.removalRate;
        if (heavier)
            into = piece;
    }

    void AddLoad(OperationLoad& operation, const MoveLoad& load, double seconds, size_t move)
    {
        if (!load.engaged)
            return;
        const bool worse = !operation.any || ((load.percent >= 0.0) ? load.percent > operation.maxPercent
                                                                     : load.chip > operation.maxChip);
        if (worse)
            operation.worstMove = move;
        operation.any = true;
        operation.maxPercent = (std::max)(operation.maxPercent, load.percent);
        operation.maxChip = (std::max)(operation.maxChip, load.chip);
        operation.chipSeconds += load.chip * seconds;
        operation.engagedSeconds += seconds;
        if (!load.plunge)
        {
            operation.maxAngle = (std::max)(operation.maxAngle, load.angle);
            operation.maxDepth = (std::max)(operation.maxDepth, load.depth);
        }
        operation.maxRemovalRate = (std::max)(operation.maxRemovalRate, load.removalRate);
        operation.maxPower = (std::max)(operation.maxPower, load.power);
        operation.maxForce = (std::max)(operation.maxForce, load.force);
        operation.maxPowerPercent = (std::max)(operation.maxPowerPercent, load.powerPercent);
    }

    namespace
    {
        std::string ChipText(double chip, double percent, const ToolLoadSetup& setup)
        {
            const char* unit = setup.drill ? "mm/rev" : "mm/tooth";
            std::string text = Format("%.4g", chip);
            if (setup.chipLoadTarget > 0.0)
                text += Format(" of %.4g ", setup.chipLoadTarget) + unit +
                        (setup.targetProgrammed ? " programmed" : "") + Format(" (%.0f%%)", percent);
            else
                text += std::string(" ") + unit;
            return text;
        }

        std::string RateText(double rate, double power, double force, const ToolLoadSetup& setup)
        {
            std::string text = Format("%.3g cm3/min", rate / 1000.0);
            if (setup.cuttingForce > 0.0)
            {
                text += Format(", %.3g kW", power / 1000.0);
                if (setup.spindlePower > 0.0)
                    text += Format(" (%.0f%% of %.3g kW)", 100.0 * power / setup.spindlePower,
                                   setup.spindlePower / 1000.0);
                text += Format(", %.0f N", force);
            }
            return text;
        }
    }

    std::string MoveLoadText(const MoveLoad& load, const ToolLoadSetup& setup)
    {
        if (!load.measured)
            return "Not cutting";
        if (!load.engaged)
            return "Air";
        if (!(setup.rpm > 0.0))
            return "No spindle speed, so no chip load";
        if (setup.drill)
            return "Feed " + ChipText(load.chip, load.percent, setup) + ", " +
                   RateText(load.removalRate, load.power, load.force, setup);
        if (load.plunge)
            return "Plunge " + ChipText(load.chip, load.percent, setup) + ", " +
                   RateText(load.removalRate, load.power, load.force, setup);
        return "Chip " + ChipText(load.chip, load.percent, setup) +
               Format(", engaged %.0f deg, %.2f deep x %.2f wide, ", load.angle, load.depth, load.width) +
               RateText(load.removalRate, load.power, load.force, setup);
    }

    std::string OperationLoadText(const OperationLoad& load, const ToolLoadSetup& setup)
    {
        if (!load.any)
            return "No cutting yet.";
        const char* unit = setup.drill ? " mm/rev" : " mm/tooth";
        std::string text = "Highest ";
        if (load.maxPercent >= 0.0)
            text += Format("%.0f%% (", load.maxPercent) + Format("%.4g", load.maxChip) + unit + ")";
        else
            text += Format("%.4g", load.maxChip) + unit;
        text += ", average " + Format("%.4g", load.AverageChip()) + unit;
        if (load.maxAngle > 0.0)
            text += Format(", up to %.0f degrees and %.2f mm deep", load.maxAngle, load.maxDepth);
        text += ", up to " + RateText(load.maxRemovalRate, load.maxPower, load.maxForce, setup);
        return text;
    }
}
