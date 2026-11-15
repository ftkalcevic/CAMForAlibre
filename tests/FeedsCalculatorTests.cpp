// FeedsCalculatorTests.cpp : tests for the feeds and speeds calculator.

#include <doctest/doctest.h>

#include <cmath>

#include "FeedsCalculator.h"

using namespace camcore;

namespace
{
    const double kPi = 3.14159265358979323846;

    FeedsCalcInput Input(ToolType type = ToolType::FlatEndMill)
    {
        FeedsCalcInput input;
        input.tool.type = type;
        input.tool.diameter = 6.0;
        input.tool.fluteLength = 18.0;
        input.tool.overallLength = 18.0;     // 3 diameters: no stickout factor
        input.tool.fluteCount = 2;
        input.tool.cornerRadius = 1.0;
        input.tool.chipLoad = 0.05;
        input.cutting.set = true;
        input.cutting.surfaceSpeed = 300.0;
        input.cutting.chipLoadFactor = 100.0;
        input.depth = 6.0;
        input.width = 3.0;                    // half the diameter: no thinning
        return input;
    }
}

TEST_CASE("the spindle speed comes from the surface speed, limited by the machine")
{
    FeedsCalcInput input = Input();
    FeedsCalcResult result = CalculateFeeds(input);
    REQUIRE(result.problems.empty());
    const double rpm = 300.0 * 1000.0 / (kPi * 6.0);
    CHECK(result.rpm == doctest::Approx(rpm));
    CHECK_FALSE(result.rpmLimited);
    CHECK(result.feeds.spindleRpm == std::round(rpm));
    CHECK(result.feeds.cut == std::round(rpm * 2 * 0.05));

    input.maxSpindleRpm = 12000.0;
    result = CalculateFeeds(input);
    CHECK(result.rpmLimited);
    CHECK(result.rpmWanted == doctest::Approx(rpm));
    CHECK(result.feeds.spindleRpm == 12000.0);
    CHECK(result.feeds.cut == std::round(12000.0 * 2 * 0.05));
}

TEST_CASE("the other feeds are fixed parts of the cutting feed, and transfer is kept")
{
    FeedsCalcInput input = Input();
    input.start.transferAtRapid = false;
    input.start.transfer = 1234.0;
    const FeedsCalcResult result = CalculateFeeds(input);
    const double cut = result.feeds.cut;
    CHECK(result.feeds.plunge == doctest::Approx(std::round(result.rpm * 2 * 0.05 * kPlungeRatio)));
    CHECK(result.feeds.engage == doctest::Approx(std::round(result.rpm * 2 * 0.05 * kEngageRatio)));
    CHECK(result.feeds.approach == cut);
    CHECK(result.feeds.retract == cut);
    CHECK(result.feeds.departure == cut);
    CHECK_FALSE(result.feeds.transferAtRapid);
    CHECK(result.feeds.transfer == 1234.0);
}

TEST_CASE("a ball mill cutting shallow spins faster")
{
    FeedsCalcInput input = Input(ToolType::BallMill);
    input.depth = 0.5;
    const FeedsCalcResult result = CalculateFeeds(input);
    REQUIRE(result.problems.empty());
    const double expected = 2.0 * std::sqrt(0.5 * (6.0 - 0.5));     // the chord at 0.5 deep in a 6 mm ball
    CHECK(result.effectiveDiameter == doctest::Approx(expected).epsilon(0.01));
    CHECK(result.rpm == doctest::Approx(300.0 * 1000.0 / (kPi * expected)).epsilon(0.01));

    input.depth = 4.0;      // past the ball
    CHECK(CalculateFeeds(input).effectiveDiameter == doctest::Approx(6.0));
}

TEST_CASE("a narrow cut thins the chips, and the feed rises to make up, up to a limit")
{
    FeedsCalcInput input = Input();
    input.width = 0.6;      // a tenth of the diameter
    FeedsCalcResult result = CalculateFeeds(input);
    CHECK(result.thinningFactor == doctest::Approx(6.0 / (2.0 * std::sqrt(0.6 * 5.4))));
    CHECK(result.chipLoad == doctest::Approx(0.05 * result.thinningFactor));

    input.width = 0.01;
    result = CalculateFeeds(input);
    CHECK(result.thinningFactor == doctest::Approx(kMaxThinning));

    input.width = 6.0;      // full width
    CHECK(CalculateFeeds(input).thinningFactor == 1.0);
}

TEST_CASE("a long tool feeds slower")
{
    FeedsCalcInput input = Input();
    input.tool.overallLength = 36.0;     // 6 diameters
    FeedsCalcResult result = CalculateFeeds(input);
    CHECK(result.stickoutFactor == doctest::Approx(kMinStickoutFactor));     // (1/2)^3 is under the floor

    input.tool.overallLength = 21.0;
    result = CalculateFeeds(input);
    CHECK(result.stickoutFactor == doctest::Approx(std::pow(18.0 / 21.0, 3.0)));
}

TEST_CASE("the material's factor and the aggression scale the chip load")
{
    FeedsCalcInput input = Input();
    input.cutting.chipLoadFactor = 50.0;
    input.aggression = 1.5;
    const FeedsCalcResult result = CalculateFeeds(input);
    CHECK(result.chipLoad == doctest::Approx(0.05 * 0.5 * 1.5));

    input.chipLoad = 0.1;   // given in the calculator, in place of the tool's
    CHECK(CalculateFeeds(input).chipLoad == doctest::Approx(0.1 * 0.5 * 1.5));
}

TEST_CASE("a drill's chip load is per revolution, and it plunges at the cutting feed")
{
    FeedsCalcInput input = Input(ToolType::Drill);
    input.tool.chipLoad = 0.1;
    input.width = 0.0;      // not needed for drills
    input.cutting.surfaceSpeed = 30.0;
    const FeedsCalcResult result = CalculateFeeds(input);
    REQUIRE(result.problems.empty());
    const double rpm = 30.0 * 1000.0 / (kPi * 6.0);
    CHECK(result.feeds.cut == std::round(rpm * 0.1));
    CHECK(result.feeds.plunge == result.feeds.cut);
    CHECK(result.thinningFactor == 1.0);
}

TEST_CASE("missing values are reported and nothing is worked out")
{
    FeedsCalcInput input = Input();
    input.cutting.set = false;
    input.tool.chipLoad = 0.0;
    input.width = 0.0;
    input.start.cut = 777.0;
    const FeedsCalcResult result = CalculateFeeds(input);
    CHECK(result.problems.size() == 3);
    CHECK(result.feeds.cut == 777.0);
}

TEST_CASE("every typical material has a speed for each tool material")
{
    for (const TypicalMaterial& material : TypicalMaterials())
    {
        CHECK(material.chipLoadFactor > 0.0);
        for (double speed : material.surfaceSpeed)
            CHECK(speed > 0.0);
    }
}
