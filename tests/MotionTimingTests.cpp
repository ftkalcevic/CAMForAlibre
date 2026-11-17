// MotionTimingTests.cpp : tests for timing moves by the machine's axis limits.

#include <doctest/doctest.h>

#include <cmath>

#include "MotionTiming.h"
#include "Rotary.h"
#include "Simulation.h"

using namespace camcore;

namespace
{
    bool Near3(const Vec3& a, const Vec3& b)
    {
        return Length(a - b) < 1e-9;
    }

    MotionLimits Limits(double velocity, double acceleration, double tolerance = 0.1)
    {
        MotionLimits limits;
        limits.x = limits.y = limits.z = { velocity, acceleration };
        limits.blendTolerance = tolerance;
        return limits;
    }
}

TEST_CASE("a long move cruises, a short one never reaches its speed")
{
    // 100 mm/s at 500 mm/s^2 takes 0.2 s and 10 mm to get there.
    const MotionLimits limits = Limits(100, 500);
    std::vector<double> t = TimeRun({ { { 100, 0, 0 }, 0.0 } }, limits);
    CHECK(t[0] == doctest::Approx(100.0 / 100.0 + 100.0 / 500.0));

    // 4 mm: a triangle, up for 2 mm and down for 2.
    t = TimeRun({ { { 4, 0, 0 }, 0.0 } }, limits);
    CHECK(t[0] == doctest::Approx(2.0 * std::sqrt(2.0 * 2.0 / 500.0)));

    // A feed slower than the axis.
    t = TimeRun({ { { 100, 0, 0 }, 600.0 } }, Limits(100, 1e12));
    CHECK(t[0] == doctest::Approx(10.0));
}

TEST_CASE("a diagonal is held by the slower axis, and a rapid runs each axis at its own maximum")
{
    MotionLimits limits = Limits(100, 1e12);
    limits.y.velocity = 50;
    // Along X and Y equally: Y's share of the speed is 1/sqrt 2, so the path goes 50 sqrt 2.
    const std::vector<double> t = TimeRun({ { { 100, 100, 0 }, 0.0 } }, limits);
    CHECK(t[0] == doctest::Approx(100.0 / 50.0));
    // A feed above that is held too.
    const std::vector<double> fed = TimeRun({ { { 100, 100, 0 }, 100000.0 } }, limits);
    CHECK(fed[0] == doctest::Approx(t[0]));
    CHECK(StepVelocity({ 1, 0, 0 }, 0.0, limits) == doctest::Approx(100.0));
    CHECK(StepVelocity({ 0, 1, 0 }, 0.0, limits) == doctest::Approx(50.0));
}

TEST_CASE("corners are blended within the tolerance, and stopped at with none")
{
    const std::vector<MotionStep> corner = { { { 50, 0, 0 }, 0.0 }, { { 0, 50, 0 }, 0.0 } };
    const double stopped = TimeRun(corner, Limits(100, 500, 0.0))[0] + TimeRun(corner, Limits(100, 500, 0.0))[1];
    // Stopping: each is a rest to rest move.
    CHECK(stopped == doctest::Approx(2.0 * TrapezoidSeconds(50, 0, 0, 100, 500)));
    const std::vector<double> blended = TimeRun(corner, Limits(100, 500, 0.1));
    CHECK(blended[0] + blended[1] < stopped);
    // A bigger tolerance is quicker still.
    const std::vector<double> looser = TimeRun(corner, Limits(100, 500, 1.0));
    CHECK(looser[0] + looser[1] < blended[0] + blended[1]);

    // Turning straight back always stops.
    const std::vector<MotionStep> back = { { { 50, 0, 0 }, 0.0 }, { { -50, 0, 0 }, 0.0 } };
    const std::vector<double> reversed = TimeRun(back, Limits(100, 500, 1.0));
    CHECK(reversed[0] + reversed[1] == doctest::Approx(stopped));

    // Steps of no length take no time and don't stop it.
    const std::vector<MotionStep> straight = { { { 50, 0, 0 }, 0.0 }, { { 0, 0, 0 }, 0.0 }, { { 50, 0, 0 }, 0.0 } };
    const std::vector<double> through = TimeRun(straight, Limits(100, 500));
    CHECK(through[1] == 0.0);
    CHECK(through[0] + through[2] == doctest::Approx(TrapezoidSeconds(100, 0, 0, 100, 500)));
}

TEST_CASE("chords of a gentle arc keep nearly full speed")
{
    // A 50 mm radius quarter circle in 1 degree chords at 3000 mm/min (50 mm/s):
    // the centripetal limit sqrt(a R) is 158 mm/s, so only the ends slow it.
    std::vector<MotionStep> chords;
    const double r = 50.0, pi = 3.14159265358979323846;
    for (int i = 0; i < 90; ++i)
    {
        const double a0 = i * pi / 180.0, a1 = (i + 1) * pi / 180.0;
        chords.push_back({ { r * (std::cos(a1) - std::cos(a0)), r * (std::sin(a1) - std::sin(a0)), 0 }, 3000.0 });
    }
    double total = 0.0, length = 0.0;
    for (size_t i = 0; i < chords.size(); ++i)
        length += Length(chords[i].machineVector);
    for (double t : TimeRun(chords, Limits(100, 500, 0.01)))
        total += t;
    CHECK(total == doctest::Approx(TrapezoidSeconds(length, 0, 0, 50, 500)).epsilon(0.01));
}

TEST_CASE("rest to rest: the time at each distance")
{
    // 100 mm/s, 500 mm/s^2, 100 mm: up over 10 mm in 0.2 s, cruise 80 mm in 0.8 s, down in 0.2 s.
    CHECK(RestToRestSecondsAt(100, 100, 500, 0) == 0.0);
    CHECK(RestToRestSecondsAt(100, 100, 500, 10) == doctest::Approx(0.2));
    CHECK(RestToRestSecondsAt(100, 100, 500, 50) == doctest::Approx(0.6));
    CHECK(RestToRestSecondsAt(100, 100, 500, 100) == doctest::Approx(1.2));
    CHECK(RestToRestSecondsAt(100, 100, 500, 100) == doctest::Approx(TrapezoidSeconds(100, 0, 0, 100, 500)));
    // Too short to cruise.
    CHECK(RestToRestSecondsAt(4, 100, 500, 4) == doctest::Approx(TrapezoidSeconds(4, 0, 0, 100, 500)));
    CHECK(RestToRestSecondsAt(4, 100, 500, 2) == doctest::Approx(TrapezoidSeconds(4, 0, 0, 100, 500) / 2.0));
}

TEST_CASE("the simulated turn of the rotary axis takes its velocity and acceleration")
{
    // Two operations, the second turned 180 about Y: the turn's steps together
    // take the whole turn's time, 180 degrees at 90 degrees/s, 180 degrees/s^2.
    SimOperation first, second;
    Tool tool;
    tool.type = ToolType::FlatEndMill;
    tool.diameter = 4;
    tool.fluteLength = 10;
    tool.overallLength = 40;
    first.profile = second.profile = ProfileOf(tool);
    ToolpathMove a;
    a.kind = MoveKind::Rapid;
    a.to = { 0, 0, 30 };
    first.toolpath.moves.push_back(a);
    second.frame.zAxis = { 0, 0, -1 };
    second.frame.xAxis = { -1, 0, 0 };
    second.toolpath.moves.push_back(a);
    second.index.present = true;
    second.index.axisDirection = { 0, 1, 0 };
    second.index.from = 0;
    second.index.to = 180;
    second.index.retract = 30;
    MotionLimits limits;
    limits.rotaryVelocity = 90;
    limits.rotaryAcceleration = 180;
    Simulation simulation;
    simulation.Start({ { -20, -20, -20 }, { 20, 20, 20 } }, {}, { first, second }, SimResolution{ true, 1.0 }, limits);
    double turn = 0.0;
    int steps = 0;
    for (const SimMove& move : simulation.Moves())
    {
        if (move.index)
        {
            turn += move.seconds;
            ++steps;
        }
    }
    CHECK(steps == 90);
    // It slows down as it sped up: the steps' times mirror each other.
    std::vector<double> times;
    for (const SimMove& move : simulation.Moves())
    {
        if (move.index)
            times.push_back(move.seconds);
    }
    for (size_t i = 0; i < times.size(); ++i)
        CHECK(times[i] == doctest::Approx(times[times.size() - 1 - i]));
    CHECK(times.front() > 2.0 * times[times.size() / 2]);
    CHECK(times.back() > 2.0 * times[times.size() / 2]);
    // Part way through a step the tool's axis is part way round, not at the step's end.
    simulation.Rewind();
    size_t firstTurn = 0;
    while (!simulation.Moves()[firstTurn].index)
        ++firstTurn;
    simulation.Step(firstTurn);
    const Vec3 before = simulation.ToolAxis();
    CHECK(Near3(before, { 0, 0, 1 }));
    simulation.Advance(simulation.Moves()[firstTurn].seconds / 2.0);
    const Vec3 half = simulation.ToolAxis();
    const double turned = std::acos((std::max)(-1.0, (std::min)(1.0, Dot(half, before)))) * 180.0 / 3.14159265358979323846;
    CHECK(turned == doctest::Approx(1.0).epsilon(1e-6));
    MESSAGE("turn step seconds: first " << times[0] << ", " << times[1] << ", middle " << times[45] << ", last "
                                        << times[times.size() - 2] << ", " << times.back());
    // Up to 90 degrees/s in 0.5 s over 22.5 degrees, cruise 135 degrees in 1.5 s, down in 0.5 s.
    CHECK(turn == doctest::Approx(2.5));
}
