// MotionTiming.cpp : see MotionTiming.h.

#include "MotionTiming.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace camcore
{
    namespace
    {
        constexpr double kHuge = std::numeric_limits<double>::max();
        constexpr double kPi = 3.14159265358979323846;

        const AxisLimit& Axis(const MotionLimits& limits, int i)
        {
            return i == 0 ? limits.x : (i == 1 ? limits.y : limits.z);
        }

        double Component(const Vec3& v, int i)
        {
            return i == 0 ? v.x : (i == 1 ? v.y : v.z);
        }
    }

    double StepVelocity(const Vec3& unitDirection, double feed, const MotionLimits& limits)
    {
        double v = (feed > 0.0) ? feed / 60.0 : kHuge;
        for (int i = 0; i < 3; ++i)
        {
            const double share = std::fabs(Component(unitDirection, i));
            if (share > 1e-9)
                v = (std::min)(v, Axis(limits, i).velocity / share);
        }
        return v;
    }

    double StepAcceleration(const Vec3& unitDirection, const MotionLimits& limits)
    {
        double a = kHuge;
        for (int i = 0; i < 3; ++i)
        {
            const double share = std::fabs(Component(unitDirection, i));
            if (share > 1e-9)
                a = (std::min)(a, Axis(limits, i).acceleration / share);
        }
        return a;
    }

    double TrapezoidSeconds(double length, double v0, double v1, double vmax, double a)
    {
        if (!(length > 0.0))
            return 0.0;
        if (!(vmax > 0.0))
            return 0.0;
        if (!(a > 0.0) || a >= kHuge)
            return length / vmax;
        v0 = (std::min)(v0, vmax);
        v1 = (std::min)(v1, vmax);
        // The highest speed reached speeding up from v0 then slowing to v1 over the length.
        const double peak = std::sqrt((std::max)(0.0, (2.0 * a * length + v0 * v0 + v1 * v1) / 2.0));
        if (peak <= vmax)
            return (std::max)(0.0, (peak - v0) / a) + (std::max)(0.0, (peak - v1) / a);
        const double up = (vmax * vmax - v0 * v0) / (2.0 * a);
        const double down = (vmax * vmax - v1 * v1) / (2.0 * a);
        return (vmax - v0) / a + (vmax - v1) / a + (std::max)(0.0, length - up - down) / vmax;
    }

    double RestToRestSecondsAt(double total, double vmax, double a, double distance)
    {
        if (!(total > 0.0) || !(vmax > 0.0))
            return 0.0;
        distance = (std::max)(0.0, (std::min)(total, distance));
        if (!(a > 0.0))
            return distance / vmax;
        // Speeding up for at most half the length, then cruising, then slowing down.
        const double ramp = (std::min)(total / 2.0, vmax * vmax / (2.0 * a));
        const double top = std::sqrt(2.0 * a * ramp);
        const double rampTime = top / a;
        const double cruise = total - 2.0 * ramp;
        if (distance <= ramp)
            return std::sqrt(2.0 * distance / a);
        if (distance <= ramp + cruise)
            return rampTime + (distance - ramp) / top;
        const double left = total - distance;     // still to go while slowing down
        const double all = 2.0 * rampTime + cruise / top;
        return all - std::sqrt(2.0 * left / a);
    }

    std::vector<double> TimeRun(const std::vector<MotionStep>& steps, const MotionLimits& limits)
    {
        std::vector<double> seconds(steps.size(), 0.0);

        // The steps that go somewhere.
        std::vector<size_t> real;
        std::vector<Vec3> directions;
        std::vector<double> lengths, velocities, accelerations;
        for (size_t i = 0; i < steps.size(); ++i)
        {
            const double length = Length(steps[i].machineVector);
            if (!(length > 1e-9))
                continue;
            const Vec3 direction = steps[i].machineVector * (1.0 / length);
            real.push_back(i);
            directions.push_back(direction);
            lengths.push_back(length);
            velocities.push_back(StepVelocity(direction, steps[i].feed, limits));
            accelerations.push_back(StepAcceleration(direction, limits));
        }
        const size_t n = real.size();
        if (n == 0)
            return seconds;

        // The fastest at each joint: 0 at the ends, otherwise the corner's blend.
        std::vector<double> joint(n + 1, 0.0);
        for (size_t k = 1; k < n; ++k)
        {
            const double cap = (std::min)(velocities[k - 1], velocities[k]);
            const double a = (std::min)(accelerations[k - 1], accelerations[k]);
            const double cosine = (std::max)(-1.0, (std::min)(1.0, Dot(directions[k - 1], directions[k])));
            const double turn = std::acos(cosine);
            if (turn < 1e-9)
                joint[k] = cap;
            else if (turn > kPi - 1e-6 || !(limits.blendTolerance > 0.0))
                joint[k] = 0.0;
            else
            {
                const double half = turn / 2.0;
                double radius = limits.blendTolerance / (1.0 / std::cos(half) - 1.0);
                radius = (std::min)(radius, (std::min)(lengths[k - 1], lengths[k]) / 2.0 / std::tan(half));
                joint[k] = (std::min)(cap, std::sqrt(a * radius));
            }
        }

        // Forward, then back: no joint faster than the step before or after lets it be.
        for (size_t k = 0; k < n; ++k)
            joint[k + 1] = (std::min)(joint[k + 1], std::sqrt(joint[k] * joint[k] + 2.0 * accelerations[k] * lengths[k]));
        for (size_t k = n; k-- > 0;)
            joint[k] = (std::min)(joint[k], std::sqrt(joint[k + 1] * joint[k + 1] + 2.0 * accelerations[k] * lengths[k]));

        for (size_t k = 0; k < n; ++k)
            seconds[real[k]] = TrapezoidSeconds(lengths[k], joint[k], joint[k + 1], velocities[k], accelerations[k]);
        return seconds;
    }
}
