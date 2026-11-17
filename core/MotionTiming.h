// MotionTiming.h : how long the machine takes over moves, from each axis's
// velocity and acceleration, blending corners within a tolerance as LinuxCNC's
// G64 P does. For the simulation's times only. No Alibre or MFC code belongs here.

#pragma once

#include <vector>

#include "Machine.h"
#include "Origin.h"     // Vec3

namespace camcore
{
    // Linear axes' velocities are mm/s, feeds mm/min.
    // A straight move as the machine sees it: its vector in the machine's axes
    // (mm), and its feed (mm/min), 0 for a rapid (each axis at its own maximum).
    struct MotionStep
    {
        Vec3 machineVector;
        double feed = 0.0;
    };

    // The seconds each step takes, a run that starts and ends at rest. Between
    // steps the tool doesn't stop: it rounds the corner on an arc that leaves the
    // path by no more than the blend tolerance (and uses no more than half of
    // either step), at the speed its acceleration allows there. Steps of no
    // length take no time and don't break the run.
    std::vector<double> TimeRun(const std::vector<MotionStep>& steps, const MotionLimits& limits);

    // The fastest a straight move can go along its direction (mm/s), and its
    // acceleration (mm/s^2), from the axes it moves and its feed (0: none).
    double StepVelocity(const Vec3& unitDirection, double feed, const MotionLimits& limits);
    double StepAcceleration(const Vec3& unitDirection, const MotionLimits& limits);

    // Seconds over a length entering at v0 and leaving at v1, no faster than
    // vmax, speeding up and slowing down at a (all in mm and seconds).
    double TrapezoidSeconds(double length, double v0, double v1, double vmax, double a);

    // Starting and ending at rest over total: the time at which distance is reached.
    double RestToRestSecondsAt(double total, double vmax, double a, double distance);
}
