// ThreadMilling.cpp : the Thread Milling operation's own settings.

#include "ThreadMilling.h"

#include <cmath>

namespace camcore
{
    double ThreadDepth(double pitch, double threadAngle)
    {
        const double kPi = 3.14159265358979323846;
        const double half = threadAngle / 2.0 * kPi / 180.0;
        if (!(pitch > 0.0) || !(half > 0.0) || !(half < kPi / 2.0))
            return 0.0;
        return 0.625 * pitch / (2.0 * std::tan(half));
    }

    double ThreadMinorDiameter(const ThreadMillingSettings& settings, double threadAngle)
    {
        return settings.majorDiameter - 2.0 * ThreadDepth(settings.pitch, threadAngle);
    }

    std::vector<std::string> ValidateThreadMilling(const ThreadMillingSettings& settings)
    {
        std::vector<std::string> problems;

        if (settings.location == CutGeometryLocation::PickTop && !std::isfinite(settings.topZ))
            problems.push_back("The top Z must be a number.");
        if (!(settings.length > 0.0))
            problems.push_back("The thread length must be more than 0.");
        if (!(settings.majorDiameter > 0.0))
            problems.push_back("The major diameter must be more than 0.");
        if (!(settings.pitch > 0.0))
            problems.push_back("The pitch must be more than 0.");
        if (settings.direction == CutDirection::Mixed)
            problems.push_back("Thread milling cuts Climb or Conventional.");
        if (settings.radialPasses < 1)
            problems.push_back("The number of radial passes must be at least 1.");
        if (!(settings.approachDistance >= 0.0))
            problems.push_back("The approach distance can't be negative.");

        return problems;
    }
}
