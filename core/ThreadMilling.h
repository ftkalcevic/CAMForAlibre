// ThreadMilling.h : the Thread Milling operation's own settings. No Alibre or MFC code belongs here.
//
// Thread milling cuts internal threads with a thread mill on a helix, at each
// picked centre. Distances are in millimetres and angles in degrees.

#pragma once

#include <string>
#include <vector>

#include "CutLevels.h"  // CutGeometryLocation
#include "Facing.h"     // CutDirection

namespace camcore
{
    // Which way the thread winds.
    enum class ThreadHand
    {
        Right,
        Left
    };

    // The Thread Milling tab.
    struct ThreadMillingSettings
    {
        CutGeometryLocation location = CutGeometryLocation::AtTop;
        double topZ = 0.0;                              // Pick Top only

        double length = 10.0;                           // the thread's length, down from its top
        double majorDiameter = 6.0;                     // the nominal size: M6 is 6
        double pitch = 1.0;                             // filled from the tool when it's chosen
        ThreadHand hand = ThreadHand::Right;
        CutDirection direction = CutDirection::Climb;   // Climb or Conventional

        int radialPasses = 1;                           // equal steps out to the full depth
        bool springPass = false;                        // the last pass again
        double approachDistance = 1.0;
    };

    // The thread's depth on each side: 5/8 of the sharp V's height,
    // pitch / (2 tan(angle / 2)). For 60 degrees, 0.5413 times the pitch.
    double ThreadDepth(double pitch, double threadAngle);

    // The internal thread's minor diameter: the major less twice the depth.
    double ThreadMinorDiameter(const ThreadMillingSettings& settings, double threadAngle);

    // Returns a sentence for each problem. Empty means the settings are valid.
    std::vector<std::string> ValidateThreadMilling(const ThreadMillingSettings& settings);
}
