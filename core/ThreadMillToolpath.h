// ThreadMillToolpath.h : calculates a Thread Milling operation's toolpath. No Alibre or MFC code belongs here.
//
// Points are in the operation's origin frame, in millimetres.

#pragma once

#include "CamJob.h"
#include "ToolLibrary.h"
#include "Toolpath.h"

namespace camcore
{
    // Everything a thread milling toolpath is worked out from.
    struct ThreadMillToolpathInput
    {
        const SequenceStep* operation = nullptr;
        Frame frame;                    // the origin the operation runs under
        const Tool* tool = nullptr;     // in millimetres, from ToolInMillimetres

        bool havePartMaxZ = false;      // the parts' highest point along the frame's Z
        double partMaxZ = 0.0;
        bool haveStockMaxZ = false;     // the stock's, when there's a stock
        double stockMaxZ = 0.0;
        bool haveFixtureMaxZ = false;   // the active fixtures' highest point plus clearance
        double fixtureMaxZ = 0.0;
    };

    // Where each thread is: its centre and its top and bottom along the frame's Z.
    struct ThreadHole
    {
        Vec3 centre;        // at the top
        double bottom = 0.0;
    };

    // The threads, in the Features list's order: centred on each circle, hole
    // face or point, with the top at the feature's height (At Top), the length
    // above it (At Bottom) or the Pick Top Z, and the bottom the length down.
    // Empty with a problem if a feature can't be used.
    std::vector<ThreadHole> ThreadHoles(const SequenceStep& operation, const Frame& frame, std::string& problem);

    // Internal threads, with a thread mill whose diameter is its teeth's tips.
    // The tool's centre goes round at the major radius less the tool's radius,
    // after radial passes stepping evenly out from the minor radius (and the
    // last again with Spring Pass). Climb turns anticlockwise and Conventional
    // clockwise (a clockwise spindle); a right-hand thread rises anticlockwise,
    // so a right-hand climb cut goes from the bottom up, and left hand the other
    // way. Each turn moves one pitch along. A single point tool (one row of
    // teeth) helixes the whole length; a tool with n rows cuts n pitches a turn,
    // in single turns moved whole pitches apart, the lowest with its bottom
    // tooth at the thread's bottom and the highest ending within a pitch above
    // its top. Each turn starts with a half circle from the centre out to the
    // wall, tangent there, along a quarter pitch, and ends with one back in.
    // The tool comes across at the clearance plane, down at rapid to the
    // approach distance above the top, and feeds down the centre; it goes up
    // the centre at rapid. A multi-row tool whose pitch isn't the thread's gives
    // a warning.
    ToolpathResult GenerateThreadMillToolpath(const ThreadMillToolpathInput& input);
}
