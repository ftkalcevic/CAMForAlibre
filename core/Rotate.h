// Rotate.h : the settings of Rotate steps and of indexing the rotary axis. No Alibre or MFC code belongs here.
//
// A Rotate step turns the work about the machine's rotary (4th) axis, which runs
// through the work zero (the first Set Origin), to an angle from the work zero.
// The operations below it are cut in the turned frame. A Set Origin can turn the
// work too: its Z is reached by turning about the rotary axis. Either kind of
// step says how the machine gets there: how high it retracts first, and which
// way it turns. The maths is in Rotary.h. Lengths are in millimetres, angles in
// degrees.

#pragma once

#include <string>
#include <vector>

namespace camcore
{
    // Which way the rotary axis turns to a new angle.
    enum class IndexDirection
    {
        Shortest,   // the shorter way round
        Positive,   // the angle only ever goes up
        Negative    // the angle only ever goes down
    };

    // How the machine indexes to a Rotate or Set Origin step's angle.
    struct IndexSettings
    {
        // Retract to this height above the rotary axis before turning, or, when
        // automatic, to the stock's furthest point from the axis (or the toolpaths',
        // if further) plus kIndexClearance.
        bool automaticRetract = true;
        double retractHeight = 50.0;
        IndexDirection direction = IndexDirection::Shortest;
    };

    // How far clear of the furthest stock the tool retracts when it's automatic.
    constexpr double kIndexClearance = 5.0;

    // Where the origin goes after a Rotate step turns the work.
    enum class OriginAfterRotate
    {
        Keep,           // the work zero, turned: still on the rotary axis
        TopOfStock      // that, moved along the turned Z to the stock's highest point
    };

    struct RotateSettings
    {
        double angle = 90.0;    // from the work zero, about the rotary axis, -360 to 360
        OriginAfterRotate originAfter = OriginAfterRotate::Keep;
    };

    // Returns a sentence for each problem. Empty means the settings are valid.
    std::vector<std::string> ValidateRotate(const RotateSettings& settings);
    std::vector<std::string> ValidateIndex(const IndexSettings& settings);

    // "Shortest", "Positive", "Negative", as saved.
    std::string IndexDirectionText(IndexDirection direction);
    bool IndexDirectionFromText(const std::string& text, IndexDirection& direction);
}
