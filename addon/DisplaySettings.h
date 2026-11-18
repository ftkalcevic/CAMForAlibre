// DisplaySettings.h : how CAM graphics look in the view.
//
// These become program settings later.

#pragma once

struct DisplayColour
{
    long red;
    long green;
    long blue;
    long alpha;     // 255 is opaque
};

// The active origin's XY plane, drawn as an outline with grid lines inside.
const DisplayColour kOriginPlaneColour = { 0, 120, 255, 255 };
const DisplayColour kOriginGridColour = { 0, 120, 255, 96 };    // alpha may be ignored
const float kOriginOutlineLineWeight = 1.5f;
const float kOriginGridLineWeight = 1.0f;

// About this many grid squares along the plane's longer side.
const int kOriginGridDivisions = 10;

// The plane reaches past the parts by this fraction of their size.
const double kOriginPlaneMargin = 0.1;

const DisplayColour kOriginXColour = { 230, 40, 40, 255 };
const DisplayColour kOriginYColour = { 40, 200, 40, 255 };
const DisplayColour kOriginZColour = { 40, 90, 255, 255 };

const double kOriginAxisLengthMm = 25.0;

// The machine's rotary axis, through the work zero, drawn with the origin on a 4 axis machine.
const DisplayColour kRotaryAxisColour = { 230, 140, 30, 255 };
const float kRotaryAxisLineWeight = 1.5f;
const float kOriginLineWeight = 3.0f;

// Toolpaths. The canvas has no dashed lines, so rapids differ by colour only.
const DisplayColour kRapidColour = { 220, 40, 40, 255 };
const DisplayColour kFeedColour = { 30, 110, 230, 255 };
const DisplayColour kArcColour = { 210, 0, 210, 255 };      // arc feeds, so arcs stand out
const float kToolpathLineWeight = 1.5f;
const float kSelectedToolpathLineWeight = 3.0f;
const DisplayColour kSuppressedColour = { 150, 150, 150, 255 };   // suppressed operations' toolpaths
const float kSuppressedToolpathLineWeight = 0.75f;

// A facing operation's boundary, as Generate found it.
const DisplayColour kBoundaryColour = { 255, 150, 0, 255 };
const float kBoundaryLineWeight = 2.0f;

// The chosen operation's picked features.
const DisplayColour kFeatureHighlightColour = { 255, 150, 0, 255 };
const float kFeatureHighlightLineWeight = 3.0f;
const float kCutLevelLineWeight = 1.5f;     // the same outline at the top and bottom of the cut

// The simulated stock, shaded, the tool, and a rapid into the stock.
const DisplayColour kSimStockColour = { 120, 160, 200, 255 };
const DisplayColour kSimToolColour = { 230, 190, 0, 255 };
const float kSimToolLineWeight = 1.5f;
const DisplayColour kSimHolderColour = { 150, 150, 160, 255 };     // the tool holder, grey
const DisplayColour kSimWarningColour = { 230, 0, 0, 255 };

// Comparing with the parts: the parts' cells, stock left that should have been cut,
// and part material that was cut away.
const DisplayColour kSimPartColour = { 110, 190, 120, 255 };
const DisplayColour kSimNotEnoughColour = { 245, 170, 30, 255 };
const DisplayColour kSimTooMuchColour = { 220, 20, 20, 255 };
const float kSimWarningLineWeight = 4.0f;

// The simulated toolpath coloured by the tool's load: air, at most the chip load,
// over it, over the stop limit, and no chip load on the tool.
const DisplayColour kSimLoadAirColour = { 160, 160, 160, 255 };
const DisplayColour kSimLoadUnderColour = { 0, 170, 60, 255 };
const DisplayColour kSimLoadOverColour = { 240, 160, 0, 255 };
const DisplayColour kSimLoadOverloadedColour = { 220, 0, 0, 255 };
const DisplayColour kSimLoadNoTargetColour = { 60, 120, 230, 255 };
const float kSimLoadLineWeight = 2.5f;

// The stock, drawn as its edges.
const DisplayColour kStockColour = { 0, 160, 160, 255 };
const float kStockLineWeight = 1.5f;
const DisplayColour kFixtureColour = { 170, 60, 200, 255 };     // a Fixtures step's parts, purple
