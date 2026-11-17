// Rotary.h : frames of Set Origin and Rotate steps, and indexing a 4th axis. No Alibre or MFC code belongs here.
//
// The job has one work zero, its first Set Origin, which the operator sets on the
// rotary axis. The axis runs through it, along the work zero's X or Y as the
// machine says (-X, +X, -Y, +Y). A positive angle turns the work so the tool, seen
// from the work, turns right-handed about that direction, as a positive A turns
// about +X. So a frame turned by an angle about the axis is the one the machine
// cuts square to when its rotary axis is at that angle.
//
// Each operation's moves are kept in its origin's frame. The machine is given
// them from the work zero, turned back by the angle (MachinePoint), so they're
// plain 3 axis moves with the rotary axis standing still.

#pragma once

#include <array>
#include <string>
#include <vector>

#include "CamJob.h"
#include "Machine.h"
#include "PseudoPost.h"     // PostOperation

namespace camcore
{
    // The rotary axis's direction in the work zero's own axes.
    Vec3 RotaryDirection(RotaryAxis axis);

    // v turned about a unit axis by an angle in degrees, right-handed.
    Vec3 RotateAbout(const Vec3& v, const Vec3& axis, double degrees);

    // What frames and indexes are worked out with.
    struct OrientationContext
    {
        int axes = 3;                               // the machine's: 3, or 4 with a rotary axis
        RotaryAxis rotary = RotaryAxis::PlusY;
        const std::vector<Vec3>* stockPoints = nullptr;     // the stock's corners or facets, in assembly mm; null for none
    };

    // The machine's axes, without the stock.
    OrientationContext OrientationFor(const Machine& machine);

    // Set Origin and Rotate steps set the frame the operations below them use.
    bool IsOrientationStep(StepType type);

    // The frame an orientation step sets. A Set Origin's comes from its picks; a
    // Rotate's is the work zero's turned about the rotary axis, moved to the top
    // of the stock if it asks. Fails with a reason, such as a Rotate on a 3 axis
    // machine or with no work zero.
    FrameResult OrientationFrame(const Job& job, size_t index, const OrientationContext& context);

    // The frame a step runs under: its own if it's an orientation step, otherwise
    // that of the nearest unsuppressed one above it.
    FrameResult FrameAt(const Job& job, size_t stepIndex, const OrientationContext& context);

    // A step's index settings: a Rotate's or a Set Origin's. Default ones for others.
    const IndexSettings& IndexSettingsOf(const SequenceStep& step);

    struct IndexSolution
    {
        bool ok = false;
        double angle = 0.0;         // 0 to 360
        std::string message;        // why not, when not ok
    };

    // The rotary angle at which a frame's Z is the machine's (the work zero's Z).
    // A 3 axis machine reaches only frames along the work zero's Z, at 0.
    IndexSolution SolveIndex(const Frame& workZero, const OrientationContext& context, const Frame& frame);

    // The rotary angle (0 to 360) at which the tool, along toolAxis in the
    // assembly, is upright on the machine; for the simulation turning the view.
    // Not ok when the tool axis isn't square to the rotary axis.
    IndexSolution RotaryAngleOf(const Frame& workZero, RotaryAxis rotary, const Vec3& toolAxis);

    // A turn of degrees (right-handed) about the line through point along
    // direction, as a 4 x 4 row-major matrix for row vectors (p' = p M), the
    // translation in row 4, as Alibre's transforms are.
    typedef std::array<double, 16> RowMatrix;
    RowMatrix TurnAboutLine(const Vec3& point, const Vec3& direction, double degrees);
    RowMatrix MultiplyRows(const RowMatrix& a, const RowMatrix& b);    // a then b
    Vec3 TransformPoint(const Vec3& p, const RowMatrix& m);

    // An assembly point as the machine sees it with the rotary axis at an angle:
    // from the work zero, turned back about the axis. Directions likewise.
    Vec3 MachinePoint(const Frame& workZero, RotaryAxis rotary, double angle, const Vec3& world);
    Vec3 MachineDirection(const Frame& workZero, RotaryAxis rotary, double angle, const Vec3& world);

    // The other way: where a machine point is in the assembly at an angle.
    Vec3 WorldFromMachine(const Frame& workZero, RotaryAxis rotary, double angle, const Vec3& machine);

    // The angle to write going from current (any value, as last written) to
    // target (0 to 360) the given way. Shortest takes +180 over -180.
    double NextIndexAngle(double current, double target, IndexDirection direction);

    // How far the furthest of the points is from the rotary axis, 0 for none.
    double FurthestFromAxis(const Frame& workZero, RotaryAxis rotary, const std::vector<Vec3>& world);

    // The rotary axis's direction in the assembly.
    Vec3 RotaryAxisDirection(const Frame& workZero, RotaryAxis rotary);

    // For an operation that can't be generated: a sentence when some of its
    // picked flat faces or circular edges are edge-on to its frame's Z (their
    // normal or axis within about 3 degrees of square to it), as when a Rotate's
    // angle (from the work zero) isn't the one their side needs. Empty otherwise.
    std::string EdgeOnFeaturesText(const std::vector<PickedGeometry>& features, const Frame& frame);

    // The turn before one operation.
    struct IndexMove
    {
        bool turn = false;          // false when it's already at its angle
        double from = 0.0;          // the angle as last written
        double to = 0.0;            // the angle written, past 360 or below 0 to turn the asked way
        double retract = 0.0;       // the height above the rotary axis the tool goes up to first
    };

    struct IndexPlan
    {
        bool ok = false;
        std::string unreachable;        // the operations the machine can't reach, a line each
        std::vector<double> angles;     // each operation's angle, 0 to 360
        std::vector<IndexMove> moves;   // the turn before each operation
        double automaticRetract = 0.0;  // the stock's and toolpaths' furthest from the axis, plus kIndexClearance
    };

    // The rotary angle of each operation in order and the turns between them, from
    // 0 at the start (the operator zeroes the axis). Each turn goes up to the height
    // its operation's Set Origin or Rotate asks, or the automatic one, and turns the
    // way it asks. With 3 axes every operation must be along the work zero's Z.
    IndexPlan PlanIndexes(const Frame& workZero, const OrientationContext& context,
                          const std::vector<PostOperation>& operations);
}
