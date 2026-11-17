// Rotary.cpp : frames of Set Origin and Rotate steps, and indexing a 4th axis.

#include "Rotary.h"

#include <algorithm>
#include <cmath>

#include "Features.h"   // WorldToFrame
#include "Stock.h"      // HighestAlongZ

namespace camcore
{
    namespace
    {
        const double kPi = 3.14159265358979323846;

        // A frame's Z within this of the machine's (about 0.08 degrees) is reached.
        const double kReachTolerance = 1e-6;

        // Turns a frame's axes, and its origin, about a line through a point.
        Frame RotateFrame(const Frame& frame, const Vec3& through, const Vec3& axis, double degrees)
        {
            Frame turned;
            turned.origin = through + RotateAbout(frame.origin - through, axis, degrees);
            turned.xAxis = RotateAbout(frame.xAxis, axis, degrees);
            turned.yAxis = RotateAbout(frame.yAxis, axis, degrees);
            turned.zAxis = RotateAbout(frame.zAxis, axis, degrees);
            return turned;
        }

        Vec3 WorldAxis(const Frame& workZero, RotaryAxis rotary)
        {
            return RotaryAxisDirection(workZero, rotary);
        }

        FrameResult Failure(const std::string& message)
        {
            FrameResult result;
            result.message = message;
            return result;
        }

        const IndexSettings kDefaultIndex;
    }

    std::vector<std::string> ValidateRotate(const RotateSettings& settings)
    {
        std::vector<std::string> problems;
        if (!(settings.angle >= -360.0 && settings.angle <= 360.0))
            problems.push_back("The angle must be from -360 to 360 degrees.");
        return problems;
    }

    std::vector<std::string> ValidateIndex(const IndexSettings& settings)
    {
        std::vector<std::string> problems;
        if (!settings.automaticRetract && !(settings.retractHeight > 0.0 && settings.retractHeight < 10000.0))
            problems.push_back("The retract height must be more than 0.");
        return problems;
    }

    std::string IndexDirectionText(IndexDirection direction)
    {
        switch (direction)
        {
        case IndexDirection::Positive: return "positive";
        case IndexDirection::Negative: return "negative";
        default:                       return "shortest";
        }
    }

    bool IndexDirectionFromText(const std::string& text, IndexDirection& direction)
    {
        for (const IndexDirection each : { IndexDirection::Shortest, IndexDirection::Positive, IndexDirection::Negative })
        {
            if (IndexDirectionText(each) == text)
            {
                direction = each;
                return true;
            }
        }
        return false;
    }

    Vec3 RotaryDirection(RotaryAxis axis)
    {
        switch (axis)
        {
        case RotaryAxis::MinusX: return { -1.0, 0.0, 0.0 };
        case RotaryAxis::PlusX:  return { 1.0, 0.0, 0.0 };
        case RotaryAxis::MinusY: return { 0.0, -1.0, 0.0 };
        default:                 return { 0.0, 1.0, 0.0 };
        }
    }

    Vec3 RotateAbout(const Vec3& v, const Vec3& axis, double degrees)
    {
        // Rodrigues' formula.
        const double radians = degrees * kPi / 180.0;
        const double c = std::cos(radians), s = std::sin(radians);
        return v * c + Cross(axis, v) * s + axis * (Dot(axis, v) * (1.0 - c));
    }

    OrientationContext OrientationFor(const Machine& machine)
    {
        OrientationContext context;
        context.axes = machine.axes;
        context.rotary = machine.rotary;
        return context;
    }

    bool IsOrientationStep(StepType type)
    {
        return type == StepType::SetOrigin || type == StepType::Rotate;
    }

    FrameResult OrientationFrame(const Job& job, size_t index, const OrientationContext& context)
    {
        if (index >= job.sequence.size())
            return Failure("There's no Set Origin above it.");
        const SequenceStep& step = job.sequence[index];
        if (step.type == StepType::SetOrigin)
            return BuildFrame(step.origin);
        if (step.type != StepType::Rotate)
            return Failure("It isn't a Set Origin or Rotate.");

        if (context.axes != 4)
            return Failure("The machine has no rotary axis to turn about.");
        const size_t zeroIndex = WorkZeroIndex(job);
        if (zeroIndex == kNoStepIndex || zeroIndex > index)
            return Failure("There's no work zero (a Set Origin) above it to turn about.");
        const FrameResult zero = BuildFrame(job.sequence[zeroIndex].origin);
        if (!zero.ok)
            return Failure("The work zero (the first Set Origin) is incomplete.");

        FrameResult result;
        result.ok = true;
        result.frame = RotateFrame(zero.frame, zero.frame.origin, WorldAxis(zero.frame, context.rotary),
                                   step.rotate.angle);
        if (step.rotate.originAfter == OriginAfterRotate::TopOfStock)
        {
            double top = 0.0;
            if (context.stockPoints == nullptr || !HighestAlongZ(result.frame, *context.stockPoints, top))
                return Failure("There's no stock to put the origin on top of.");
            result.frame.origin = result.frame.origin + result.frame.zAxis * top;
        }
        return result;
    }

    FrameResult FrameAt(const Job& job, size_t stepIndex, const OrientationContext& context)
    {
        const size_t active = ActiveOriginIndex(job, stepIndex);
        if (active == kNoStepIndex)
            return Failure("There's no Set Origin above it.");
        return OrientationFrame(job, active, context);
    }

    const IndexSettings& IndexSettingsOf(const SequenceStep& step)
    {
        return IsOrientationStep(step.type) ? step.index : kDefaultIndex;
    }

    IndexSolution SolveIndex(const Frame& workZero, const OrientationContext& context, const Frame& frame)
    {
        IndexSolution solution;
        const Vec3& up = workZero.zAxis;
        if (context.axes != 4)
        {
            solution.ok = Dot(frame.zAxis, up) >= 1.0 - kReachTolerance;
            if (!solution.ok)
                solution.message = "A 3 axis machine can only cut along the work zero's Z.";
            return solution;
        }

        // Turning about the axis keeps Z's part along the axis, which is none.
        const Vec3 axis = WorldAxis(workZero, context.rotary);
        const double along = Dot(frame.zAxis, axis);
        const Vec3 across = frame.zAxis - axis * along;
        if (std::fabs(along) > std::sqrt(2.0 * kReachTolerance) || Length(across) < 0.5)
        {
            solution.message = "Its Z can't be reached by turning the rotary axis: it isn't square to the axis.";
            return solution;
        }

        // The angle that turns the work zero's Z to the frame's.
        double degrees = std::atan2(Dot(Cross(up, across), axis), Dot(up, across)) * 180.0 / kPi;
        if (degrees < 0.0)
            degrees += 360.0;
        if (degrees >= 360.0 - 1e-9)
            degrees = 0.0;
        solution.ok = true;
        solution.angle = degrees;
        return solution;
    }

    IndexSolution RotaryAngleOf(const Frame& workZero, RotaryAxis rotary, const Vec3& toolAxis)
    {
        Frame frame = workZero;
        IndexSolution solution;
        if (!Normalise(toolAxis, frame.zAxis))
        {
            solution.message = "The tool has no axis.";
            return solution;
        }
        OrientationContext context;
        context.axes = 4;
        context.rotary = rotary;
        return SolveIndex(workZero, context, frame);
    }

    RowMatrix TurnAboutLine(const Vec3& point, const Vec3& direction, double degrees)
    {
        RowMatrix m{};
        Vec3 axis;
        if (!Normalise(direction, axis))
            axis = { 0, 0, 1 };
        const Vec3 rows[3] = { RotateAbout({ 1, 0, 0 }, axis, degrees), RotateAbout({ 0, 1, 0 }, axis, degrees),
                               RotateAbout({ 0, 0, 1 }, axis, degrees) };
        for (int r = 0; r < 3; ++r)
        {
            m[r * 4 + 0] = rows[r].x;
            m[r * 4 + 1] = rows[r].y;
            m[r * 4 + 2] = rows[r].z;
        }
        // The point stays where it is.
        const Vec3 moved = RotateAbout(point, axis, degrees);
        m[12] = point.x - moved.x;
        m[13] = point.y - moved.y;
        m[14] = point.z - moved.z;
        m[15] = 1.0;
        return m;
    }

    RowMatrix MultiplyRows(const RowMatrix& a, const RowMatrix& b)
    {
        RowMatrix m{};
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                double sum = 0.0;
                for (int k = 0; k < 4; ++k)
                    sum += a[r * 4 + k] * b[k * 4 + c];
                m[r * 4 + c] = sum;
            }
        }
        return m;
    }

    Vec3 TransformPoint(const Vec3& p, const RowMatrix& m)
    {
        return { p.x * m[0] + p.y * m[4] + p.z * m[8] + m[12], p.x * m[1] + p.y * m[5] + p.z * m[9] + m[13],
                 p.x * m[2] + p.y * m[6] + p.z * m[10] + m[14] };
    }

    Vec3 MachinePoint(const Frame& workZero, RotaryAxis rotary, double angle, const Vec3& world)
    {
        // The axis runs through the work zero, so turning about it turns about 0.
        return RotateAbout(WorldToFrame(workZero, world), RotaryDirection(rotary), -angle);
    }

    Vec3 MachineDirection(const Frame& workZero, RotaryAxis rotary, double angle, const Vec3& world)
    {
        const Vec3 local{ Dot(world, workZero.xAxis), Dot(world, workZero.yAxis), Dot(world, workZero.zAxis) };
        return RotateAbout(local, RotaryDirection(rotary), -angle);
    }

    Vec3 WorldFromMachine(const Frame& workZero, RotaryAxis rotary, double angle, const Vec3& machine)
    {
        return FramePointToWorld(workZero, RotateAbout(machine, RotaryDirection(rotary), angle));
    }

    double NextIndexAngle(double current, double target, IndexDirection direction)
    {
        double change = std::fmod(target - current, 360.0);     // -360 to 360
        switch (direction)
        {
        case IndexDirection::Positive:
            if (change < 0.0)
                change += 360.0;
            break;
        case IndexDirection::Negative:
            if (change > 0.0)
                change -= 360.0;
            break;
        default:
            if (change > 180.0)
                change -= 360.0;
            else if (change <= -180.0)
                change += 360.0;
            break;
        }
        if (std::fabs(change) < 1e-9 || std::fabs(std::fabs(change) - 360.0) < 1e-9)
            change = 0.0;
        return current + change;
    }

    std::string EdgeOnFeaturesText(const std::vector<PickedGeometry>& features, const Frame& frame)
    {
        size_t count = 0;
        for (const PickedGeometry& feature : features)
        {
            Vec3 direction;
            if ((feature.kind == PickKind::PlanarFace || feature.kind == PickKind::CircleEdge) &&
                Normalise(feature.direction, direction) && std::fabs(Dot(direction, frame.zAxis)) < 0.05)
                ++count;
        }
        if (count == 0)
            return std::string();
        return " " + std::to_string(count) + (count == 1 ? " picked face or circle is" : " picked faces or circles are") +
               " edge-on to the Z of the Set Origin or Rotate above (seen side on, they're lines). A Rotate's angle "
               "is from the work zero, not from the Rotate above it: check it points Z out of their side.";
    }

    Vec3 RotaryAxisDirection(const Frame& workZero, RotaryAxis rotary)
    {
        const Vec3 local = RotaryDirection(rotary);
        return workZero.xAxis * local.x + workZero.yAxis * local.y + workZero.zAxis * local.z;
    }

    IndexPlan PlanIndexes(const Frame& workZero, const OrientationContext& context,
                          const std::vector<PostOperation>& operations)
    {
        IndexPlan plan;
        plan.angles.assign(operations.size(), 0.0);
        plan.moves.assign(operations.size(), IndexMove());
        for (size_t i = 0; i < operations.size(); ++i)
        {
            if (operations[i].operation == nullptr)
                continue;
            const IndexSolution solution = SolveIndex(workZero, context, operations[i].frame);
            if (solution.ok)
                plan.angles[i] = solution.angle;
            else
                plan.unreachable += "\n" + operations[i].operation->name;
        }
        plan.ok = plan.unreachable.empty();
        if (context.axes != 4)
            return plan;

        // The automatic retract clears the stock, and every toolpath.
        double furthest = (context.stockPoints != nullptr) ? FurthestFromAxis(workZero, context.rotary, *context.stockPoints)
                                                            : 0.0;
        for (const PostOperation& entry : operations)
        {
            if (entry.operation == nullptr)
                continue;
            std::vector<Vec3> points;
            for (const ToolpathMove& move : entry.operation->toolpath.moves)
            {
                if (move.kind != MoveKind::Dwell)
                    points.push_back(FramePointToWorld(entry.frame, move.to));
            }
            furthest = std::max(furthest, FurthestFromAxis(workZero, context.rotary, points));
        }
        plan.automaticRetract = furthest + kIndexClearance;

        double current = 0.0;
        for (size_t i = 0; i < operations.size(); ++i)
        {
            if (operations[i].operation == nullptr ||
                std::fabs(NextIndexAngle(current, plan.angles[i], IndexDirection::Shortest) - current) < 1e-9)
                continue;
            const IndexSettings index = (operations[i].origin != nullptr) ? IndexSettingsOf(*operations[i].origin)
                                                                           : IndexSettings();
            IndexMove& move = plan.moves[i];
            move.turn = true;
            move.from = current;
            move.to = NextIndexAngle(current, plan.angles[i], index.direction);
            move.retract = index.automaticRetract ? plan.automaticRetract : index.retractHeight;
            current = move.to;
        }
        return plan;
    }

    double FurthestFromAxis(const Frame& workZero, RotaryAxis rotary, const std::vector<Vec3>& world)
    {
        const Vec3 axis = RotaryDirection(rotary);
        double furthest = 0.0;
        for (const Vec3& point : world)
        {
            const Vec3 local = WorldToFrame(workZero, point);
            const Vec3 across = local - axis * Dot(local, axis);
            furthest = std::max(furthest, Length(across));
        }
        return furthest;
    }
}
