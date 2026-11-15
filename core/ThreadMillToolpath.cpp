// ThreadMillToolpath.cpp : calculates a Thread Milling operation's toolpath.

#include "ThreadMillToolpath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "Features.h"

namespace camcore
{
    namespace
    {
        const double kPi = 3.14159265358979323846;

        ToolpathResult Failure(const std::string& problem)
        {
            ToolpathResult result;
            result.problem = problem;
            return result;
        }

        std::string Number(double value)
        {
            char text[64];
            std::snprintf(text, sizeof(text), "%.3f", value);
            std::string result = text;
            while (!result.empty() && result.back() == '0')
                result.pop_back();
            if (!result.empty() && result.back() == '.')
                result.pop_back();
            return result;
        }

        Vec3 Around(const Vec3& centre, double radius, double angle, double z)
        {
            return { centre.x + radius * std::cos(angle), centre.y + radius * std::sin(angle), z };
        }

        class ThreadBuilder
        {
        public:
            ThreadBuilder(Toolpath& toolpath, const FeedSettings& feeds, bool anticlockwise)
                : m_toolpath(toolpath), m_feeds(feeds), m_anticlockwise(anticlockwise)
            {
            }

            const Vec3& At() const { return m_toolpath.moves.back().to; }

            void Rapid(const Vec3& to) { Add(MoveKind::Rapid, to, 0.0); }

            void Transfer(const Vec3& to)
            {
                if (m_toolpath.moves.empty() || m_feeds.transferAtRapid)
                    Add(MoveKind::Rapid, to, 0.0);
                else
                    Add(MoveKind::Feed, to, m_feeds.transfer);
            }

            void Feed(const Vec3& to, double feed) { Add(MoveKind::Feed, to, feed); }

            // Along the centre to z: feeding down, as the hole below may not be
            // open, and at rapid up through what's been cut.
            void AlongCentre(const Vec3& centre, double z)
            {
                const Vec3 to{ centre.x, centre.y, z };
                if (z < At().z - 1e-9)
                    Feed(to, m_feeds.plunge);
                else if (z > At().z + 1e-9)
                    Rapid(to);
            }

            void Arc(const Vec3& to, const Vec3& centre, double feed)
            {
                Add(MoveKind::Feed, to, feed);
                ToolpathMove& move = m_toolpath.moves.back();
                move.arc = true;
                move.centre = { centre.x, centre.y, to.z };
                move.clockwise = !m_anticlockwise;
            }

            // From the centre out to the circle at angle, round a half circle on
            // that radius, tangent to the circle there, ending at z.
            void LeadIn(const Vec3& centre, double radius, double angle, double z)
            {
                const Vec3 middle = Around(centre, radius / 2.0, angle, z);
                Arc(Around(centre, radius, angle, z), middle, m_feeds.engage);
            }

            // From the circle at angle back in to the centre the same way round.
            void LeadOut(const Vec3& centre, double radius, double angle, double z)
            {
                const Vec3 middle = Around(centre, radius / 2.0, angle, z);
                Arc({ centre.x, centre.y, z }, middle, m_feeds.engage);
            }

            // Round the circle about centre from angle, moving from the current
            // Z to z, by sweep radians, in arcs of at most half a turn. Returns
            // the angle it ends at.
            double Helix(const Vec3& centre, double radius, double angle, double sweep, double z)
            {
                const double from = At().z;
                const int halves = (std::max)(1, static_cast<int>(std::ceil(std::fabs(sweep) / kPi - 1e-9)));
                for (int i = 1; i <= halves; ++i)
                {
                    const double a = angle + sweep * i / halves;
                    Arc(Around(centre, radius, a, from + (z - from) * i / halves), centre, m_feeds.cut);
                }
                return angle + sweep;
            }

        private:
            void Add(MoveKind kind, const Vec3& to, double feed)
            {
                ToolpathMove move;
                move.kind = kind;
                move.to = to;
                move.feed = feed;
                m_toolpath.moves.push_back(move);
            }

            Toolpath& m_toolpath;
            const FeedSettings& m_feeds;
            bool m_anticlockwise;
        };
    }

    std::vector<ThreadHole> ThreadHoles(const SequenceStep& operation, const Frame& frame, std::string& problem)
    {
        const ThreadMillingSettings& thread = operation.threadMilling;
        std::vector<ThreadHole> holes;
        for (const PickedGeometry& feature : operation.features)
        {
            const Vec3 point = WorldToFrame(frame, FeaturePoint(frame, feature));
            const double top = (thread.location == CutGeometryLocation::AtBottom) ? point.z + thread.length
                               : (thread.location == CutGeometryLocation::PickTop) ? thread.topZ
                                                                                   : point.z;
            ThreadHole hole;
            hole.centre = { point.x, point.y, top };
            hole.bottom = top - thread.length;
            holes.push_back(hole);
        }
        if (holes.empty())
            problem = "There are no threads. Pick circles, hole faces or points on the Features tab.";
        return holes;
    }

    ToolpathResult GenerateThreadMillToolpath(const ThreadMillToolpathInput& input)
    {
        const SequenceStep* pOperation = input.operation;
        if (pOperation == nullptr || pOperation->type != StepType::ThreadMilling)
            return Failure("This isn't a thread milling operation.");
        const Tool* pTool = input.tool;
        if (pTool == nullptr || !(pTool->diameter > 0.0))
            return Failure("Choose a tool on the Tool tab.");
        if (pTool->type != ToolType::ThreadMill)
            return Failure("Thread milling needs a thread mill. Choose one on the Tool tab.");
        if (pOperation->features.empty())
            return Failure("There are no threads. Pick circles, hole faces or points on the Features tab.");

        const ThreadMillingSettings& thread = pOperation->threadMilling;
        for (const std::vector<std::string>& problems : { ValidateThreadMilling(thread), ValidateFeeds(pOperation->feeds) })
        {
            if (!problems.empty())
                return Failure(problems.front());
        }

        double clearance = 0.0;
        std::string problem;
        if (!ClearanceHeight(pOperation->clearance, input.havePartMaxZ, input.partMaxZ, clearance, problem,
                             input.haveStockMaxZ, input.stockMaxZ, input.haveFixtureMaxZ, input.fixtureMaxZ))
            return Failure(problem);

        // The radii the tool's centre goes round at.
        const double toolRadius = pTool->diameter / 2.0;
        const double minor = ThreadMinorDiameter(thread, pTool->threadAngle);
        if (!(toolRadius < minor / 2.0 - 1e-6))
            return Failure("The tool is too wide for the thread's minor diameter (" + Number(minor) + " mm).");
        const double first = minor / 2.0 - toolRadius;
        const double last = thread.majorDiameter / 2.0 - toolRadius;
        std::vector<double> radii;
        for (int k = 1; k <= thread.radialPasses; ++k)
            radii.push_back(first + (last - first) * k / thread.radialPasses);
        if (thread.springPass)
            radii.push_back(last);

        // Which way round, and whether that goes up or down.
        const bool anticlockwise = (thread.direction != CutDirection::Conventional);
        const bool rising = (thread.hand == ThreadHand::Right) == anticlockwise;
        const double turn = anticlockwise ? 2.0 * kPi : -2.0 * kPi;
        const double pitch = thread.pitch;
        const double lead = pitch / 4.0;        // along the thread in a lead's quarter turn
        const double along = rising ? 1.0 : -1.0;

        ToolpathResult result;
        const int teeth = (std::max)(1, pTool->threadTeeth);
        if (teeth > 1 && std::fabs(pTool->pitch - pitch) > 1e-6)
            result.warnings.push_back("The tool's teeth are " + Number(pTool->pitch) + " mm apart but the thread's pitch is " +
                                      Number(pitch) + " mm.");

        // Each turn's lowest tooth runs from start to end along the frame's Z.
        // A single point helixes the whole length in one go.
        struct Turn
        {
            double low = 0.0;       // where the lowest tooth starts or ends, the lower of the two
            double turns = 1.0;
        };
        std::vector<Turn> pieces;
        if (teeth == 1)
        {
            pieces.push_back({ 0.0, thread.length / pitch });
        }
        else
        {
            // Turns n pitches apart from the bottom, the last no further up than
            // it needs to be, moved by whole pitches so the teeth stay in step.
            const double form = teeth * pitch;
            const int count = (std::max)(1, static_cast<int>(std::ceil(thread.length / form - 1e-9)));
            const double highest = (std::max)(0.0, std::ceil((thread.length - form) / pitch - 1e-9)) * pitch;
            for (int j = 0; j < count; ++j)
                pieces.push_back({ (std::min)(j * form, highest), 1.0 });
            if (!rising)
                std::reverse(pieces.begin(), pieces.end());
        }

        std::string holeProblem;
        const std::vector<ThreadHole> holes = ThreadHoles(*pOperation, input.frame, holeProblem);
        if (holes.empty())
            return Failure(holeProblem);

        ThreadBuilder path(result.toolpath, pOperation->feeds, anticlockwise);
        for (size_t n = 0; n < holes.size(); ++n)
        {
            const ThreadHole& hole = holes[n];
            const std::string which = (holes.size() > 1) ? "Thread " + std::to_string(n + 1) + ": " : "";
            const Vec3& centre = hole.centre;
            const double approach = centre.z + thread.approachDistance;
            if (!(clearance > approach))
                return Failure(which + "the clearance plane must be above the approach distance over the thread.");

            path.Transfer({ centre.x, centre.y, clearance });
            path.Rapid({ centre.x, centre.y, approach });

            for (const double radius : radii)
            {
                for (const Turn& piece : pieces)
                {
                    // The lowest tooth's travel along Z, in the way it goes.
                    const double low = hole.bottom + piece.low;
                    const double high = low + piece.turns * pitch;
                    const double start = rising ? low : high;
                    const double end = rising ? high : low;

                    const double angle = 0.0;
                    path.AlongCentre(centre, start - along * lead);
                    path.LeadIn(centre, radius, angle, start);
                    const double finish = path.Helix(centre, radius, angle, turn * piece.turns, end);
                    path.LeadOut(centre, radius, finish, end + along * lead);
                }
            }
            path.Rapid({ centre.x, centre.y, clearance });
        }

        result.ok = !result.toolpath.moves.empty();
        if (!result.ok)
            result.problem = "No threads to cut.";
        return result;
    }
}
