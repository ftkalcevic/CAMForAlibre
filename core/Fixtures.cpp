// Fixtures.cpp : work holding (vices, clamps, screws) the tool must keep clear of.

#include "FixtureCheck.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace camcore
{
    namespace
    {
        constexpr double kPiValue = 3.14159265358979323846;

        // How far down a rapid comes before the plunge feed into a cut that starts again.
        constexpr double kTrimApproach = 2.0;

        std::string Number(double value)
        {
            char text[32];
            std::snprintf(text, sizeof(text), "%.4g", value);
            return text;
        }

        void Grow(Vec3& lo, Vec3& hi, const Vec3& p)
        {
            lo = { (std::min)(lo.x, p.x), (std::min)(lo.y, p.y), (std::min)(lo.z, p.z) };
            hi = { (std::max)(hi.x, p.x), (std::max)(hi.y, p.y), (std::max)(hi.z, p.z) };
        }
    }

    std::vector<std::string> ValidateFixtures(const FixtureSettings& settings)
    {
        std::vector<std::string> problems;
        if (!(settings.defaultClearance >= 0.0))
            problems.push_back("The clearance for new parts can't be negative.");
        for (const FixturePart& part : settings.parts)
        {
            if (!(part.clearance >= 0.0))
            {
                problems.push_back("The clearance for " + part.name + " can't be negative.");
                break;
            }
        }
        return problems;
    }

    double FixtureCell(const FixtureMesh& mesh, double maxColumns)
    {
        double cell = (mesh.clearance > 0.0) ? (std::min)(0.25, (std::max)(0.05, mesh.clearance / 2.0)) : 0.25;
        if (mesh.triangles.empty())
            return cell;
        Vec3 lo = mesh.triangles.front(), hi = lo;
        for (const Vec3& p : mesh.triangles)
            Grow(lo, hi, p);
        const double area = (hi.x - lo.x + 2.0 * cell) * (hi.y - lo.y + 2.0 * cell);
        if (area / (cell * cell) > maxColumns)
            cell = std::sqrt(area / maxColumns);
        return cell;
    }

    FixtureGrid MakeFixtureGrid(const FixtureMesh& mesh)
    {
        FixtureGrid fixture;
        fixture.name = mesh.name;
        fixture.clearance = mesh.clearance;
        if (mesh.triangles.size() < 3)
            return fixture;

        Vec3 lo = mesh.triangles.front(), hi = lo;
        for (const Vec3& p : mesh.triangles)
            Grow(lo, hi, p);
        const double cell = FixtureCell(mesh);
        // A cell to spare all round.
        const StockBox box{ lo - Vec3{ cell, cell, cell }, hi + Vec3{ cell, cell, cell } };
        fixture.grid.Reset(box, cell, true);
        fixture.grid.FillMesh(mesh.triangles);
        fixture.lower = box.lower;
        fixture.upper = box.upper;
        return fixture;
    }

    FixtureGrids MakeFixtureGrids(const std::vector<FixtureMesh>& meshes)
    {
        auto grids = std::make_shared<std::vector<FixtureGrid>>();
        for (const FixtureMesh& mesh : meshes)
            grids->push_back(MakeFixtureGrid(mesh));
        return grids;
    }

    bool FixtureTopAlongZ(const Frame& frame, const std::vector<FixtureMesh>& meshes, double& top)
    {
        bool found = false;
        for (const FixtureMesh& mesh : meshes)
        {
            for (const Vec3& p : mesh.triangles)
            {
                const double z = Dot(p - frame.origin, frame.zAxis) + mesh.clearance;
                if (!found || z > top)
                    top = z;
                found = true;
            }
        }
        return found;
    }

    FixtureTool FixtureToolOf(const Tool& toolMm, double holderDiameter, double holderDepth)
    {
        FixtureTool tool;
        tool.type = toolMm.type;
        tool.radius = toolMm.diameter / 2.0;
        tool.cornerRadius = toolMm.cornerRadius;
        tool.length = (toolMm.overallLength > 0.0) ? toolMm.overallLength : (std::max)(toolMm.fluteLength, 50.0);
        if (toolMm.HasShank() && toolMm.fluteLength > 0.0)
            tool.length = (std::min)(tool.length, toolMm.fluteLength);     // the shank is checked on its own
        tool.shank = ShankOf(toolMm);
        tool.holder = HolderOf(toolMm, holderDiameter, holderDepth);
        return tool;
    }

    std::vector<ToolpathChord> ToolpathChords(const Toolpath& toolpath, double tolerance)
    {
        std::vector<ToolpathChord> chords;
        bool havePrevious = false;
        Vec3 previous;
        for (size_t m = 0; m < toolpath.moves.size(); ++m)
        {
            const ToolpathMove& move = toolpath.moves[m];
            if (move.kind == MoveKind::Dwell)
                continue;
            if (!havePrevious)
            {
                // Where the operation starts; the move there isn't known.
                previous = move.to;
                havePrevious = true;
                continue;
            }
            std::vector<Vec3> points{ previous };
            if (move.arc)
            {
                double fu, fv, fw, cu, cv, cw;
                ToPlane(move.plane, previous, fu, fv, fw);
                ToPlane(move.plane, move.centre, cu, cv, cw);
                const double r = std::hypot(fu - cu, fv - cv);
                const double step = (r > tolerance) ? 2.0 * std::acos(1.0 - tolerance / r) : kPiValue / 4.0;
                std::vector<Vec3> along = ArcMovePoints(previous, move, step);
                along.pop_back();
                points.insert(points.end(), along.begin(), along.end());
            }
            points.push_back(move.to);
            for (size_t p = 0; p + 1 < points.size(); ++p)
                chords.push_back({ points[p], points[p + 1], m, move.kind == MoveKind::Rapid });
            previous = move.to;
        }
        return chords;
    }

    bool ChordHitsFixture(const FixtureGrid& fixture, const FixtureTool& tool, const Frame& frame, const Vec3& from,
                          const Vec3& to, bool* pHolder)
    {
        if (fixture.grid.SizeX() == 0)
            return false;
        Vec3 w;
        if (!Normalise(frame.zAxis, w))
            return false;
        const Vec3 a = FramePointToWorld(frame, from);
        const Vec3 b = FramePointToWorld(frame, to);
        const double c = fixture.clearance;

        // Each piece grown by the clearance all round: its end rounded by the
        // clearance (a ball's by its own radius plus it), starting `start` above the tip.
        struct Piece
        {
            double start, radius, corner, length;
            bool holder;
        };
        std::vector<Piece> pieces;
        if (tool.radius > 0.0 && tool.length > 0.0)
        {
            double corner = c;
            if (tool.type == ToolType::BallMill)
                corner = tool.radius + c;
            else if (tool.type == ToolType::CornerRadiusMill)
                corner = (std::min)(tool.cornerRadius, tool.radius) + c;
            pieces.push_back({ -c, tool.radius + c, corner, tool.length + 2.0 * c, false });
        }
        if (tool.shank.Present())
            pieces.push_back({ tool.shank.start - c, tool.shank.radius + c, c, tool.shank.length + 2.0 * c, false });
        if (tool.holder.Present())
            pieces.push_back({ tool.holder.start - c, tool.holder.radius + c, c, tool.holder.length + 2.0 * c, true });

        for (const Piece& piece : pieces)
        {
            const Vec3 pa = a + w * piece.start;
            const Vec3 pb = b + w * piece.start;

            // Nothing to do if the swept cylinder's box misses the fixture's.
            Vec3 lo = pa, hi = pa;
            for (const Vec3& p : { pb, pa + w * piece.length, pb + w * piece.length })
                Grow(lo, hi, p);
            const double r = piece.radius;
            if (hi.x + r < fixture.lower.x || lo.x - r > fixture.upper.x || hi.y + r < fixture.lower.y ||
                lo.y - r > fixture.upper.y || hi.z + r < fixture.lower.z || lo.z - r > fixture.upper.z)
                continue;

            ToolProfile plain;
            plain.type = (piece.corner > 0.0) ? ToolType::CornerRadiusMill : ToolType::FlatEndMill;
            plain.radius = piece.radius;
            plain.cornerRadius = piece.corner;
            plain.length = piece.length;
            if (CountMove(fixture.grid, plain, pa, pb, w) > 0)
            {
                if (pHolder != nullptr)
                    *pHolder = piece.holder;
                return true;
            }
        }
        return false;
    }

    std::vector<FixtureHit> CheckFixtures(const Toolpath& toolpath, const Frame& frame, const FixtureTool& tool,
                                          const std::vector<FixtureGrid>& fixtures)
    {
        std::vector<FixtureHit> hits;
        if (fixtures.empty())
            return hits;
        const std::vector<ToolpathChord> chords = ToolpathChords(toolpath);
        for (size_t f = 0; f < fixtures.size(); ++f)
        {
            for (const ToolpathChord& chord : chords)
            {
                bool holder = false;
                if (ChordHitsFixture(fixtures[f], tool, frame, chord.from, chord.to, &holder))
                {
                    hits.push_back({ f, chord.move, holder });
                    break;
                }
            }
        }
        return hits;
    }

    std::string FixtureHitText(const FixtureHit& hit, const std::vector<FixtureGrid>& fixtures)
    {
        const FixtureGrid* pFixture = (hit.fixture < fixtures.size()) ? &fixtures[hit.fixture] : nullptr;
        const std::string name = pFixture ? pFixture->name : std::string("a fixture");
        const double clearance = pFixture ? pFixture->clearance : 0.0;
        return std::string(hit.holder ? "the tool holder comes" : "comes") + " within " + Number(clearance) +
               " mm of " + name + " at move " + std::to_string(hit.move + 1);
    }

    namespace
    {
        bool HitsAny(const std::vector<FixtureGrid>& fixtures, const FixtureTool& tool, const Frame& frame,
                     const Vec3& a, const Vec3& b)
        {
            for (const FixtureGrid& fixture : fixtures)
            {
                if (ChordHitsFixture(fixture, tool, frame, a, b))
                    return true;
            }
            return false;
        }

        // A chord as clear and colliding pieces in order, split no shorter than
        // `shortest`: each piece's end, and whether the piece up to it collides.
        void SplitChord(const std::vector<FixtureGrid>& fixtures, const FixtureTool& tool, const Frame& frame,
                        const Vec3& a, const Vec3& b, double shortest, std::vector<std::pair<Vec3, bool>>& ends,
                        int depth = 0)
        {
            if (!HitsAny(fixtures, tool, frame, a, b))
            {
                ends.push_back({ b, false });
                return;
            }
            if (Length(b - a) <= shortest || depth > 40)
            {
                ends.push_back({ b, true });
                return;
            }
            const Vec3 middle = (a + b) * 0.5;
            SplitChord(fixtures, tool, frame, a, middle, shortest, ends, depth + 1);
            SplitChord(fixtures, tool, frame, middle, b, shortest, ends, depth + 1);
        }

        ToolpathMove Straight(MoveKind kind, const Vec3& to, double feed)
        {
            ToolpathMove move;
            move.kind = kind;
            move.to = to;
            move.feed = (kind == MoveKind::Feed) ? feed : 0.0;
            return move;
        }
    }

    bool TrimAroundFixtures(Toolpath& toolpath, const Frame& frame, const FixtureTool& tool,
                            const std::vector<FixtureGrid>& fixtures, double clearanceHeight, double plungeFeed,
                            int& trimmed, std::string& problem)
    {
        trimmed = 0;
        if (fixtures.empty() || toolpath.moves.empty())
            return true;

        double shortest = 1e300;
        for (const FixtureGrid& fixture : fixtures)
        {
            if (fixture.grid.SizeX() > 0)
                shortest = (std::min)(shortest, (std::max)(0.01, fixture.grid.Cell() / 2.0));
        }
        if (shortest > 1e299)
            return true;

        const std::string tooLow =
            "The clearance plane (" + Number(clearanceHeight) + " mm) is too low to lift over a fixture.";

        Toolpath result;
        bool havePrevious = false;
        Vec3 previous;          // where the tool is, in the original toolpath
        bool inside = false;    // in a stretch that's being left out
        Vec3 leftAt;            // the last clear point before it
        double feedBefore = plungeFeed;
        bool failed = false;

        // Over the stretch left out: up from leftAt, across, and down into `to`.
        const auto bridge = [&](const Vec3& to) {
            const Vec3 up{ leftAt.x, leftAt.y, (std::max)(clearanceHeight, leftAt.z) };
            const Vec3 over{ to.x, to.y, (std::max)(clearanceHeight, to.z) };
            const Vec3 above{ to.x, to.y, (std::min)(over.z, to.z + kTrimApproach) };
            if (HitsAny(fixtures, tool, frame, leftAt, up) || HitsAny(fixtures, tool, frame, up, over) ||
                HitsAny(fixtures, tool, frame, over, to))
            {
                failed = true;
                return;
            }
            if (up.z > leftAt.z)
                result.moves.push_back(Straight(MoveKind::Feed, up, feedBefore));
            result.moves.push_back(Straight(MoveKind::Rapid, over, 0.0));
            if (above.z < over.z)
                result.moves.push_back(Straight(MoveKind::Rapid, above, 0.0));
            if (to.z < above.z)
                result.moves.push_back(Straight(MoveKind::Feed, to, (plungeFeed > 0.0) ? plungeFeed : feedBefore));
            ++trimmed;
        };

        for (size_t m = 0; m < toolpath.moves.size() && !failed; ++m)
        {
            const ToolpathMove& move = toolpath.moves[m];
            if (move.kind == MoveKind::Dwell)
            {
                if (!inside)
                    result.moves.push_back(move);
                continue;
            }
            if (!havePrevious)
            {
                // Where the operation starts.
                previous = leftAt = move.to;
                havePrevious = true;
                result.moves.push_back(move);
                continue;
            }

            Toolpath single;
            single.moves.push_back(Straight(MoveKind::Rapid, previous, 0.0));
            single.moves.push_back(move);
            const std::vector<ToolpathChord> chords = ToolpathChords(single);
            bool clear = true;
            for (const ToolpathChord& chord : chords)
            {
                if (HitsAny(fixtures, tool, frame, chord.from, chord.to))
                {
                    clear = false;
                    break;
                }
            }

            if (clear)
            {
                // A move that's clear all along stays as it is.
                if (inside)
                {
                    bridge(previous);
                    inside = false;
                }
                result.moves.push_back(move);
                if (move.kind == MoveKind::Feed)
                    feedBefore = move.feed;
                previous = leftAt = move.to;
                continue;
            }

            for (const ToolpathChord& chord : chords)
            {
                std::vector<std::pair<Vec3, bool>> ends;
                SplitChord(fixtures, tool, frame, chord.from, chord.to, shortest, ends);
                Vec3 pieceStart = chord.from;
                for (const auto& [end, hits] : ends)
                {
                    if (hits)
                    {
                        inside = true;
                    }
                    else
                    {
                        if (inside)
                        {
                            bridge(pieceStart);
                            inside = false;
                        }
                        result.moves.push_back(Straight(move.kind, end, move.feed));
                        if (move.kind == MoveKind::Feed)
                            feedBefore = move.feed;
                        leftAt = end;
                    }
                    pieceStart = end;
                }
            }
            previous = move.to;
        }

        if (!failed && inside)
        {
            // It ends too close: lift away and stop there.
            const Vec3 up{ leftAt.x, leftAt.y, (std::max)(clearanceHeight, leftAt.z) };
            if (HitsAny(fixtures, tool, frame, leftAt, up))
                failed = true;
            else
            {
                if (up.z > leftAt.z)
                    result.moves.push_back(Straight(MoveKind::Feed, up, feedBefore));
                ++trimmed;
            }
        }
        if (failed)
        {
            trimmed = 0;
            problem = tooLow;
            return false;
        }
        if (trimmed > 0)
            toolpath = std::move(result);
        return true;
    }
}
