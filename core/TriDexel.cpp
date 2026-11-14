// TriDexel.cpp : the simulation's stock as three grids of rays (tri-dexels).

#include "TriDexel.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <execution>
#include <numeric>
#include <tuple>

#include "Simulation.h"

namespace camcore
{
    namespace
    {
        // Spacings, fine to coarse (as the voxels' cell sizes).
        const double kSpacings[] = { 0.05, 0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 2.5, 5.0, 10.0, 20.0, 25.0, 50.0, 100.0 };

        constexpr double kEndTolerance = 1e-4;      // how closely a cut's ends are found, in mm
        constexpr double kSliver = 1e-5;            // stock shorter than this left by a cut is dropped
        constexpr double kPenalty = 1e6;
        constexpr size_t kParallelRays = 256;       // fewer rays than this are worked out on one core

        int NodesAlong(double length, double spacing)
        {
            return (std::max)(1, static_cast<int>(std::ceil(length / spacing - 1e-9)));
        }

        double Get(const Vec3& v, int axis)
        {
            return (axis == 0) ? v.x : (axis == 1) ? v.y : v.z;
        }

        void Set(Vec3& v, int axis, double value)
        {
            if (axis == 0)
                v.x = value;
            else if (axis == 1)
                v.y = value;
            else
                v.z = value;
        }

        Vec3 Unit(int axis)
        {
            Vec3 v;
            Set(v, axis, 1.0);
            return v;
        }

        double DistanceToSegment(const Vec3& p, const Vec3& a, const Vec3& b)
        {
            const Vec3 ab = b - a;
            const double lengthSquared = Dot(ab, ab);
            double t = 0.0;
            if (lengthSquared > 1e-18)
                t = (std::min)(1.0, (std::max)(0.0, Dot(p - a, ab) / lengthSquared));
            return Length(p - (a + ab * t));
        }

        // Where the row at v crosses the points within radius of the segment a-b, in
        // a plane's (u, v): from lo to hi along u, or false if it doesn't.
        bool CapsuleRow(double au, double av, double bu, double bv, double radius, double v, double& lo, double& hi)
        {
            lo = 1e300;
            hi = -1e300;
            for (int end = 0; end < 2; ++end)
            {
                const double cu = end ? bu : au, cv = end ? bv : av;
                const double dv = v - cv;
                if (std::fabs(dv) <= radius)
                {
                    const double du = std::sqrt((std::max)(0.0, radius * radius - dv * dv));
                    lo = (std::min)(lo, cu - du);
                    hi = (std::max)(hi, cu + du);
                }
            }
            const double eu = bu - au, ev = bv - av;
            const double length = std::hypot(eu, ev);
            if (length > 1e-12)
            {
                const double nu = -ev / length * radius, nv = eu / length * radius;
                const double cornersU[4] = { au + nu, bu + nu, bu - nu, au - nu };
                const double cornersV[4] = { av + nv, bv + nv, bv - nv, av - nv };
                for (int e = 0; e < 4; ++e)
                {
                    const double pu = cornersU[e], pv = cornersV[e];
                    const double qu = cornersU[(e + 1) % 4], qv = cornersV[(e + 1) % 4];
                    if ((pv - v) * (qv - v) > 0.0)
                        continue;
                    if (std::fabs(qv - pv) < 1e-15)
                    {
                        lo = (std::min)({ lo, pu, qu });
                        hi = (std::max)({ hi, pu, qu });
                        continue;
                    }
                    const double u = pu + (v - pv) * (qu - pu) / (qv - pv);
                    lo = (std::min)(lo, u);
                    hi = (std::max)(hi, u);
                }
            }
            return lo <= hi;
        }

        // Where the line o + t e (e a unit vector) is within rho of the segment a-b:
        // the capsule round it is convex, so it's one stretch, from t0 to t1.
        bool LineCapsule(const Vec3& o, const Vec3& e, const Vec3& a, const Vec3& b, double rho, double& t0, double& t1)
        {
            if (rho < 0.0)
                return false;
            double lo = 1e300, hi = -1e300;
            const auto merge = [&](double x, double y) {
                if (x <= y)
                {
                    lo = (std::min)(lo, x);
                    hi = (std::max)(hi, y);
                }
            };
            for (const Vec3* c : { &a, &b })
            {
                const Vec3 q = o - *c;
                const double bq = Dot(q, e);
                const double disc = bq * bq - (Dot(q, q) - rho * rho);
                if (disc >= 0.0)
                {
                    const double root = std::sqrt(disc);
                    merge(-bq - root, -bq + root);
                }
            }
            const Vec3 ab = b - a;
            const double abLength = Length(ab);
            if (abLength > 1e-12)
            {
                const Vec3 u = ab * (1.0 / abLength);
                const Vec3 q = o - a;
                const double qu = Dot(q, u), eu = Dot(e, u);
                const Vec3 p0 = q - u * qu;         // the part of q square to the segment
                const Vec3 p1 = e - u * eu;
                // Along the segment: 0 <= qu + t eu <= abLength.
                double s0 = -1e300, s1 = 1e300;
                if (std::fabs(eu) < 1e-15)
                {
                    if (qu < 0.0 || qu > abLength)
                        s0 = 1.0, s1 = 0.0;
                }
                else
                {
                    s0 = (0.0 - qu) / eu;
                    s1 = (abLength - qu) / eu;
                    if (s0 > s1)
                        std::swap(s0, s1);
                }
                // Square to it: |p0 + t p1| <= rho.
                const double qa = Dot(p1, p1), qb = Dot(p0, p1), qc = Dot(p0, p0) - rho * rho;
                double r0 = -1e300, r1 = 1e300;
                if (qa < 1e-18)
                {
                    if (qc > 0.0)
                        r0 = 1.0, r1 = 0.0;
                }
                else
                {
                    const double disc = qb * qb - qa * qc;
                    if (disc < 0.0)
                    {
                        r0 = 1.0, r1 = 0.0;
                    }
                    else
                    {
                        const double root = std::sqrt(disc);
                        r0 = (-qb - root) / qa;
                        r1 = (-qb + root) / qa;
                    }
                }
                merge((std::max)(s0, r0), (std::min)(s1, r1));
            }
            if (lo > hi)
                return false;
            t0 = lo;
            t1 = hi;
            return true;
        }

        // The least of f between lo and hi, which must have one low point (fall,
        // then rise). Stops early once it finds a value at or under stopAt.
        template <typename F>
        double GoldenMin(F f, double lo, double hi, double tolerance, double& best, double stopAt = -1e300)
        {
            const double g = 0.6180339887498949;
            double bestX = lo;
            best = f(lo);
            const auto consider = [&](double x, double fx) {
                if (fx < best)
                {
                    best = fx;
                    bestX = x;
                }
            };
            if (best <= stopAt)
                return bestX;
            consider(hi, f(hi));
            if (best <= stopAt || !(hi - lo > tolerance))
                return bestX;
            double a = lo, b = hi;
            double c = b - g * (b - a), d = a + g * (b - a);
            double fc = f(c), fd = f(d);
            consider(c, fc);
            consider(d, fd);
            while (b - a > tolerance && best > stopAt)
            {
                if (fc < fd)
                {
                    b = d;
                    d = c;
                    fd = fc;
                    c = b - g * (b - a);
                    fc = f(c);
                    consider(c, fc);
                }
                else
                {
                    a = c;
                    c = d;
                    fc = fd;
                    d = a + g * (b - a);
                    fd = f(d);
                    consider(d, fd);
                }
            }
            return bestX;
        }

        // Between a point where in(x) holds and one where it doesn't, where it stops.
        template <typename F>
        double Boundary(F in, double inside, double outside, double tolerance)
        {
            while (std::fabs(outside - inside) > tolerance)
            {
                const double middle = 0.5 * (inside + outside);
                if (in(middle))
                    inside = middle;
                else
                    outside = middle;
            }
            return 0.5 * (inside + outside);
        }

        // The tool's height for a radius, or a large number where it's never that wide.
        double HeightOrPenalty(const ToolProfile& profile, double radius)
        {
            const double h = profile.HeightForRadius(radius);
            return (h < 0.0) ? kPenalty : h;
        }

        // A move of the tool, as the sweeps along and square to its axis see it.
        struct Sweep
        {
            const ToolProfile* profile = nullptr;
            Vec3 w;             // the tool's axis
            double za = 0.0;    // the tips along it
            double zb = 0.0;
            Vec3 pa;            // the tips with the axis taken out
            Vec3 pb;
        };

        Sweep MakeSweep(const ToolProfile& profile, const Vec3& w, const Vec3& a, const Vec3& b)
        {
            Sweep sweep;
            sweep.profile = &profile;
            sweep.w = w;
            sweep.za = Dot(a, w);
            sweep.zb = Dot(b, w);
            sweep.pa = a - w * sweep.za;
            sweep.pb = b - w * sweep.zb;
            return sweep;
        }

        // A ray along the tool's axis, through o: where the swept tool covers it, in
        // distance along the axis.
        bool AlongAxis(const Sweep& sweep, const Vec3& o, double& low, double& high)
        {
            const ToolProfile& profile = *sweep.profile;
            const Vec3 op = o - sweep.w * Dot(o, sweep.w);
            const Vec3 q = op - sweep.pa;
            const Vec3 v = sweep.pb - sweep.pa;
            const double radius = profile.radius;

            // The part of the move whose tool reaches out to the ray.
            const double qa = Dot(v, v), qb = Dot(q, v), qc = Dot(q, q) - radius * radius;
            double l0 = 0.0, l1 = 1.0;
            if (qa < 1e-18)
            {
                if (qc > 0.0)
                    return false;
            }
            else
            {
                const double disc = qb * qb - qa * qc;
                if (disc < 0.0)
                    return false;
                const double root = std::sqrt(disc);
                l0 = (std::max)(0.0, (qb - root) / qa);
                l1 = (std::min)(1.0, (qb + root) / qa);
                if (l0 > l1)
                    return false;
            }
            const double dz = sweep.zb - sweep.za;
            const auto distance = [&](double lambda) { return Length(q - v * lambda); };
            if (std::fabs(dz) < 1e-9)
            {
                // A level move: lowest where the tool's centre is nearest.
                const double lambda = (qa < 1e-18) ? 0.0 : (std::min)(l1, (std::max)(l0, qb / qa));
                const double h = profile.HeightForRadius(distance(lambda));
                if (h < 0.0)
                    return false;
                low = sweep.za + h;
                high = sweep.za + profile.length;
                return true;
            }
            high = (std::max)(sweep.za + dz * l0, sweep.za + dz * l1) + profile.length;
            if (profile.type == ToolType::FlatEndMill)
            {
                // Flat: its tip, wherever it's over the ray.
                low = (std::min)(sweep.za + dz * l0, sweep.za + dz * l1);
                return true;
            }
            if (profile.type == ToolType::BallMill && profile.length >= profile.radius)
            {
                // A ball's sweep is the capsule round its centre's path: where the ray enters it.
                const Vec3 ca = sweep.pa + sweep.w * (sweep.za + profile.radius);
                const Vec3 cb = sweep.pb + sweep.w * (sweep.zb + profile.radius);
                const Vec3 origin = op;     // along the axis it starts at 0
                double t0 = 0.0, t1 = 0.0;
                if (!LineCapsule(origin, sweep.w, ca, cb, profile.radius, t0, t1))
                    return false;
                low = t0;
                return true;
            }
            // The tool's lowest point over the ray, tip height plus the height where
            // it's as wide as the ray is far, is convex along the move.
            const auto lowest = [&](double lambda) {
                return sweep.za + dz * lambda + HeightOrPenalty(profile, distance(lambda));
            };
            const double scale = std::sqrt(qa) + std::fabs(dz);
            double best = 0.0;
            GoldenMin(lowest, l0, l1, kEndTolerance / (std::max)(scale, 1e-9), best);
            if (best >= kPenalty / 2)
                return false;
            low = best;
            return true;
        }

        // A ray square to the tool's axis, o + t e: where the swept tool covers it.
        bool AcrossAxis(const Sweep& sweep, const Vec3& o, const Vec3& e, double& t0, double& t1)
        {
            const ToolProfile& profile = *sweep.profile;
            const double along = Dot(o, sweep.w);
            const Vec3 op = o - sweep.w * along;
            const double dz = sweep.zb - sweep.za;
            if (std::fabs(dz) < 1e-12)
            {
                // A level move: the tool is the same width all along it at this height.
                const double h = along - sweep.za;
                if (h < -1e-9 || h > profile.length)
                    return false;
                const double rho = profile.RadiusAt((std::max)(0.0, (std::min)(h, profile.length)));
                return LineCapsule(op, e, sweep.pa, sweep.pb, rho, t0, t1);
            }

            // The part of the move where the tool spans this height.
            double l0 = (along - profile.length - sweep.za) / dz;
            double l1 = (along - sweep.za) / dz;
            if (l0 > l1)
                std::swap(l0, l1);
            l0 = (std::max)(0.0, l0);
            l1 = (std::min)(1.0, l1);
            if (l0 > l1)
                return false;

            // At each point of the move the tool is a disc at this height: its centre
            // c is B along the ray and D to one side of it, its radius rho.
            const Vec3 n = Cross(sweep.w, e);
            const Vec3 v = sweep.pb - sweep.pa;
            const Vec3 c0 = sweep.pa - op;
            const double b0 = Dot(c0, e), bv = Dot(v, e);
            const double d0 = Dot(c0, n), dv = Dot(v, n);
            const auto rho = [&](double lambda) {
                const double h = along - (sweep.za + dz * lambda);
                return profile.RadiusAt((std::max)(0.0, (std::min)(h, profile.length)));
            };
            // Only where the disc's centre is within the tool's radius of the ray.
            if (std::fabs(dv) < 1e-15)
            {
                if (std::fabs(d0) > profile.radius)
                    return false;
            }
            else
            {
                double s0 = (-profile.radius - d0) / dv, s1 = (profile.radius - d0) / dv;
                if (s0 > s1)
                    std::swap(s0, s1);
                l0 = (std::max)(l0, s0);
                l1 = (std::min)(l1, s1);
                if (l0 > l1)
                    return false;
            }
            const double scale = Length(v) + std::fabs(dz) + 1e-9;
            const double tolerance = kEndTolerance / scale;

            // The ray's stretch is from the least near end to the greatest far end of
            // the discs that reach it. Where a disc doesn't, how far it falls short
            // (convex along the move) leads the search to where they do.
            const auto reach = [&](double lambda, double& half) {
                const double r = rho(lambda), d = d0 + dv * lambda;
                if (r < std::fabs(d))
                    return std::fabs(d) - r;
                half = std::sqrt(r * r - d * d);
                return 0.0;
            };
            const auto nearEnd = [&](double lambda) {
                double half = 0.0;
                const double short_ = reach(lambda, half);
                return (short_ > 0.0) ? kPenalty + short_ : b0 + bv * lambda - half;
            };
            const auto farEnd = [&](double lambda) {
                double half = 0.0;
                const double short_ = reach(lambda, half);
                return (short_ > 0.0) ? kPenalty + short_ : -(b0 + bv * lambda + half);
            };
            double nearest = 0.0, furthest = 0.0;
            GoldenMin(nearEnd, l0, l1, tolerance, nearest);
            if (nearest >= kPenalty / 2)
                return false;
            GoldenMin(farEnd, l0, l1, tolerance, furthest);
            if (furthest >= kPenalty / 2)
                return false;
            t0 = nearest;
            t1 = -furthest;
            return t0 <= t1;
        }

        // A ray at a slant to the tool, o + t e from t = lo to hi, against one short
        // step: the tool's shape at its lower tip swept square to the axis, up to
        // its length above the higher tip.
        bool Slanted(const Sweep& step, const Vec3& o, const Vec3& e, double lo, double hi, double& t0, double& t1)
        {
            const ToolProfile& profile = *step.profile;
            const double tipLow = (std::min)(step.za, step.zb);
            const double tipHigh = (std::max)(step.za, step.zb);
            const auto outside = [&](double t) {
                const Vec3 p = o + e * t;
                const double along = Dot(p, step.w);
                const double h = along - tipLow;
                if (h < 0.0)
                    return kPenalty - h;
                if (along - tipHigh > profile.length)
                    return kPenalty + (along - tipHigh - profile.length);
                const double radius = profile.RadiusAt((std::min)(h, profile.length));
                return DistanceToSegment(p - step.w * along, step.pa, step.pb) - radius;
            };
            double best = 0.0;
            const double in = GoldenMin(outside, lo, hi, kEndTolerance, best, 0.0);
            if (best > 0.0)
                return false;
            const auto inside = [&](double t) { return outside(t) <= 0.0; };
            t0 = inside(lo) ? lo : Boundary(inside, in, lo, kEndTolerance);
            t1 = inside(hi) ? hi : Boundary(inside, in, hi, kEndTolerance);
            return t0 <= t1;
        }

        struct Candidate
        {
            int iu;
            int iv;
        };

        struct Cut
        {
            bool hit = false;
            double t0 = 0.0;
            double t1 = 0.0;
        };

        template <typename F>
        void ForEachIndex(size_t count, F f)
        {
            if (count < kParallelRays)
            {
                for (size_t n = 0; n < count; ++n)
                    f(n);
                return;
            }
            std::vector<size_t> indices(count);
            std::iota(indices.begin(), indices.end(), size_t(0));
            std::for_each(std::execution::par, indices.begin(), indices.end(), f);
        }

        // Cutting, it removes the stock and returns the length removed; touching,
        // it returns 1 if any ray has more than tolerance of stock in the sweep.
        template <bool kCut>
        double SweepDexels(TriDexelStock& stock, const ToolProfile& profile, const Vec3& from, const Vec3& to,
                           const Vec3& axis, double tolerance)
        {
            Vec3 w;
            if (!Normalise(axis, w) || stock.RayCount(2) == 0)
                return 0.0;
            const double s = stock.Spacing();
            const Vec3 lower = stock.Lower();
            const Vec3 upper = stock.Box().upper;

            // Only the part of the tool below the stock's top can cut: a V bit skimming
            // the top sweeps only its narrow tip.
            double stockTop = -1e300;
            for (int corner = 0; corner < 8; ++corner)
            {
                const Vec3 c{ (corner & 1) ? upper.x : lower.x, (corner & 2) ? upper.y : lower.y,
                              (corner & 4) ? upper.z : lower.z };
                stockTop = (std::max)(stockTop, Dot(c, w));
            }
            const auto reachOf = [&](double tipLow) {
                const double up = stockTop - tipLow;
                if (up < 0.0)
                    return -1.0;
                const double widest = profile.RadiusAt((std::min)(up, profile.length));
                return (widest >= 0.0) ? (std::min)(profile.radius, widest + s) : profile.radius;
            };

            double result = 0.0;
            std::atomic<bool> touched{ false };

            // The rays of one grid in a box, each against a way of finding its cut.
            const auto cutRays = [&](int g, const Vec3& lo, const Vec3& hi, auto&& rowRange, auto&& find) {
                const int u = TriDexelStock::U(g), v = TriDexelStock::V(g);
                const int nu = stock.Size(u), nv = stock.Size(v);
                const int iv0 = (std::max)(0, static_cast<int>(std::floor((Get(lo, v) - Get(lower, v)) / s - 0.5)));
                const int iv1 = (std::min)(nv - 1, static_cast<int>(std::ceil((Get(hi, v) - Get(lower, v)) / s - 0.5)));
                const int iu0 = (std::max)(0, static_cast<int>(std::floor((Get(lo, u) - Get(lower, u)) / s - 0.5)));
                const int iu1 = (std::min)(nu - 1, static_cast<int>(std::ceil((Get(hi, u) - Get(lower, u)) / s - 0.5)));
                const double tLo = (std::max)(0.0, Get(lo, g) - Get(lower, g));
                const double tHi = (std::min)(stock.Length(g), Get(hi, g) - Get(lower, g));
                if (iv0 > iv1 || iu0 > iu1 || tLo > tHi)
                    return;

                std::vector<Candidate> candidates;
                for (int iv = iv0; iv <= iv1; ++iv)
                {
                    int a = iu0, b = iu1;
                    if (!rowRange(Get(lower, v) + (iv + 0.5) * s, a, b))
                        continue;
                    a = (std::max)(a, iu0);
                    b = (std::min)(b, iu1);
                    for (int iu = a; iu <= b; ++iu)
                    {
                        if (stock.AnyStock(g, stock.RayIndex(g, iu, iv), tLo, tHi))
                            candidates.push_back({ iu, iv });
                    }
                }
                if (candidates.empty())
                    return;

                const Vec3 e = Unit(g);
                std::vector<Cut> cuts(candidates.size());
                ForEachIndex(candidates.size(), [&](size_t n) {
                    if (!kCut && touched.load(std::memory_order_relaxed))
                        return;
                    const Candidate& candidate = candidates[n];
                    Vec3 o = lower;
                    Set(o, u, Get(lower, u) + (candidate.iu + 0.5) * s);
                    Set(o, v, Get(lower, v) + (candidate.iv + 0.5) * s);
                    Cut& cut = cuts[n];
                    cut.hit = find(o, e, tLo, tHi, cut.t0, cut.t1);
                    if (!kCut && cut.hit &&
                        stock.Overlap(g, stock.RayIndex(g, candidate.iu, candidate.iv), cut.t0, cut.t1) > tolerance)
                        touched.store(true, std::memory_order_relaxed);
                });
                if (!kCut)
                    return;
                for (size_t n = 0; n < candidates.size(); ++n)
                {
                    const Cut& cut = cuts[n];
                    if (!cut.hit)
                        continue;
                    const Candidate& candidate = candidates[n];
                    const double removed =
                        stock.Subtract(g, stock.RayIndex(g, candidate.iu, candidate.iv), cut.t0, cut.t1);
                    if (removed > 0.0)
                    {
                        result += removed;
                        stock.MarkDirtyRay(g, candidate.iu, candidate.iv, cut.t0, cut.t1);
                    }
                }
            };

            const auto boxOf = [&](const Vec3& a, const Vec3& b, double r, Vec3& lo, Vec3& hi) {
                lo = a;
                hi = a;
                for (const Vec3& p : { a, b, a + w * profile.length, b + w * profile.length })
                {
                    lo = { (std::min)(lo.x, p.x), (std::min)(lo.y, p.y), (std::min)(lo.z, p.z) };
                    hi = { (std::max)(hi.x, p.x), (std::max)(hi.y, p.y), (std::max)(hi.z, p.z) };
                }
                lo = lo - Vec3{ r, r, r };
                hi = hi + Vec3{ r, r, r };
            };

            // Rays along and square to the axis: the whole move at once, exactly.
            const Sweep whole = MakeSweep(profile, w, from, to);
            const double wholeReach = reachOf((std::min)(whole.za, whole.zb));
            bool anySlanted = false;
            if (wholeReach >= 0.0)
            {
                Vec3 lo, hi;
                boxOf(from, to, wholeReach, lo, hi);
                for (int g = 0; g < 3 && !(!kCut && touched.load()); ++g)
                {
                    const double ew = Get(w, g);
                    if (std::fabs(ew) > 1.0 - 1e-12)
                    {
                        // Along the axis: only the rows the tool's footprint crosses.
                        const int u = TriDexelStock::U(g), v = TriDexelStock::V(g);
                        const auto rowRange = [&](double vCoord, int& a, int& b) {
                            double uLo, uHi;
                            if (!CapsuleRow(Get(whole.pa, u), Get(whole.pa, v), Get(whole.pb, u), Get(whole.pb, v),
                                            wholeReach + 1e-6, vCoord, uLo, uHi))
                                return false;
                            a = static_cast<int>(std::floor((uLo - Get(lower, u)) / s - 0.5));
                            b = static_cast<int>(std::ceil((uHi - Get(lower, u)) / s - 0.5));
                            return true;
                        };
                        const auto find = [&](const Vec3& o, const Vec3&, double, double, double& t0, double& t1) {
                            double low, high;
                            if (!AlongAxis(whole, o, low, high))
                                return false;
                            const double base = Dot(o, w);
                            t0 = (low - base) / ew;
                            t1 = (high - base) / ew;
                            if (t0 > t1)
                                std::swap(t0, t1);
                            return true;
                        };
                        cutRays(g, lo, hi, rowRange, find);
                    }
                    else if (std::fabs(ew) < 1e-12)
                    {
                        const auto rowRange = [](double, int&, int&) { return true; };
                        const auto find = [&](const Vec3& o, const Vec3& e, double, double, double& t0, double& t1) {
                            return AcrossAxis(whole, o, e, t0, t1);
                        };
                        cutRays(g, lo, hi, rowRange, find);
                    }
                    else
                    {
                        anySlanted = true;
                    }
                }
            }
            else
            {
                for (int g = 0; g < 3; ++g)
                {
                    const double ew = std::fabs(Get(w, g));
                    anySlanted = anySlanted || (ew >= 1e-12 && ew <= 1.0 - 1e-12);
                }
            }

            // Slanted rays: short steps, as the voxels are cut.
            if (anySlanted && !(!kCut && touched.load()))
            {
                const Vec3 d = to - from;
                const double axial = Dot(d, w);
                const double across = Length(d - w * axial);
                const double reach = (std::max)(profile.radius, s);
                const int axialSteps = (across < 1e-9) ? 1 : static_cast<int>(std::ceil(std::fabs(axial) / (0.5 * s)));
                const int acrossSteps = static_cast<int>(std::ceil(across / (2.0 * reach)));
                const int steps = (std::max)({ 1, axialSteps, acrossSteps });
                for (int n = 0; n < steps && !(!kCut && touched.load()); ++n)
                {
                    const Vec3 a = from + d * (static_cast<double>(n) / steps);
                    const Vec3 b = from + d * (static_cast<double>(n + 1) / steps);
                    const Sweep step = MakeSweep(profile, w, a, b);
                    const double r = reachOf((std::min)(step.za, step.zb));
                    if (r < 0.0)
                        continue;
                    Vec3 lo, hi;
                    boxOf(a, b, r, lo, hi);
                    for (int g = 0; g < 3; ++g)
                    {
                        const double ew = std::fabs(Get(w, g));
                        if (ew < 1e-12 || ew > 1.0 - 1e-12)
                            continue;
                        const auto rowRange = [](double, int&, int&) { return true; };
                        const auto find = [&](const Vec3& o, const Vec3& e, double tLo, double tHi, double& t0,
                                              double& t1) { return Slanted(step, o, e, tLo, tHi, t0, t1); };
                        cutRays(g, lo, hi, rowRange, find);
                    }
                }
            }
            if (!kCut)
                return touched.load() ? 1.0 : 0.0;
            return result;
        }
    }

    std::vector<double> DexelResolutionStops(const StockBox& box, double maxRays)
    {
        const Vec3 size = box.upper - box.lower;
        std::vector<double> stops;
        for (double spacing : kSpacings)
        {
            const double nx = NodesAlong(size.x, spacing), ny = NodesAlong(size.y, spacing),
                         nz = NodesAlong(size.z, spacing);
            const double largest = (std::max)({ nx * ny, ny * nz, nz * nx });
            if (largest <= maxRays && stops.size() < 7)
                stops.push_back(spacing);
        }
        if (stops.empty())
            stops.push_back(kSpacings[sizeof(kSpacings) / sizeof(kSpacings[0]) - 1]);
        return stops;
    }

    /////////////////////////////////////////////////////////////////////////
    // TriDexelStock

    void TriDexelStock::Reset(const StockBox& box, double spacing)
    {
        m_box = box;
        m_lower = box.lower;
        m_spacing = spacing;
        const Vec3 size = box.upper - box.lower;
        for (int a = 0; a < 3; ++a)
        {
            m_length[a] = (std::max)(0.0, Get(size, a));
            m_n[a] = NodesAlong(m_length[a], spacing);
        }
        for (int a = 0; a < 3; ++a)
            m_rays[a].assign(static_cast<size_t>(m_n[U(a)]) * static_cast<size_t>(m_n[V(a)]), Node{ 1.0f, 0.0f, -1 });
        m_pool.clear();
        m_free.clear();
        m_dirty.assign(static_cast<size_t>(ChunkCount()), 1);
    }

    void TriDexelStock::FillAll()
    {
        m_pool.clear();
        m_free.clear();
        for (int a = 0; a < 3; ++a)
        {
            const int u = U(a), v = V(a);
            for (int iv = 0; iv < m_n[v]; ++iv)
            {
                for (int iu = 0; iu < m_n[u]; ++iu)
                {
                    Node& node = m_rays[a][RayIndex(a, iu, iv)];
                    const bool inBox = (iu + 0.5) * m_spacing < m_length[u] && (iv + 0.5) * m_spacing < m_length[v];
                    node = inBox ? Node{ 0.0f, static_cast<float>(m_length[a]), -1 } : Node{ 1.0f, 0.0f, -1 };
                }
            }
        }
        MarkAllDirty();
    }

    void TriDexelStock::FillMesh(const std::vector<Vec3>& triangles)
    {
        m_pool.clear();
        m_free.clear();
        for (int a = 0; a < 3; ++a)
        {
            std::fill(m_rays[a].begin(), m_rays[a].end(), Node{ 1.0f, 0.0f, -1 });
            const int u = U(a), v = V(a);
            const double lowerU = Get(m_lower, u), lowerV = Get(m_lower, v), lowerA = Get(m_lower, a);

            // The triangles crossing each row of rays.
            std::vector<std::vector<int>> rows(static_cast<size_t>(m_n[v]));
            for (size_t t = 0; t + 2 < triangles.size(); t += 3)
            {
                const double p = Get(triangles[t], v), q = Get(triangles[t + 1], v), r = Get(triangles[t + 2], v);
                const double minV = (std::min)({ p, q, r }), maxV = (std::max)({ p, q, r });
                const int j0 = (std::max)(0, static_cast<int>(std::floor((minV - lowerV) / m_spacing - 0.5)));
                const int j1 = (std::min)(m_n[v] - 1, static_cast<int>(std::ceil((maxV - lowerV) / m_spacing - 0.5)));
                for (int j = j0; j <= j1; ++j)
                    rows[static_cast<size_t>(j)].push_back(static_cast<int>(t));
            }

            // A row at a time: where each ray crosses the mesh, nudged a little so it
            // doesn't run exactly along an edge.
            const double nudgeU = m_spacing * 1.37e-6;
            const double nudgeV = m_spacing * 0.71e-6;
            std::vector<std::vector<double>> hits(static_cast<size_t>(m_n[u]));
            std::vector<Interval> list;
            for (int iv = 0; iv < m_n[v]; ++iv)
            {
                if (rows[static_cast<size_t>(iv)].empty())
                    continue;
                for (auto& ray : hits)
                    ray.clear();
                const double y = lowerV + (iv + 0.5) * m_spacing + nudgeV;
                for (int t : rows[static_cast<size_t>(iv)])
                {
                    const Vec3& pa = triangles[static_cast<size_t>(t)];
                    const Vec3& pb = triangles[static_cast<size_t>(t) + 1];
                    const Vec3& pc = triangles[static_cast<size_t>(t) + 2];
                    const double ax = Get(pa, u), ay = Get(pa, v), bx = Get(pb, u), by = Get(pb, v), cx = Get(pc, u),
                                 cy = Get(pc, v);
                    const double area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
                    if (std::fabs(area) < 1e-18)
                        continue;
                    const double minX = (std::min)({ ax, bx, cx }), maxX = (std::max)({ ax, bx, cx });
                    const int i0 = (std::max)(0, static_cast<int>(std::floor((minX - lowerU) / m_spacing - 0.5)));
                    const int i1 =
                        (std::min)(m_n[u] - 1, static_cast<int>(std::ceil((maxX - lowerU) / m_spacing - 0.5)));
                    for (int iu = i0; iu <= i1; ++iu)
                    {
                        const double x = lowerU + (iu + 0.5) * m_spacing + nudgeU;
                        const double wa = ((bx - x) * (cy - y) - (cx - x) * (by - y)) / area;
                        const double wb = ((cx - x) * (ay - y) - (ax - x) * (cy - y)) / area;
                        const double wc = 1.0 - wa - wb;
                        if (wa < 0.0 || wb < 0.0 || wc < 0.0)
                            continue;
                        hits[static_cast<size_t>(iu)].push_back(wa * Get(pa, a) + wb * Get(pb, a) + wc * Get(pc, a) -
                                                                lowerA);
                    }
                }
                for (int iu = 0; iu < m_n[u]; ++iu)
                {
                    std::vector<double>& ray = hits[static_cast<size_t>(iu)];
                    if (ray.size() < 2)
                        continue;
                    std::sort(ray.begin(), ray.end());
                    list.clear();
                    for (size_t h = 0; h + 1 < ray.size(); h += 2)
                    {
                        const double e = (std::max)(0.0, ray[h]);
                        const double x = (std::min)(m_length[a], ray[h + 1]);
                        if (x - e > kSliver)
                            list.push_back({ static_cast<float>(e), static_cast<float>(x) });
                    }
                    SetIntervals(a, RayIndex(a, iu, iv), list.data(), list.size());
                }
            }
        }
        MarkAllDirty();
    }

    std::int32_t TriDexelStock::Allocate()
    {
        if (!m_free.empty())
        {
            const std::int32_t index = m_free.back();
            m_free.pop_back();
            return index;
        }
        m_pool.push_back(Node{ 1.0f, 0.0f, -1 });
        return static_cast<std::int32_t>(m_pool.size() - 1);
    }

    void TriDexelStock::SetIntervals(int axis, size_t ray, const Interval* list, size_t count)
    {
        Node& head = m_rays[axis][ray];
        for (std::int32_t m = head.more; m >= 0;)
        {
            const std::int32_t next = m_pool[static_cast<size_t>(m)].more;
            m_free.push_back(m);
            m = next;
        }
        if (count == 0)
        {
            head = Node{ 1.0f, 0.0f, -1 };
            return;
        }
        head = Node{ list[0].enter, list[0].exit, -1 };
        std::int32_t last = -1;
        for (size_t n = 1; n < count; ++n)
        {
            const std::int32_t index = Allocate();
            m_pool[static_cast<size_t>(index)] = Node{ list[n].enter, list[n].exit, -1 };
            if (last < 0)
                m_rays[axis][ray].more = index;
            else
                m_pool[static_cast<size_t>(last)].more = index;
            last = index;
        }
    }

    void TriDexelStock::Intervals(int axis, size_t ray, std::vector<Interval>& out) const
    {
        out.clear();
        Walk(axis, ray, [&](float e, float x) {
            out.push_back({ e, x });
            return true;
        });
    }

    double TriDexelStock::Overlap(int axis, size_t ray, double a, double b) const
    {
        double total = 0.0;
        Walk(axis, ray, [&](float e, float x) {
            const double lo = (std::max)(a, static_cast<double>(e)), hi = (std::min)(b, static_cast<double>(x));
            if (hi > lo)
                total += hi - lo;
            return e < b;
        });
        return total;
    }

    bool TriDexelStock::AnyStock(int axis, size_t ray, double a, double b) const
    {
        bool any = false;
        Walk(axis, ray, [&](float e, float x) {
            if (x > a && e < b)
                any = true;
            return !any && e < b;
        });
        return any;
    }

    double TriDexelStock::Subtract(int axis, size_t ray, double a, double b)
    {
        if (!AnyStock(axis, ray, a, b))
            return 0.0;
        m_scratch.clear();
        double removed = 0.0;
        Walk(axis, ray, [&](float e, float x) {
            if (x <= a || e >= b)
            {
                m_scratch.push_back({ e, x });
                return true;
            }
            removed += (std::min)(b, static_cast<double>(x)) - (std::max)(a, static_cast<double>(e));
            if (a - e > kSliver)
                m_scratch.push_back({ e, static_cast<float>(a) });
            if (x - b > kSliver)
                m_scratch.push_back({ static_cast<float>(b), x });
            return true;
        });
        SetIntervals(axis, ray, m_scratch.data(), m_scratch.size());
        return removed;
    }

    bool TriDexelStock::Inside(int i, int j, int k) const
    {
        if (i < 0 || j < 0 || k < 0 || i >= m_n[0] || j >= m_n[1] || k >= m_n[2])
            return false;
        const double z = (k + 0.5) * m_spacing;
        bool inside = false;
        Walk(2, RayIndex(2, i, j), [&](float e, float x) {
            if (z >= e && z < x)
                inside = true;
            return !inside && e <= z;
        });
        return inside;
    }

    double TriDexelStock::Crossing(int axis, size_t ray, double a, double b, bool leaving) const
    {
        double found = -1.0;
        Walk(axis, ray, [&](float e, float x) {
            const double end = leaving ? x : e;
            if (end >= a && end <= b)
            {
                found = end;
                return false;
            }
            return e <= b;
        });
        return found;
    }

    double TriDexelStock::Volume() const
    {
        double length = 0.0;
        for (size_t ray = 0; ray < m_rays[2].size(); ++ray)
        {
            Walk(2, ray, [&](float e, float x) {
                length += static_cast<double>(x) - e;
                return true;
            });
        }
        return length * m_spacing * m_spacing;
    }

    size_t TriDexelStock::IntervalCount() const
    {
        size_t count = 0;
        for (int a = 0; a < 3; ++a)
        {
            for (const Node& node : m_rays[a])
                count += Empty(node) ? 0 : 1;
        }
        return count + m_pool.size() - m_free.size();
    }

    size_t TriDexelStock::MemoryBytes() const
    {
        return (m_rays[0].capacity() + m_rays[1].capacity() + m_rays[2].capacity() + m_pool.capacity()) * sizeof(Node) +
               m_free.capacity() * sizeof(std::int32_t) + m_dirty.capacity();
    }

    void TriDexelStock::ToVoxels(VoxelStock& voxels) const
    {
        voxels.Reset(m_box, m_spacing, true);
        const int nz = voxels.SizeZ();
        for (int j = 0; j < (std::min)(m_n[1], voxels.SizeY()); ++j)
        {
            for (int i = 0; i < (std::min)(m_n[0], voxels.SizeX()); ++i)
            {
                Walk(2, RayIndex(2, i, j), [&](float e, float x) {
                    const int k0 = (std::max)(0, static_cast<int>(std::ceil(e / m_spacing - 0.5)));
                    const int k1 = (std::min)(nz - 1, static_cast<int>(std::ceil(x / m_spacing - 0.5)) - 1);
                    for (int w = k0 / VoxelStock::kWord; k0 <= k1 && w <= k1 / VoxelStock::kWord; ++w)
                        voxels.SetCells(i, j, w, VoxelStock::SpanMask(w, k0, k1));
                    return true;
                });
            }
        }
        voxels.Settle();
    }

    void TriDexelStock::MarkAllDirty()
    {
        std::fill(m_dirty.begin(), m_dirty.end(), 1);
    }

    void TriDexelStock::MarkDirtyNodes(int i0, int j0, int k0, int i1, int j1, int k1)
    {
        const int from[3] = { i0, j0, k0 };
        const int to[3] = { i1, j1, k1 };
        int c0[3], c1[3];
        for (int a = 0; a < 3; ++a)
        {
            const int lo = (std::max)(0, from[a] - 1);
            const int hi = (std::min)(m_n[a] - 1, to[a] + 1);
            if (lo > hi)
                return;
            c0[a] = lo / kChunk;
            c1[a] = hi / kChunk;
        }
        const int cx = ChunksAlong(0), cy = ChunksAlong(1);
        for (int z = c0[2]; z <= c1[2]; ++z)
        {
            for (int y = c0[1]; y <= c1[1]; ++y)
            {
                for (int x = c0[0]; x <= c1[0]; ++x)
                    m_dirty[static_cast<size_t>((z * cy + y) * cx + x)] = 1;
            }
        }
    }

    void TriDexelStock::MarkDirtyRay(int axis, int iu, int iv, double a, double b)
    {
        int from[3], to[3];
        from[axis] = static_cast<int>(std::floor(a / m_spacing - 0.5));
        to[axis] = static_cast<int>(std::ceil(b / m_spacing - 0.5));
        from[U(axis)] = to[U(axis)] = iu;
        from[V(axis)] = to[V(axis)] = iv;
        MarkDirtyNodes(from[0], from[1], from[2], to[0], to[1], to[2]);
    }

    /////////////////////////////////////////////////////////////////////////
    // Cutting

    double CutDexels(TriDexelStock& stock, const ToolProfile& profile, const Vec3& from, const Vec3& to,
                     const Vec3& axis)
    {
        return SweepDexels<true>(stock, profile, from, to, axis, 0.0);
    }

    bool DexelsTouch(const TriDexelStock& stock, const ToolProfile& profile, const Vec3& from, const Vec3& to,
                     const Vec3& axis, double tolerance)
    {
        // Touching only reads the stock.
        return SweepDexels<false>(const_cast<TriDexelStock&>(stock), profile, from, to, axis, tolerance) > 0.0;
    }

    /////////////////////////////////////////////////////////////////////////
    // Meshing

    StockMesh MeshDexelChunk(const TriDexelStock& stock, int chunk, bool mergeFlats)
    {
        StockMesh mesh;
        const int chunks[3] = { stock.ChunksAlong(0), stock.ChunksAlong(1), stock.ChunksAlong(2) };
        const int c[3] = { chunk % chunks[0], (chunk / chunks[0]) % chunks[1], chunk / (chunks[0] * chunks[1]) };
        const int n[3] = { stock.Size(0), stock.Size(1), stock.Size(2) };
        const double s = stock.Spacing();
        const Vec3 lower = stock.Lower();

        // The lattice edges this chunk owns start at nodes lo to hi; the first chunk
        // along an axis also owns those from the node before the lattice.
        int lo[3], hi[3];
        for (int a = 0; a < 3; ++a)
        {
            lo[a] = c[a] * TriDexelStock::kChunk - ((c[a] == 0) ? 1 : 0);
            hi[a] = (std::min)(n[a] - 1, c[a] * TriDexelStock::kChunk + TriDexelStock::kChunk - 1);
            if (lo[a] > hi[a])
                return mesh;
        }

        // Which nodes are inside, from lo - 1 to hi + 1.
        int nodeBase[3], nodeSize[3];
        for (int a = 0; a < 3; ++a)
        {
            nodeBase[a] = lo[a] - 1;
            nodeSize[a] = hi[a] - lo[a] + 3;
        }
        std::vector<char> inside(static_cast<size_t>(nodeSize[0]) * nodeSize[1] * nodeSize[2], 0);
        const auto nodeAt = [&](int i, int j, int k) {
            return (static_cast<size_t>(k - nodeBase[2]) * nodeSize[1] + (j - nodeBase[1])) * nodeSize[0] +
                   (i - nodeBase[0]);
        };
        {
            std::vector<TriDexelStock::Interval> intervals;
            const int k0 = (std::max)(0, nodeBase[2]);
            const int k1 = (std::min)(n[2] - 1, nodeBase[2] + nodeSize[2] - 1);
            for (int j = (std::max)(0, nodeBase[1]); j <= (std::min)(n[1] - 1, nodeBase[1] + nodeSize[1] - 1); ++j)
            {
                for (int i = (std::max)(0, nodeBase[0]); i <= (std::min)(n[0] - 1, nodeBase[0] + nodeSize[0] - 1); ++i)
                {
                    stock.Intervals(2, stock.RayIndex(2, i, j), intervals);
                    for (const TriDexelStock::Interval& interval : intervals)
                    {
                        // Node k is inside if enter <= (k + 0.5) s < exit.
                        const int a = (std::max)(k0, static_cast<int>(std::ceil(interval.enter / s - 0.5)));
                        const int b = (std::min)(k1, static_cast<int>(std::ceil(interval.exit / s - 0.5)) - 1);
                        for (int k = a; k <= b; ++k)
                            inside[nodeAt(i, j, k)] = 1;
                    }
                }
            }
        }
        const auto in = [&](int i, int j, int k) { return inside[nodeAt(i, j, k)] != 0; };

        // A vertex in each cell the surface passes through, from lo - 1 to hi.
        int cellSize[3];
        for (int a = 0; a < 3; ++a)
            cellSize[a] = hi[a] - lo[a] + 2;
        std::vector<int> vertexOf(static_cast<size_t>(cellSize[0]) * cellSize[1] * cellSize[2], -1);
        std::vector<double> normalSum;
        const auto vertex = [&](int a, int b, int cc) {
            int& index = vertexOf[(static_cast<size_t>(cc - lo[2] + 1) * cellSize[1] + (b - lo[1] + 1)) * cellSize[0] +
                                  (a - lo[0] + 1)];
            if (index >= 0)
                return index;
            const int cell[3] = { a, b, cc };
            Vec3 sum;
            int count = 0;
            for (int g = 0; g < 3; ++g)
            {
                const int u = TriDexelStock::U(g), v = TriDexelStock::V(g);
                for (int du = 0; du < 2; ++du)
                {
                    for (int dv = 0; dv < 2; ++dv)
                    {
                        int p[3], q[3];
                        p[g] = cell[g];
                        p[u] = cell[u] + du;
                        p[v] = cell[v] + dv;
                        q[0] = p[0];
                        q[1] = p[1];
                        q[2] = p[2];
                        q[g] += 1;
                        const bool pIn = in(p[0], p[1], p[2]);
                        if (pIn == in(q[0], q[1], q[2]))
                            continue;
                        // The ray along this edge, where it crosses between the two nodes.
                        const double t0 = (p[g] + 0.5) * s, t1 = (q[g] + 0.5) * s;
                        double t = stock.Crossing(g, stock.RayIndex(g, p[u], p[v]), t0, t1, pIn);
                        if (t < 0.0)
                            t = 0.5 * (t0 + t1);
                        Vec3 point = lower;
                        Set(point, g, Get(lower, g) + t);
                        Set(point, u, Get(lower, u) + (p[u] + 0.5) * s);
                        Set(point, v, Get(lower, v) + (p[v] + 0.5) * s);
                        sum = sum + point;
                        ++count;
                    }
                }
            }
            const Vec3 at = sum * (1.0 / (std::max)(1, count));
            index = static_cast<int>(mesh.vertices.size() / 3);
            mesh.vertices.insert(mesh.vertices.end(),
                                 { static_cast<float>(at.x), static_cast<float>(at.y), static_cast<float>(at.z) });
            normalSum.insert(normalSum.end(), { 0.0, 0.0, 0.0 });
            return index;
        };
        const auto position = [&](int index) {
            const size_t at = static_cast<size_t>(index) * 3;
            return Vec3{ mesh.vertices[at], mesh.vertices[at + 1], mesh.vertices[at + 2] };
        };
        const auto triangle = [&](int a, int b, int cc) {
            mesh.faces.insert(mesh.faces.end(), { 3, a, b, cc });
            const Vec3 normal = Cross(position(b) - position(a), position(cc) - position(a));
            for (int corner : { a, b, cc })
            {
                normalSum[static_cast<size_t>(corner) * 3] += normal.x;
                normalSum[static_cast<size_t>(corner) * 3 + 1] += normal.y;
                normalSum[static_cast<size_t>(corner) * 3 + 2] += normal.z;
            }
        };
        const auto quad = [&](bool outwardPositive, int v0, int v1, int v2, int v3) {
            if (outwardPositive)
            {
                triangle(v0, v1, v2);
                triangle(v0, v2, v3);
            }
            else
            {
                triangle(v0, v2, v1);
                triangle(v0, v3, v2);
            }
        };

        // A quad across each edge the surface crosses, facing from inside out. Quads
        // lying flat on a lattice plane, their vertices at their cells' middles, are
        // kept to be merged into rectangles.
        struct Flat
        {
            int axis;
            bool positive;
            float height;
            int u;
            int v;
        };
        std::vector<Flat> flats;
        const auto edgeQuad = [&](int g, bool here, const int node[3]) {
            const int u = TriDexelStock::U(g), v = TriDexelStock::V(g);
            // The four cells round the edge, anticlockwise seen from the edge's +g end.
            const int du[4] = { -1, 0, 0, -1 };
            const int dv[4] = { -1, -1, 0, 0 };
            int corners[4];
            bool flat = true;
            for (int q = 0; q < 4; ++q)
            {
                int cell[3];
                cell[g] = node[g];
                cell[u] = node[u] + du[q];
                cell[v] = node[v] + dv[q];
                corners[q] = vertex(cell[0], cell[1], cell[2]);
                const Vec3 p = position(corners[q]);
                flat = flat && std::fabs(Get(p, g) - Get(position(corners[0]), g)) < 1e-6 &&
                       std::fabs(Get(p, u) - (Get(lower, u) + (cell[u] + 1) * s)) < 1e-4 &&
                       std::fabs(Get(p, v) - (Get(lower, v) + (cell[v] + 1) * s)) < 1e-4;
            }
            if (flat && mergeFlats)
            {
                flats.push_back({ g, here, static_cast<float>(Get(position(corners[0]), g)), node[u], node[v] });
                return;
            }
            quad(here, corners[0], corners[1], corners[2], corners[3]);
        };
        for (int k = lo[2]; k <= hi[2]; ++k)
        {
            for (int j = lo[1]; j <= hi[1]; ++j)
            {
                for (int i = lo[0]; i <= hi[0]; ++i)
                {
                    const int node[3] = { i, j, k };
                    const bool here = in(i, j, k);
                    if (here != in(i + 1, j, k))
                        edgeQuad(0, here, node);
                    if (here != in(i, j + 1, k))
                        edgeQuad(1, here, node);
                    if (here != in(i, j, k + 1))
                        edgeQuad(2, here, node);
                }
            }
        }

        // The flat quads of each plane and facing, merged greedily into rectangles.
        std::sort(flats.begin(), flats.end(), [](const Flat& a, const Flat& b) {
            return std::tie(a.axis, a.positive, a.height, a.v, a.u) < std::tie(b.axis, b.positive, b.height, b.v, b.u);
        });
        for (size_t first = 0; first < flats.size();)
        {
            size_t last = first;
            while (last < flats.size() && flats[last].axis == flats[first].axis &&
                   flats[last].positive == flats[first].positive && flats[last].height == flats[first].height)
                ++last;
            const int g = flats[first].axis, u = TriDexelStock::U(g), v = TriDexelStock::V(g);
            const int u0 = lo[u], v0 = lo[v];
            const int nu = hi[u] - lo[u] + 1, nv = hi[v] - lo[v] + 1;
            std::vector<char> grid(static_cast<size_t>(nu) * nv, 0);
            for (size_t f = first; f < last; ++f)
                grid[static_cast<size_t>(flats[f].v - v0) * nu + (flats[f].u - u0)] = 1;
            const auto addVertex = [&](double cu, double cv) {
                Vec3 p;
                Set(p, g, flats[first].height);
                Set(p, u, Get(lower, u) + cu * s);
                Set(p, v, Get(lower, v) + cv * s);
                const int index = static_cast<int>(mesh.vertices.size() / 3);
                mesh.vertices.insert(mesh.vertices.end(),
                                     { static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z) });
                Vec3 normal;
                Set(normal, g, flats[first].positive ? 1.0 : -1.0);
                normalSum.insert(normalSum.end(), { normal.x, normal.y, normal.z });
                return index;
            };
            for (int b = 0; b < nv; ++b)
            {
                for (int a = 0; a < nu; ++a)
                {
                    if (!grid[static_cast<size_t>(b) * nu + a])
                        continue;
                    int a1 = a;
                    while (a1 + 1 < nu && grid[static_cast<size_t>(b) * nu + a1 + 1])
                        ++a1;
                    int b1 = b;
                    for (bool grow = true; grow && b1 + 1 < nv;)
                    {
                        for (int x = a; x <= a1 && grow; ++x)
                            grow = grid[static_cast<size_t>(b1 + 1) * nu + x] != 0;
                        if (grow)
                            ++b1;
                    }
                    for (int y = b; y <= b1; ++y)
                    {
                        for (int x = a; x <= a1; ++x)
                            grid[static_cast<size_t>(y) * nu + x] = 0;
                    }
                    // A quad of edge (u, v) spans u * s to (u + 1) * s from the lattice's corner.
                    const double cu0 = a + u0, cu1 = a1 + u0 + 1, cv0 = b + v0, cv1 = b1 + v0 + 1;
                    const int r0 = addVertex(cu0, cv0), r1 = addVertex(cu1, cv0), r2 = addVertex(cu1, cv1),
                              r3 = addVertex(cu0, cv1);
                    if (flats[first].positive)
                        mesh.faces.insert(mesh.faces.end(), { 3, r0, r1, r2, 3, r0, r2, r3 });
                    else
                        mesh.faces.insert(mesh.faces.end(), { 3, r0, r2, r1, 3, r0, r3, r2 });
                }
            }
            first = last;
        }

        // Only the vertices the faces use, with their normals.
        std::vector<int> renumber(mesh.vertices.size() / 3, -1);
        StockMesh used;
        for (size_t f = 0; f + 3 < mesh.faces.size(); f += 4)
        {
            used.faces.push_back(3);
            for (int c = 1; c <= 3; ++c)
            {
                const int old = mesh.faces[f + c];
                int& index = renumber[static_cast<size_t>(old)];
                if (index < 0)
                {
                    index = static_cast<int>(used.vertices.size() / 3);
                    const size_t at = static_cast<size_t>(old) * 3;
                    used.vertices.insert(used.vertices.end(),
                                         { mesh.vertices[at], mesh.vertices[at + 1], mesh.vertices[at + 2] });
                    Vec3 unit;
                    if (!Normalise(Vec3{ normalSum[at], normalSum[at + 1], normalSum[at + 2] }, unit))
                        unit = Vec3{ 0.0, 0.0, 1.0 };
                    used.normals.insert(used.normals.end(), { static_cast<float>(unit.x), static_cast<float>(unit.y),
                                                              static_cast<float>(unit.z) });
                }
                used.faces.push_back(index);
            }
        }
        return used;
    }

    void MeshDexelChunks(const TriDexelStock& stock, const std::vector<int>& chunks, std::vector<StockMesh>& meshes,
                         bool mergeFlats)
    {
        meshes.resize((std::max)(meshes.size(), static_cast<size_t>(stock.ChunkCount())));
        std::for_each(std::execution::par, chunks.begin(), chunks.end(), [&](int chunk) {
            meshes[static_cast<size_t>(chunk)] = MeshDexelChunk(stock, chunk, mergeFlats);
        });
    }

    std::vector<int> DirtyDexelChunks(const TriDexelStock& stock)
    {
        std::vector<int> chunks;
        for (int c = 0; c < stock.ChunkCount(); ++c)
        {
            if (stock.ChunkDirty(c))
                chunks.push_back(c);
        }
        return chunks;
    }

    StockMesh MeshDexelStock(const TriDexelStock& stock, bool mergeFlats)
    {
        std::vector<int> all(static_cast<size_t>(stock.ChunkCount()));
        std::iota(all.begin(), all.end(), 0);
        std::vector<StockMesh> chunks;
        MeshDexelChunks(stock, all, chunks, mergeFlats);
        return JoinMeshes(chunks);
    }
}
