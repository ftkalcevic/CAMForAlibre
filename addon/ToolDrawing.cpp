// ToolDrawing.cpp : the Tool dialog's drawing of what each measurement means. Plain
// GDI, in design units with Y up from the tool's tip and X out from its axis.

#include "stdafx.h"
#include "ToolDrawing.h"

#include <algorithm>
#include <cmath>
#include <vector>

using camcore::Tool;
using camcore::ToolType;

namespace
{
    const double kPi = 3.14159265358979323846;

    // The drawing's layout. The tool itself is drawn to its own proportions, as
    // long as kLength or as wide as kMaxRadius either side, whichever comes first;
    // the dimensions, arrows and letters keep their own sizes.
    const double kDesignWidth = 200.0;
    const double kAxisX = 90.0;             // the tool's axis, from the drawing's left
    const double kLength = 380.0;           // the longest a tool is drawn
    const double kMaxRadius = 50.0;         // the widest, either side of its axis
    const double kShortDimension = 44.0;    // shorter than this, a dimension's arrows go outside

    const int kSupersample = 3;             // drawn larger then shrunk, to smooth the lines

    const COLORREF kPaper = RGB(255, 255, 255);
    const COLORREF kToolFill = RGB(226, 230, 236);
    const COLORREF kToolLine = RGB(40, 40, 40);
    const COLORREF kFluteLine = RGB(150, 156, 166);
    const COLORREF kDimension = RGB(0, 84, 166);

    struct P
    {
        double x, y;
    };

    P Polar(const P& centre, double radius, double degrees)
    {
        const double a = degrees * kPi / 180.0;
        return { centre.x + radius * std::cos(a), centre.y + radius * std::sin(a) };
    }

    double Clamp(double value, double low, double high)
    {
        return (std::max)(low, (std::min)(high, value));
    }

    class Canvas
    {
    public:
        Canvas(HDC hdc, int width, int height, double designTop, double designBottom) : m_hdc(hdc)
        {
            m_scale = (std::min)(width / kDesignWidth, height / (designTop - designBottom));
            m_left = (width - kDesignWidth * m_scale) / 2.0 + kAxisX * m_scale;
            m_top = (height - (designTop - designBottom) * m_scale) / 2.0 + designTop * m_scale;
            m_thin = kSupersample;
            m_thick = m_thin * 2;
            m_font = CreateFontW(-static_cast<int>(16.0 * m_scale), 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                                 DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY,
                                 DEFAULT_PITCH | FF_SWISS, L"Arial");
        }

        ~Canvas()
        {
            DeleteObject(m_font);
        }

        POINT At(const P& p) const
        {
            return { static_cast<LONG>(std::lround(m_left + p.x * m_scale)),
                     static_cast<LONG>(std::lround(m_top - p.y * m_scale)) };
        }

        void Polyline(const std::vector<P>& points, COLORREF colour, bool thick = false)
        {
            std::vector<POINT> device;
            for (const P& p : points)
                device.push_back(At(p));
            HPEN pen = CreatePen(PS_SOLID, thick ? m_thick : m_thin, colour);
            HGDIOBJ old = SelectObject(m_hdc, pen);
            ::Polyline(m_hdc, device.data(), static_cast<int>(device.size()));
            SelectObject(m_hdc, old);
            DeleteObject(pen);
        }

        void Line(const P& a, const P& b, COLORREF colour = kDimension)
        {
            Polyline({ a, b }, colour);
        }

        // A filled arrowhead with its point at tip, coming from the direction of `from`.
        void Arrow(const P& tip, const P& from)
        {
            const double dx = from.x - tip.x, dy = from.y - tip.y;
            const double length = std::hypot(dx, dy);
            if (length <= 0.0)
                return;
            const double ux = dx / length, uy = dy / length;
            const double size = 9.0, half = 3.2;
            const POINT points[3] = {
                At(tip),
                At({ tip.x + ux * size - uy * half, tip.y + uy * size + ux * half }),
                At({ tip.x + ux * size + uy * half, tip.y + uy * size - ux * half }),
            };
            HBRUSH brush = CreateSolidBrush(kDimension);
            HPEN pen = CreatePen(PS_SOLID, 1, kDimension);
            HGDIOBJ oldBrush = SelectObject(m_hdc, brush);
            HGDIOBJ oldPen = SelectObject(m_hdc, pen);
            ::Polygon(m_hdc, points, 3);
            SelectObject(m_hdc, oldBrush);
            SelectObject(m_hdc, oldPen);
            DeleteObject(brush);
            DeleteObject(pen);
        }

        // The letter centred at p, on a patch of paper so lines don't run through it.
        void Letter(const P& p, wchar_t letter)
        {
            HGDIOBJ oldFont = SelectObject(m_hdc, m_font);
            SIZE size = {};
            GetTextExtentPoint32W(m_hdc, &letter, 1, &size);
            const POINT at = At(p);
            const int pad = kSupersample * 2;
            const RECT patch = { at.x - size.cx / 2 - pad, at.y - size.cy / 2, at.x + size.cx / 2 + pad,
                                 at.y + size.cy / 2 };
            HBRUSH paper = CreateSolidBrush(kPaper);
            FillRect(m_hdc, &patch, paper);
            DeleteObject(paper);
            SetBkMode(m_hdc, TRANSPARENT);
            SetTextColor(m_hdc, kDimension);
            TextOutW(m_hdc, at.x - size.cx / 2, at.y - size.cy / 2, &letter, 1);
            SelectObject(m_hdc, oldFont);
        }

        // A dimension from a to b, arrows at both ends, with the letter half way
        // along. Too short for that, the arrows point in from outside and the
        // letter goes past b.
        void Dimension(const P& a, const P& b, wchar_t letter)
        {
            const double length = std::hypot(b.x - a.x, b.y - a.y);
            if (length >= kShortDimension)
            {
                Line(a, b);
                Arrow(a, b);
                Arrow(b, a);
                Letter({ (a.x + b.x) / 2.0, (a.y + b.y) / 2.0 }, letter);
                return;
            }
            const P u = (length > 1e-9) ? P{ (b.x - a.x) / length, (b.y - a.y) / length } : P{ 0.0, 1.0 };
            const double out = 20.0;
            Line({ a.x - u.x * out, a.y - u.y * out }, { b.x + u.x * out, b.y + u.y * out });
            Arrow(a, { a.x - u.x, a.y - u.y });
            Arrow(b, { b.x + u.x, b.y + u.y });
            Letter({ b.x + u.x * (out + 12.0), b.y + u.y * (out + 12.0) }, letter);
        }

        // An arc about centre, from one angle to another in degrees, with its letter
        // at the middle and, if there's room, arrows at its ends.
        void Angle(const P& centre, double radius, double from, double to, wchar_t letter, bool arrows = true)
        {
            std::vector<P> arc;
            for (int i = 0; i <= 32; ++i)
                arc.push_back(Polar(centre, radius, from + (to - from) * i / 32));
            Polyline(arc, kDimension);
            if (arrows)
            {
                Arrow(arc.front(), arc[4]);
                Arrow(arc.back(), arc[arc.size() - 5]);
            }
            if (letter != 0)
                Letter(Polar(centre, radius, (from + to) / 2.0), letter);
        }

        // The tool: filled, with flute lines below fluteTop, then its outline.
        void Tool(const std::vector<P>& right, bool slantedFlutes, double fluteTop, double width, double bottom)
        {
            std::vector<P> outline = right;
            for (auto it = right.rbegin(); it != right.rend(); ++it)
                outline.push_back({ -it->x, it->y });

            std::vector<POINT> device;
            for (const P& p : outline)
                device.push_back(At(p));

            HBRUSH brush = CreateSolidBrush(kToolFill);
            HPEN none = CreatePen(PS_NULL, 0, 0);
            HGDIOBJ oldBrush = SelectObject(m_hdc, brush);
            HGDIOBJ oldPen = SelectObject(m_hdc, none);
            ::Polygon(m_hdc, device.data(), static_cast<int>(device.size()));
            SelectObject(m_hdc, oldBrush);
            SelectObject(m_hdc, oldPen);
            DeleteObject(brush);
            DeleteObject(none);

            if (slantedFlutes && fluteTop > 0.0)
            {
                HRGN shape = CreatePolygonRgn(device.data(), static_cast<int>(device.size()), WINDING);
                const POINT a = At({ -kDesignWidth, fluteTop });
                const POINT b = At({ kDesignWidth, bottom });
                HRGN below = CreateRectRgn(a.x, a.y, b.x, b.y);
                CombineRgn(shape, shape, below, RGN_AND);
                SelectClipRgn(m_hdc, shape);
                const double rise = width + 4.0;
                for (double y = -rise; y < fluteTop + rise; y += 26.0)
                    Polyline({ { -width - 4.0, y }, { width + 4.0, y + rise } }, kFluteLine);
                SelectClipRgn(m_hdc, nullptr);
                DeleteObject(shape);
                DeleteObject(below);
            }

            outline.push_back(outline.front());
            Polyline(outline, kToolLine, true);
        }

    private:
        HDC m_hdc;
        HFONT m_font = nullptr;
        double m_scale = 1.0, m_left = 0.0, m_top = 0.0;
        int m_thin = 1, m_thick = 2;
    };

    // Where a point's angle is drawn: its flanks carried on past the (perhaps cut
    // off) apex, and the angle between them underneath. Returns its lowest point.
    double PointAngleBottom(double tip, double halfAngle)
    {
        const double a = halfAngle * kPi / 180.0;
        return -tip / std::tan(a) - 40.0;
    }

    void DrawPointAngle(Canvas& canvas, double tip, double halfAngle, wchar_t letter)
    {
        const double a = halfAngle * kPi / 180.0;
        const P apex = { 0.0, -tip / std::tan(a) };
        const double radius = 32.0;
        for (double side : { -1.0, 1.0 })
        {
            const P end = { side * (radius + 8.0) * std::sin(a), apex.y - (radius + 8.0) * std::cos(a) };
            canvas.Line({ side * tip, 0.0 }, end);
        }
        canvas.Angle(apex, radius, -90.0 - halfAngle, -90.0 + halfAngle, letter);
    }

    // The tip's width, by a leader from the left.
    void DrawTipLeader(Canvas& canvas, double tip, double radius)
    {
        const P label = { -radius - 20.0, 0.0 };
        canvas.Line({ -tip, 0.0 }, label);
        canvas.Arrow({ -tip, 0.0 }, label);
        canvas.Letter(label, L'g');
    }
}

void DrawToolDiagram(HDC hdc, const RECT& rect, const Tool& tool)
{
    const int width = rect.right - rect.left;
    const int height = rect.bottom - rect.top;
    if (width <= 0 || height <= 0)
        return;

    // The tool's sizes, in its own units, made drawable: missing ones guessed.
    const ToolType type = tool.type;
    const double diameter = (tool.diameter > 0.0) ? tool.diameter : 6.0;
    const double fluteLength = (tool.fluteLength > 0.0) ? tool.fluteLength : 2.0 * diameter;
    const double overallLength = (std::max)(tool.overallLength, fluteLength);
    const bool ownShank = tool.shankDiameter > 0.0;
    const double shankDiameter = ownShank ? tool.shankDiameter : diameter;

    // One scale for every length: as long as kLength, or narrower than kMaxRadius.
    const double s = (std::min)(kLength / overallLength, kMaxRadius / ((std::max)(diameter, shankDiameter) / 2.0));
    const double radius = diameter / 2.0 * s;
    const double flute = fluteLength * s;
    const double length = overallLength * s;
    const double topRadius = shankDiameter / 2.0 * s;

    // The right side of the tool, tip to top.
    std::vector<P> right;
    double tip = 0.0;           // half the width at y = 0
    double widestY = 0.0;       // where the cutting diameter is first reached
    double low = -24.0;         // the lowest the type's own dimensions reach
    const double cornerRadius = Clamp(tool.cornerRadius, 0.0, diameter / 2.0) * s;
    const double tipRadius = Clamp(tool.tipDiameter / 2.0, 0.0, diameter / 2.0 * 0.999) * s;
    const double vHalf = Clamp(tool.includedAngle / 2.0, 1.0, 89.0);
    const double drillHalf = Clamp(tool.pointAngle / 2.0, 1.0, 89.5);
    const double taperAngle = Clamp(tool.taperAngle, 0.1, 89.0);

    // A thread mill: sharp V teeth a pitch apart from the tip, as many as it has
    // (up to 60), their depth from the pitch and thread angle.
    const int threadTeeth = (std::max)(1, (std::min)(60, tool.threadTeeth));
    const double threadPitch = ((tool.pitch > 0.0) ? tool.pitch : diameter / 4.0) * s;
    const double threadHalf = Clamp(tool.threadAngle / 2.0, 5.0, 85.0) * kPi / 180.0;
    const double threadRoot = (std::max)(radius * 0.25, radius - threadPitch / (2.0 * std::tan(threadHalf)));

    const auto point = [&](double t, double h) {
        tip = t;
        const double rise = (radius - t) / std::tan(h * kPi / 180.0);
        right.push_back({ t, 0.0 });
        right.push_back({ radius, rise });
        widestY = rise;
        low = (std::min)(low, PointAngleBottom(t, h));
    };
    switch (type)
    {
    case ToolType::BallMill:
        for (int i = 0; i <= 24; ++i)
        {
            const double a = i * (kPi / 2.0) / 24;
            right.push_back({ radius * std::sin(a), radius - radius * std::cos(a) });
        }
        widestY = radius;
        break;
    case ToolType::CornerRadiusMill:
        tip = radius - cornerRadius;
        for (int i = 0; i <= 16; ++i)
        {
            const double a = i * (kPi / 2.0) / 16;
            right.push_back({ tip + cornerRadius * std::sin(a), cornerRadius - cornerRadius * std::cos(a) });
        }
        widestY = cornerRadius;
        low = -34.0;
        break;
    case ToolType::VMill:
    case ToolType::ChamferMill:
        point(tipRadius, vHalf);
        break;
    case ToolType::Drill:
        point(0.0, drillHalf);
        break;
    case ToolType::TaperMill:
    {
        // Out from the tip at the taper angle, to the diameter or the flutes' top.
        tip = tipRadius;
        right.push_back({ tipRadius, 0.0 });
        const double slope = std::tan(taperAngle * kPi / 180.0);
        const double rise = (radius - tipRadius) / slope;
        if (rise <= flute)
        {
            right.push_back({ radius, rise });
            widestY = rise;
        }
        else
        {
            right.push_back({ tipRadius + flute * slope, flute });
            right.push_back({ radius, flute });
            widestY = flute;
        }
        low = -72.0;
        break;
    }
    case ToolType::ThreadMill:
    {
        tip = threadRoot;
        for (int i = 0; i < threadTeeth; ++i)
        {
            right.push_back({ threadRoot, i * threadPitch });
            right.push_back({ radius, (i + 0.5) * threadPitch });
        }
        // Above the teeth, the head at their root up to the flute length.
        const double teethTop = threadTeeth * threadPitch;
        right.push_back({ threadRoot, teethTop });
        if (flute > teethTop)
            right.push_back({ threadRoot, flute });
        if (!ownShank)
            right.push_back({ radius, (std::max)(flute, teethTop) });
        widestY = threadPitch / 2.0;
        break;
    }
    default:
        tip = radius;
        right.push_back({ radius, 0.0 });
        break;
    }

    // The shank, from the flutes' top (or whatever's higher) to the overall length.
    const double shankFrom = (std::max)(flute, right.empty() ? 0.0 : right.back().y);
    if (ownShank)
    {
        if (right.empty() || right.back().y < shankFrom)
            right.push_back({ radius, shankFrom });
        right.push_back({ topRadius, shankFrom });
    }
    right.push_back({ topRadius, (std::max)(length, shankFrom) });
    const double top = right.back().y;

    // The drawing's height: the tool, the shank's diameter above it, the
    // diameter's dimension below everything else.
    const double diameterY = (std::min)(-80.0, low - 22.0);
    const double designTop = top + (ownShank ? 44.0 : 20.0);
    const double designBottom = diameterY - 16.0;

    HDC memory = CreateCompatibleDC(hdc);
    HBITMAP bitmap = CreateCompatibleBitmap(hdc, width * kSupersample, height * kSupersample);
    HGDIOBJ oldBitmap = SelectObject(memory, bitmap);
    const RECT whole = { 0, 0, width * kSupersample, height * kSupersample };
    HBRUSH paper = CreateSolidBrush(kPaper);
    FillRect(memory, &whole, paper);
    DeleteObject(paper);

    {
        Canvas canvas(memory, whole.right, whole.bottom, designTop, designBottom);
        canvas.Tool(right, type != ToolType::ThreadMill, flute, radius, designBottom);

        // The diameter, below the tool.
        for (double side : { -1.0, 1.0 })
            canvas.Line({ side * radius, widestY }, { side * radius, diameterY - 5.0 });
        canvas.Dimension({ -radius, diameterY }, { radius, diameterY }, L'a');

        // The lengths, all from the tip, up the right side.
        const double widest = (std::max)(radius, topRadius);
        const double fluteX = widest + 18.0;
        const double shankX = fluteX + 16.0;
        const double overallX = ownShank ? shankX + 16.0 : fluteX + 16.0;
        canvas.Line({ tip, 0.0 }, { overallX + 5.0, 0.0 });
        canvas.Line({ radius, flute }, { fluteX + 5.0, flute });
        canvas.Dimension({ fluteX, 0.0 }, { fluteX, flute }, L'b');
        canvas.Line({ topRadius, top }, { overallX + 5.0, top });
        canvas.Dimension({ overallX, 0.0 }, { overallX, top }, L'c');
        if (ownShank)
        {
            canvas.Line({ widest, shankFrom }, { shankX + 5.0, shankFrom });
            canvas.Dimension({ shankX, shankFrom }, { shankX, top }, L'd');

            const double y = top + 24.0;
            for (double side : { -1.0, 1.0 })
                canvas.Line({ side * topRadius, top }, { side * topRadius, y + 5.0 });
            canvas.Dimension({ -topRadius, y }, { topRadius, y }, L'e');
        }

        // Each type's own.
        switch (type)
        {
        case ToolType::CornerRadiusMill:
        {
            const double a = kPi / 4.0;
            const P on = { -(tip + cornerRadius * std::sin(a)), cornerRadius - cornerRadius * std::cos(a) };
            const P label = { -radius - 24.0, -24.0 };
            canvas.Line(on, label);
            canvas.Arrow(on, label);
            canvas.Letter(label, L'f');
            break;
        }
        case ToolType::VMill:
        case ToolType::ChamferMill:
            DrawTipLeader(canvas, tipRadius, radius);
            DrawPointAngle(canvas, tipRadius, vHalf, L'h');
            break;
        case ToolType::Drill:
            DrawPointAngle(canvas, 0.0, drillHalf, L'j');
            break;
        case ToolType::TaperMill:
        {
            DrawTipLeader(canvas, tipRadius, radius);
            // The flank carried on past the tip, against the tool's axis direction.
            const P corner = { -tipRadius, 0.0 };
            const double arc = 60.0;
            canvas.Line(corner, Polar(corner, arc + 8.0, -90.0));
            canvas.Line(corner, Polar(corner, arc + 8.0, -90.0 + taperAngle));
            canvas.Angle(corner, arc, -90.0, -90.0 + taperAngle, 0, false);
            canvas.Letter({ corner.x - 13.0, -arc }, L'i');
            break;
        }
        case ToolType::ThreadMill:
        {
            // The pitch: crest to crest, or root to root across a single tooth.
            const double x = -radius - 14.0;
            const bool single = threadTeeth == 1;
            const double y0 = single ? 0.0 : 0.5 * threadPitch, y1 = y0 + threadPitch;
            const double from = single ? -threadRoot : -radius;
            canvas.Line({ from, y0 }, { x - 5.0, y0 });
            canvas.Line({ from, y1 }, { x - 5.0, y1 });
            canvas.Dimension({ x, y0 }, { x, y1 }, L'k');

            const P crest = { radius, ((std::min)(threadTeeth - 1, 3) + 0.5) * threadPitch };
            const double half = threadHalf * 180.0 / kPi;
            canvas.Line(crest, Polar(crest, 26.0, half));
            canvas.Line(crest, Polar(crest, 26.0, -half));
            canvas.Angle(crest, 20.0, -half, half, L'l', false);
            break;
        }
        default:
            break;
        }
    }

    SetStretchBltMode(hdc, HALFTONE);
    SetBrushOrgEx(hdc, 0, 0, nullptr);
    StretchBlt(hdc, rect.left, rect.top, width, height, memory, 0, 0, whole.right, whole.bottom, SRCCOPY);

    SelectObject(memory, oldBitmap);
    DeleteObject(bitmap);
    DeleteDC(memory);
}
