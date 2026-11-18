// ShapeDialog.cpp : the dialog that edits a shape typed in an operation.

#include "stdafx.h"
#include "CAMForAlibre.h"
#include "ShapeDialog.h"
#include "Help.h"
#include "ModelReader.h"

#include <cmath>
#include <cwchar>
#include <sstream>

using namespace camcore;

namespace
{
    bool ReadNumber(CWnd* pDialog, int id, double& value)
    {
        CString text;
        pDialog->GetDlgItemText(id, text);
        text.Trim();
        wchar_t* pEnd = nullptr;
        const double parsed = std::wcstod(text, &pEnd);
        if (text.IsEmpty() || pEnd == nullptr || *pEnd != L'\0' || !std::isfinite(parsed))
            return false;
        value = parsed;
        return true;
    }

    std::wstring Number(double value)
    {
        wchar_t text[64] = {};
        _snwprintf_s(text, _countof(text), _TRUNCATE, L"%.10g", value);
        return text;
    }

    // "x, y" a line; commas, semicolons, spaces or tabs between. False on the
    // first line that isn't two numbers.
    bool ReadPoints(const CString& text, std::vector<Vec3>& points, int& badLine)
    {
        points.clear();
        std::wistringstream lines(static_cast<const wchar_t*>(text));
        std::wstring line;
        int number = 0;
        while (std::getline(lines, line))
        {
            ++number;
            for (wchar_t& c : line)
            {
                if (c == L',' || c == L'\t' || c == L';' || c == L'\r')
                    c = L' ';
            }
            if (line.find_first_not_of(L' ') == std::wstring::npos)
                continue;
            std::wistringstream values(line);
            double x = 0.0, y = 0.0;
            std::wstring rest;
            if (!(values >> x >> y) || (values >> rest) || !std::isfinite(x) || !std::isfinite(y))
            {
                badLine = number;
                return false;
            }
            points.push_back({ x, y, 0.0 });
        }
        return true;
    }
}

std::wstring ShapePositionText(const TypedShape& shape)
{
    if (shape.kind == ShapeKind::Polygon)
        return std::to_wstring(shape.points.size()) + L" points";
    return L"X " + Number(shape.x) + L", Y " + Number(shape.y);
}

std::wstring ShapeSizeText(const TypedShape& shape)
{
    switch (shape.kind)
    {
    case ShapeKind::Circle:
        return L"Dia " + Number(shape.diameter);
    case ShapeKind::Rectangle:
        return Number(shape.width) + L" x " + Number(shape.height) +
               (shape.angle != 0.0 ? L" at " + Number(shape.angle) + L"\u00B0" : std::wstring());
    default:
        return L"";
    }
}

namespace
{
    // Each spinner and the box it steps, and how far a click goes.
    struct Spinner
    {
        int spin;
        int box;
    };
    const Spinner kSpinners[] = {
        { IDC_SHAPE_X_SPIN, IDC_SHAPE_X },         { IDC_SHAPE_Y_SPIN, IDC_SHAPE_Y },
        { IDC_SHAPE_Z_SPIN, IDC_SHAPE_Z },         { IDC_SHAPE_WIDTH_SPIN, IDC_SHAPE_WIDTH },
        { IDC_SHAPE_HEIGHT_SPIN, IDC_SHAPE_HEIGHT }, { IDC_SHAPE_ANGLE_SPIN, IDC_SHAPE_ANGLE },
        { IDC_SHAPE_DIAMETER_SPIN, IDC_SHAPE_DIAMETER },
    };
    const wchar_t kZNote[] = L"Z is used only when the operation has nothing else in its area; otherwise the shape is "
                             L"put at the picked geometry's height.";
}

BEGIN_MESSAGE_MAP(CShapeDlg, CDialog)
    ON_BN_CLICKED(IDC_HELP_BUTTON, &CShapeDlg::OnHelpButton)
    ON_CONTROL_RANGE(EN_CHANGE, IDC_SHAPE_X, IDC_SHAPE_POINTS, &CShapeDlg::OnValueChanged)
    ON_NOTIFY_RANGE(UDN_DELTAPOS, IDC_SHAPE_X_SPIN, IDC_SHAPE_DIAMETER_SPIN, &CShapeDlg::OnSpin)
END_MESSAGE_MAP()

CShapeDlg::CShapeDlg(const TypedShape& shape, CWnd* pParent, std::function<void(const TypedShape&)> preview)
    : CDialog(IDD_SHAPE, pParent)
    , m_shape(shape)
    , m_preview(std::move(preview))
{
}

BOOL CShapeDlg::OnInitDialog()
{
    CDialog::OnInitDialog();
    SetWindowText(FromUtf8(ShapeKindTitle(m_shape.kind)).c_str());

    const bool rectangle = m_shape.kind == ShapeKind::Rectangle;
    const bool circle = m_shape.kind == ShapeKind::Circle;
    const bool polygon = m_shape.kind == ShapeKind::Polygon;
    const auto show = [this](std::initializer_list<int> ids, bool visible) {
        for (int id : ids)
            GetDlgItem(id)->ShowWindow(visible ? SW_SHOW : SW_HIDE);
    };
    show({ IDC_SHAPE_X, IDC_SHAPE_X_LABEL, IDC_SHAPE_X_SPIN, IDC_SHAPE_Y, IDC_SHAPE_Y_LABEL, IDC_SHAPE_Y_SPIN },
         !polygon);
    show({ IDC_SHAPE_WIDTH, IDC_SHAPE_WIDTH_LABEL, IDC_SHAPE_WIDTH_SPIN, IDC_SHAPE_HEIGHT, IDC_SHAPE_HEIGHT_LABEL,
           IDC_SHAPE_HEIGHT_SPIN, IDC_SHAPE_ANGLE, IDC_SHAPE_ANGLE_LABEL, IDC_SHAPE_ANGLE_SPIN },
         rectangle);
    show({ IDC_SHAPE_DIAMETER, IDC_SHAPE_DIAMETER_LABEL, IDC_SHAPE_DIAMETER_SPIN }, circle);

    // The spinners stay in the middle of their range: each click steps the box's own number.
    for (const Spinner& spinner : kSpinners)
    {
        CSpinButtonCtrl* pSpin = static_cast<CSpinButtonCtrl*>(GetDlgItem(spinner.spin));
        pSpin->SetRange32(0, 1000);
        pSpin->SetPos32(500);
    }
    show({ IDC_SHAPE_POINTS, IDC_SHAPE_POINTS_LABEL }, polygon);

    SetDlgItemText(IDC_SHAPE_X, Number(m_shape.x).c_str());
    SetDlgItemText(IDC_SHAPE_Y, Number(m_shape.y).c_str());
    SetDlgItemText(IDC_SHAPE_Z, Number(m_shape.z).c_str());
    SetDlgItemText(IDC_SHAPE_WIDTH, Number(m_shape.width).c_str());
    SetDlgItemText(IDC_SHAPE_HEIGHT, Number(m_shape.height).c_str());
    SetDlgItemText(IDC_SHAPE_ANGLE, Number(m_shape.angle).c_str());
    SetDlgItemText(IDC_SHAPE_DIAMETER, Number(m_shape.diameter).c_str());
    std::wstring points;
    for (const Vec3& p : m_shape.points)
        points += Number(p.x) + L", " + Number(p.y) + L"\r\n";
    SetDlgItemText(IDC_SHAPE_POINTS, points.c_str());
    SetDlgItemText(IDC_SHAPE_NOTE, kZNote);
    m_ready = true;
    if (m_preview)
        m_preview(m_shape);
    return TRUE;
}

void CShapeDlg::ShowNote(const std::wstring& problem)
{
    SetDlgItemText(IDC_SHAPE_NOTE, problem.empty() ? kZNote : problem.c_str());
}

void CShapeDlg::OnValueChanged(UINT /*id*/)
{
    if (!m_ready)
        return;
    TypedShape shape;
    std::wstring problem;
    const bool usable = ReadShape(shape, problem);
    ShowNote(problem);
    if (usable && m_preview)
        m_preview(shape);
}

void CShapeDlg::OnSpin(UINT id, NMHDR* pNotify, LRESULT* pResult)
{
    // The spinner itself doesn't move; the box steps by 1 (mm or degree), or 0.1 with Ctrl.
    *pResult = 1;
    const NMUPDOWN* pUpDown = reinterpret_cast<const NMUPDOWN*>(pNotify);
    for (const Spinner& spinner : kSpinners)
    {
        if (static_cast<UINT>(spinner.spin) != id)
            continue;
        double value = 0.0;
        if (!ReadNumber(this, spinner.box, value))
            value = 0.0;
        const double step = (GetKeyState(VK_CONTROL) < 0) ? 0.1 : 1.0;
        value += (pUpDown->iDelta > 0 ? step : -step);
        value = std::round(value / step) * step;       // stay on the step's grid
        if (std::fabs(value) < 1e-9)
            value = 0.0;
        SetDlgItemText(spinner.box, Number(value).c_str());     // EN_CHANGE updates the view
        return;
    }
}

bool CShapeDlg::ReadShape(TypedShape& shape, std::wstring& problem)
{
    shape = m_shape;
    problem.clear();
    const auto read = [&](int id, double& value, const wchar_t* name) {
        if (problem.empty() && !ReadNumber(this, id, value))
            problem = std::wstring(name) + L" must be a number.";
    };
    if (shape.kind != ShapeKind::Polygon)
    {
        read(IDC_SHAPE_X, shape.x, L"Centre X");
        read(IDC_SHAPE_Y, shape.y, L"Centre Y");
    }
    if (shape.kind == ShapeKind::Rectangle)
    {
        read(IDC_SHAPE_WIDTH, shape.width, L"Width");
        read(IDC_SHAPE_HEIGHT, shape.height, L"Height");
        read(IDC_SHAPE_ANGLE, shape.angle, L"Angle");
    }
    if (shape.kind == ShapeKind::Circle)
        read(IDC_SHAPE_DIAMETER, shape.diameter, L"Diameter");
    read(IDC_SHAPE_Z, shape.z, L"Z");
    if (problem.empty() && shape.kind == ShapeKind::Polygon)
    {
        CString text;
        GetDlgItemText(IDC_SHAPE_POINTS, text);
        int badLine = 0;
        if (!ReadPoints(text, shape.points, badLine))
            problem = L"Line " + std::to_wstring(badLine) + L" of the points isn't an X and a Y.";
    }
    if (problem.empty())
        problem = FromUtf8(ValidateShape(shape));
    return problem.empty();
}

void CShapeDlg::OnOK()
{
    TypedShape shape;
    std::wstring problem;
    if (!ReadShape(shape, problem))
    {
        ShowNote(problem);
        MessageBeep(MB_ICONWARNING);
        return;
    }
    m_shape = shape;
    CDialog::OnOK();
}

void CShapeDlg::OnHelpButton()
{
    ShowHelp(L"shape");
}

BEGIN_MESSAGE_MAP(CChooseSketchDlg, CDialog)
    ON_BN_CLICKED(IDC_HELP_BUTTON, &CChooseSketchDlg::OnHelpButton)
END_MESSAGE_MAP()

CChooseSketchDlg::CChooseSketchDlg(const std::vector<PickedGeometry>& sketches, CWnd* pParent)
    : CDialog(IDD_CHOOSE_SKETCH, pParent)
    , m_sketches(sketches)
{
}

BOOL CChooseSketchDlg::OnInitDialog()
{
    CDialog::OnInitDialog();
    CListBox* pList = static_cast<CListBox*>(GetDlgItem(IDC_CHOOSE_SKETCH_LIST));
    for (const PickedGeometry& sketch : m_sketches)
    {
        const std::wstring owner = sketch.occurrenceName.empty() ? L"Assembly" : FromUtf8(sketch.occurrenceName);
        pList->AddString((owner + L": " + FromUtf8(sketch.targetName)).c_str());
    }
    if (m_sketches.empty())
        SetDlgItemText(IDC_CHOOSE_SKETCH_NOTE, L"There are no sketches with lines, arcs or circles in the assembly "
                                               L"or its visible parts.");
    return TRUE;
}

void CChooseSketchDlg::OnOK()
{
    CListBox* pList = static_cast<CListBox*>(GetDlgItem(IDC_CHOOSE_SKETCH_LIST));
    m_chosen.clear();
    for (int i = 0; i < static_cast<int>(m_sketches.size()); ++i)
    {
        if (pList->GetSel(i) > 0)
            m_chosen.push_back(m_sketches[i]);
    }
    CDialog::OnOK();
}

void CChooseSketchDlg::OnHelpButton()
{
    ShowHelp(L"choose-sketches");
}
