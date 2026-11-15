// FeedsCalculatorDialog.cpp : the feeds and speeds calculator, which fills a Feeds & Speeds tab.

#include "stdafx.h"
#include "CAMForAlibre.h"
#include "FeedsCalculatorDialog.h"
#include "Help.h"
#include "ModelReader.h"

#include <cmath>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace camcore;

namespace
{
    // The depth and width of cut, as fractions of the diameter, remembered until Alibre closes.
    double g_depthFraction = 1.0;
    double g_widthFraction = 0.5;
    int g_aggression = 100;

    std::wstring Format(const wchar_t* format, double value)
    {
        wchar_t text[64] = {};
        _snwprintf_s(text, _countof(text), _TRUNCATE, format, value);
        return text;
    }
}

BEGIN_MESSAGE_MAP(CFeedsCalcDlg, CDialog)
    ON_BN_CLICKED(IDC_HELP_BUTTON, &CFeedsCalcDlg::OnHelpButton)
    ON_CBN_SELCHANGE(IDC_CALC_MATERIAL, &CFeedsCalcDlg::OnInputChanged)
    ON_EN_CHANGE(IDC_CALC_CHIP_LOAD, &CFeedsCalcDlg::OnInputChanged)
    ON_EN_CHANGE(IDC_CALC_DEPTH, &CFeedsCalcDlg::OnInputChanged)
    ON_EN_CHANGE(IDC_CALC_WIDTH, &CFeedsCalcDlg::OnInputChanged)
    ON_WM_HSCROLL()
    ON_WM_CTLCOLOR()
END_MESSAGE_MAP()

CFeedsCalcDlg::CFeedsCalcDlg(const FeedsCalcSource& source, const FeedSettings& start, CWnd* pParent)
    : CDialog(IDD_FEEDS_CALC, pParent)
    , m_source(source)
    , m_tool(ToolInMillimetres(source.tool))
    , m_start(start)
    , m_feeds(start)
{
}

void CFeedsCalcDlg::OnHelpButton()
{
    ShowHelp(L"feeds-calculator");
}

BOOL CFeedsCalcDlg::OnInitDialog()
{
    CDialog::OnInitDialog();

    const bool drill = m_tool.type == ToolType::Drill;
    std::wstring tool = L"T" + std::to_wstring(m_source.tool.number) + L" " + FromUtf8(m_source.tool.name) + L": " +
                        FromUtf8(ToolTypeTitle(m_tool.type)) + L", " + Format(L"%.4g", m_tool.diameter) + L" mm";
    if (!drill)
        tool += L", " + std::to_wstring(m_tool.fluteCount) + L" flutes";
    tool += L", " + FromUtf8(ToolMaterialTitle(m_tool.toolMaterial)) + L"\nStickout (overall length) " +
            Format(L"%.4g", m_tool.overallLength) + L" mm";
    SetDlgItemText(IDC_CALC_TOOL, tool.c_str());

    m_material.SubclassDlgItem(IDC_CALC_MATERIAL, this);
    int chosen = -1;
    for (const Material& material : m_source.materials.materials)
    {
        if (m_source.materialFixed && material.id != m_source.materialId)
            continue;
        const int index = m_material.AddString(FromUtf8(material.name).c_str());
        m_materialIds.insert(m_materialIds.begin() + index, material.id);
        if (material.id == m_source.materialId)
            chosen = index;
    }
    if (chosen < 0 && !m_source.materialFixed && m_material.GetCount() > 0)
        chosen = 0;
    m_material.SetCurSel(chosen);
    m_material.EnableWindow(m_source.materialFixed ? FALSE : TRUE);

    SetDlgItemText(IDC_CALC_CHIP_UNIT, drill ? L"mm/rev" : L"mm/tooth");
    if (m_tool.chipLoad > 0.0)
        SetDlgItemText(IDC_CALC_CHIP_LOAD, Format(L"%.4g", m_tool.chipLoad).c_str());
    SetDlgItemText(IDC_CALC_DEPTH, Format(L"%.3g", g_depthFraction * m_tool.diameter).c_str());
    SetDlgItemText(IDC_CALC_WIDTH, Format(L"%.3g", g_widthFraction * m_tool.diameter).c_str());
    GetDlgItem(IDC_CALC_WIDTH)->EnableWindow(drill ? FALSE : TRUE);

    m_aggression.SubclassDlgItem(IDC_CALC_AGGRESSION, this);
    m_aggression.SetRange(50, 150);
    m_aggression.SetLineSize(5);
    m_aggression.SetPageSize(10);
    m_aggression.SetPos(g_aggression);

    m_ready = true;
    Recalculate();
    return TRUE;
}

bool CFeedsCalcDlg::ReadBox(int id, double& value) const
{
    CString text;
    GetDlgItemText(id, text);
    text.Trim();
    if (text.IsEmpty())
        return false;
    wchar_t* pEnd = nullptr;
    const double parsed = wcstod(text, &pEnd);
    if (pEnd == nullptr || *pEnd != L'\0')
        return false;
    value = parsed;
    return true;
}

void CFeedsCalcDlg::Recalculate()
{
    if (!m_ready)
        return;

    FeedsCalcInput input;
    input.tool = m_tool;
    input.start = m_start;
    input.maxSpindleRpm = m_source.maxSpindleRpm;
    input.aggression = m_aggression.GetPos() / 100.0;
    SetDlgItemText(IDC_CALC_AGGRESSION_TEXT, (std::to_wstring(m_aggression.GetPos()) + L"%").c_str());

    std::wstring problem;
    const int index = m_material.GetCurSel();
    const Material* pMaterial = nullptr;
    if (index >= 0 && static_cast<size_t>(index) < m_materialIds.size())
        pMaterial = FindMaterial(m_source.materials, m_materialIds[static_cast<size_t>(index)]);
    if (pMaterial == nullptr)
    {
        problem = m_source.materials.materials.empty()
                      ? L"There are no materials yet. Add them on the Machine dialog's Materials tab."
                      : L"Choose a material.";
    }
    else
    {
        input.cutting = pMaterial->CuttingFor(m_tool.toolMaterial);
    }

    const bool drill = m_tool.type == ToolType::Drill;
    double value = 0.0;
    input.chipLoad = ReadBox(IDC_CALC_CHIP_LOAD, value) ? value : 0.0;
    input.depth = ReadBox(IDC_CALC_DEPTH, value) ? value : 0.0;
    input.width = (!drill && ReadBox(IDC_CALC_WIDTH, value)) ? value : 0.0;

    FeedsCalcResult result = CalculateFeeds(input);
    if (problem.empty() && !result.problems.empty())
    {
        problem = FromUtf8(result.problems.front());
        if (!input.cutting.set && pMaterial != nullptr)
        {
            problem = FromUtf8(pMaterial->name) + L" has no surface speed for " +
                      FromUtf8(ToolMaterialTitle(m_tool.toolMaterial)) +
                      L" tools. Set it on the Machine dialog's Materials tab (Edit).";
        }
    }

    m_ok = problem.empty();
    SetDlgItemText(IDC_CALC_PROBLEM, problem.c_str());
    GetDlgItem(IDOK)->EnableWindow(m_ok ? TRUE : FALSE);
    if (!m_ok)
    {
        SetDlgItemText(IDC_CALC_RESULTS, L"");
        return;
    }

    // Remembered for next time, as parts of the diameter.
    if (m_tool.diameter > 0.0)
    {
        g_depthFraction = input.depth / m_tool.diameter;
        if (!drill)
            g_widthFraction = input.width / m_tool.diameter;
    }
    g_aggression = m_aggression.GetPos();

    const wchar_t* unit = drill ? L" mm/rev" : L" mm/tooth";
    std::wstring text = L"Diameter at that depth: " + Format(L"%.3g", result.effectiveDiameter) + L" mm\n";
    text += L"Spindle speed: " + Format(L"%.0f", std::round(result.rpm)) + L" RPM";
    if (result.rpmLimited)
        text += L" (limited by the machine; " + Format(L"%.0f", std::round(result.rpmWanted)) + L" wanted)";
    text += L"\nChip load: " + Format(L"%.4g", result.toolChipLoad) + L" x " +
            Format(L"%.0f", result.materialFactor * 100.0) + L"% material x " +
            Format(L"%.0f", result.aggression * 100.0) + L"% aggression";
    if (result.thinningFactor != 1.0)
        text += L" x " + Format(L"%.2f", result.thinningFactor) + L" thinning";
    if (result.stickoutFactor != 1.0)
        text += L" x " + Format(L"%.2f", result.stickoutFactor) + L" stickout";
    text += L" = " + Format(L"%.4g", result.chipLoad) + unit + L"\n";
    text += L"Cut " + Format(L"%.0f", result.feeds.cut) + L", plunge " + Format(L"%.0f", result.feeds.plunge) +
            L", engage " + Format(L"%.0f", result.feeds.engage) + L" mm/min\n";
    text += L"Removal rate: " + Format(L"%.3g", result.removalRate / 1000.0) + L" cm\u00B3/min";
    SetDlgItemText(IDC_CALC_RESULTS, text.c_str());
    m_feeds = result.feeds;
}

void CFeedsCalcDlg::OnInputChanged()
{
    Recalculate();
}

void CFeedsCalcDlg::OnHScroll(UINT code, UINT position, CScrollBar* pScrollBar)
{
    CDialog::OnHScroll(code, position, pScrollBar);
    if (pScrollBar != nullptr && pScrollBar->GetSafeHwnd() == m_aggression.GetSafeHwnd())
        Recalculate();
}

HBRUSH CFeedsCalcDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT ctlColor)
{
    HBRUSH brush = CDialog::OnCtlColor(pDC, pWnd, ctlColor);
    if (pWnd != nullptr && pWnd->GetDlgCtrlID() == IDC_CALC_PROBLEM)
        pDC->SetTextColor(RGB(200, 0, 0));
    return brush;
}

void CFeedsCalcDlg::OnOK()
{
    Recalculate();
    if (!m_ok)
        return;
    CDialog::OnOK();
}
