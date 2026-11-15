// FixturesDialog.cpp : the Fixtures step's dialog: the parts that hold the stock, and how close tools may come.

#include "stdafx.h"
#include "CAMForAlibre.h"
#include "FixturesDialog.h"
#include "Help.h"
#include "CamTabCommand.h"
#include "ModelReader.h"
#include "Log.h"

#include <algorithm>
#include <cmath>
#include <cwchar>

using camcore::FixturePart;

namespace
{
    // Where the dialog was last, so it opens there again.
    CPoint g_lastFixturesPosition(-1, -1);

    bool ReadNumber(CWnd* pDialog, int id, double& value)
    {
        CString text;
        pDialog->GetDlgItemText(id, text);
        text.Trim();
        if (text.IsEmpty())
            return false;
        wchar_t* pEnd = nullptr;
        const double number = std::wcstod(text, &pEnd);
        if (pEnd == nullptr || *pEnd != L'\0' || !std::isfinite(number))
            return false;
        value = number;
        return true;
    }

    std::wstring NumberText(double value)
    {
        wchar_t text[64];
        swprintf_s(text, L"%.4f", value);
        std::wstring result = text;
        while (!result.empty() && result.back() == L'0')
            result.pop_back();
        if (!result.empty() && result.back() == L'.')
            result.pop_back();
        return result;
    }

    std::wstring Join(const std::vector<std::string>& problems)
    {
        std::wstring text;
        for (const std::string& problem : problems)
            text += (text.empty() ? L"" : L"\n") + FromUtf8(problem);
        return text;
    }
}

BEGIN_MESSAGE_MAP(CFixturesDlg, CDialog)
    ON_BN_CLICKED(IDC_HELP_BUTTON, &CFixturesDlg::OnHelpButton)
    ON_BN_CLICKED(IDC_FIX_SELECT_PARTS, &CFixturesDlg::OnSelectParts)
    ON_BN_CLICKED(IDC_FIX_REMOVE, &CFixturesDlg::OnRemove)
    ON_BN_CLICKED(IDC_FIX_SET_CLEARANCE, &CFixturesDlg::OnSetClearance)
    ON_NOTIFY(LVN_ITEMCHANGED, IDC_FIX_PARTS, &CFixturesDlg::OnPartsChanged)
    ON_WM_CLOSE()
END_MESSAGE_MAP()

CFixturesDlg::CFixturesDlg(CCamTabCommand* pTab, const camcore::SequenceStep& step)
    : CDialog(IDD_FIXTURES, nullptr)
    , m_pTab(pTab)
    , m_step(step)
    , m_selecting(false)
{
}

bool CFixturesDlg::CreateModeless(CWnd* pParent)
{
    return Create(IDD_FIXTURES, pParent) != FALSE;
}

void CFixturesDlg::OnHelpButton()
{
    ShowHelp(L"fixtures");
}

BOOL CFixturesDlg::OnInitDialog()
{
    CDialog::OnInitDialog();

    m_parts.SubclassDlgItem(IDC_FIX_PARTS, this);
    m_parts.SetExtendedStyle(m_parts.GetExtendedStyle() | LVS_EX_FULLROWSELECT);
    CRect rect;
    m_parts.GetClientRect(&rect);
    m_parts.InsertColumn(0, L"Part", LVCFMT_LEFT, rect.Width() * 2 / 3);
    m_parts.InsertColumn(1, L"Clearance (mm)", LVCFMT_RIGHT, rect.Width() - rect.Width() * 2 / 3);

    SetDlgItemText(IDC_FIX_NAME, FromUtf8(m_step.name).c_str());
    SetDlgItemText(IDC_FIX_DEFAULT_CLEARANCE, NumberText(m_step.fixtures.defaultClearance).c_str());
    SetDlgItemText(IDC_FIX_CLEARANCE, NumberText(m_step.fixtures.defaultClearance).c_str());
    CheckDlgButton(IDC_FIX_TRIM, m_step.fixtures.trim ? BST_CHECKED : BST_UNCHECKED);
    SetDlgItemText(IDC_FIX_PROMPT, L"Click Select Parts to choose the vices, clamps and screws in the view.");
    ShowParts();

    if (g_lastFixturesPosition.x >= 0)
        SetWindowPos(nullptr, g_lastFixturesPosition.x, g_lastFixturesPosition.y, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
    return TRUE;
}

void CFixturesDlg::PostNcDestroy()
{
    delete this;
}

void CFixturesDlg::ShowParts()
{
    m_parts.DeleteAllItems();
    for (size_t i = 0; i < m_step.fixtures.parts.size(); ++i)
    {
        const FixturePart& part = m_step.fixtures.parts[i];
        const int row = m_parts.InsertItem(static_cast<int>(i), FromUtf8(part.name).c_str());
        m_parts.SetItemText(row, 1, NumberText(part.clearance).c_str());
    }
    LRESULT ignored = 0;
    OnPartsChanged(nullptr, &ignored);
}

std::vector<int> CFixturesDlg::ChosenRows() const
{
    std::vector<int> rows;
    POSITION position = m_parts.GetFirstSelectedItemPosition();
    while (position != nullptr)
        rows.push_back(m_parts.GetNextSelectedItem(position));
    return rows;
}

void CFixturesDlg::OnPartsChanged(NMHDR*, LRESULT* pResult)
{
    const bool any = !ChosenRows().empty();
    GetDlgItem(IDC_FIX_REMOVE)->EnableWindow(any);
    GetDlgItem(IDC_FIX_SET_CLEARANCE)->EnableWindow(any);
    *pResult = 0;
}

void CFixturesDlg::OnSelectParts()
{
    if (m_selecting)
    {
        StopSelectingParts();
        if (m_pTab != nullptr)
            m_pTab->EndPartSelect();
        return;
    }

    m_selecting = true;
    SetDlgItemText(IDC_FIX_SELECT_PARTS, L"Done");
    SetDlgItemText(IDC_FIX_PROMPT, L"Click parts in the view. A listed part is taken off the list. "
                                   L"Click Done when finished.");
    if (m_pTab != nullptr)
        m_pTab->BeginPartSelect();
}

void CFixturesDlg::StopSelectingParts()
{
    m_selecting = false;
    SetDlgItemText(IDC_FIX_SELECT_PARTS, L"Select Parts");
    SetDlgItemText(IDC_FIX_PROMPT, L"Click Select Parts to choose the vices, clamps and screws in the view.");
}

void CFixturesDlg::ToggleParts(const std::vector<camcore::StockPart>& parts)
{
    // New parts take the clearance for new parts as it's typed now.
    double clearance = m_step.fixtures.defaultClearance;
    double typed = 0.0;
    if (ReadNumber(this, IDC_FIX_DEFAULT_CLEARANCE, typed) && typed >= 0.0)
        clearance = typed;

    std::vector<FixturePart>& list = m_step.fixtures.parts;
    for (const camcore::StockPart& part : parts)
    {
        const auto listed = std::find_if(list.begin(), list.end(), [&](const FixturePart& other)
                                         { return other.occurrenceKey.hex == part.occurrenceKey.hex; });
        if (listed != list.end())
        {
            list.erase(listed);
            continue;
        }
        FixturePart fixture;
        fixture.occurrenceKey = part.occurrenceKey;
        fixture.name = part.name;
        fixture.clearance = clearance;
        list.push_back(fixture);
    }
    ShowParts();
    if (m_pTab != nullptr)
        m_pTab->OnFixturesPreviewChanged();
}

void CFixturesDlg::OnRemove()
{
    std::vector<int> rows = ChosenRows();
    std::sort(rows.rbegin(), rows.rend());
    for (int row : rows)
    {
        if (row >= 0 && static_cast<size_t>(row) < m_step.fixtures.parts.size())
            m_step.fixtures.parts.erase(m_step.fixtures.parts.begin() + row);
    }
    ShowParts();
    if (m_pTab != nullptr)
        m_pTab->OnFixturesPreviewChanged();
}

void CFixturesDlg::OnSetClearance()
{
    double clearance = 0.0;
    if (!ReadNumber(this, IDC_FIX_CLEARANCE, clearance) || clearance < 0.0)
    {
        ShowResult(L"The clearance needs a number, 0 or more.");
        return;
    }
    const std::vector<int> rows = ChosenRows();
    for (int row : rows)
    {
        if (row >= 0 && static_cast<size_t>(row) < m_step.fixtures.parts.size())
            m_step.fixtures.parts[row].clearance = clearance;
    }
    ShowParts();
    for (int row : rows)
        m_parts.SetItemState(row, LVIS_SELECTED, LVIS_SELECTED);
    ShowResult(L"");
}

void CFixturesDlg::ShowResult(const std::wstring& text)
{
    SetDlgItemText(IDC_FIX_RESULT, text.c_str());
}

bool CFixturesDlg::ReadFields(std::wstring& problem)
{
    camcore::SequenceStep step = m_step;
    CString name;
    GetDlgItemText(IDC_FIX_NAME, name);
    name.Trim();
    if (name.IsEmpty())
    {
        problem = L"The step needs a name.";
        return false;
    }
    step.name = ToUtf8(static_cast<LPCWSTR>(name));
    if (!ReadNumber(this, IDC_FIX_DEFAULT_CLEARANCE, step.fixtures.defaultClearance))
    {
        problem = L"The clearance for new parts needs a number.";
        return false;
    }
    step.fixtures.trim = IsDlgButtonChecked(IDC_FIX_TRIM) == BST_CHECKED;
    const std::vector<std::string> problems = camcore::ValidateFixtures(step.fixtures);
    if (!problems.empty())
    {
        problem = Join(problems);
        return false;
    }
    m_step = step;
    return true;
}

void CFixturesDlg::OnOK()
{
    std::wstring problem;
    if (!ReadFields(problem))
    {
        AfxMessageBox(problem.c_str(), MB_OK | MB_ICONWARNING);
        return;
    }
    Finish(true);
}

void CFixturesDlg::OnCancel()
{
    Finish(false);
}

void CFixturesDlg::OnClose()
{
    Finish(false);
}

void CFixturesDlg::Finish(bool apply)
{
    CRect rect;
    GetWindowRect(&rect);
    g_lastFixturesPosition = rect.TopLeft();

    if (m_pTab != nullptr)
    {
        CCamTabCommand* pTab = m_pTab;
        m_pTab = nullptr;
        pTab->OnFixturesDialogDone(this, apply);
    }
    DestroyWindow();
}
