// RotateDialog.cpp : the dialog that edits a Rotate step, and the Indexing group.

#include "stdafx.h"
#include "CAMForAlibre.h"
#include "RotateDialog.h"
#include "Help.h"
#include "ModelReader.h"

#include <cmath>
#include <cwchar>

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

    void SetNumber(CWnd* pDialog, int id, double value)
    {
        wchar_t text[64] = {};
        _snwprintf_s(text, _countof(text), _TRUNCATE, L"%.10g", value);
        pDialog->SetDlgItemText(id, text);
    }

    std::wstring JoinProblems(const std::vector<std::string>& problems)
    {
        std::wstring text;
        for (const std::string& problem : problems)
            text += (text.empty() ? L"" : L"\n") + FromUtf8(problem);
        return text;
    }
}

void LoadIndexControls(CWnd* pDialog, const IndexSettings& index)
{
    pDialog->CheckDlgButton(IDC_INDEX_AUTO_RETRACT, index.automaticRetract ? BST_CHECKED : BST_UNCHECKED);
    SetNumber(pDialog, IDC_INDEX_RETRACT, index.retractHeight);
    CComboBox* pDirection = static_cast<CComboBox*>(pDialog->GetDlgItem(IDC_INDEX_DIRECTION));
    pDirection->ResetContent();
    pDirection->AddString(L"Shortest");
    pDirection->AddString(L"Positive");
    pDirection->AddString(L"Negative");
    pDirection->SetCurSel(index.direction == IndexDirection::Positive   ? 1
                          : index.direction == IndexDirection::Negative ? 2
                                                                        : 0);
    UpdateIndexControls(pDialog);
}

bool ReadIndexControls(CWnd* pDialog, IndexSettings& index, std::wstring& problem)
{
    IndexSettings read = index;
    read.automaticRetract = pDialog->IsDlgButtonChecked(IDC_INDEX_AUTO_RETRACT) == BST_CHECKED;
    if (!read.automaticRetract && !ReadNumber(pDialog, IDC_INDEX_RETRACT, read.retractHeight))
    {
        problem = L"The retract height must be a number.";
        return false;
    }
    const int selection = static_cast<CComboBox*>(pDialog->GetDlgItem(IDC_INDEX_DIRECTION))->GetCurSel();
    read.direction = (selection == 1) ? IndexDirection::Positive
                     : (selection == 2) ? IndexDirection::Negative
                                        : IndexDirection::Shortest;
    problem = JoinProblems(ValidateIndex(read));
    if (!problem.empty())
        return false;
    index = read;
    return true;
}

void UpdateIndexControls(CWnd* pDialog)
{
    const bool manual = pDialog->IsDlgButtonChecked(IDC_INDEX_AUTO_RETRACT) != BST_CHECKED;
    pDialog->GetDlgItem(IDC_INDEX_RETRACT)->EnableWindow(manual);
    pDialog->GetDlgItem(IDC_INDEX_RETRACT_LABEL)->EnableWindow(manual);
}

/////////////////////////////////////////////////////////////////////////////
// CRotateDlg

BEGIN_MESSAGE_MAP(CRotateDlg, CDialog)
    ON_BN_CLICKED(IDC_HELP_BUTTON, &CRotateDlg::OnHelpButton)
    ON_BN_CLICKED(IDC_INDEX_AUTO_RETRACT, &CRotateDlg::OnAutomaticRetract)
END_MESSAGE_MAP()

CRotateDlg::CRotateDlg(const SequenceStep& step, const std::wstring& machineText, CWnd* pParent)
    : CDialog(IDD_ROTATE, pParent)
    , m_step(step)
    , m_machineText(machineText)
{
}

void CRotateDlg::OnHelpButton()
{
    ShowHelp(L"rotate");
}

void CRotateDlg::OnAutomaticRetract()
{
    UpdateIndexControls(this);
}

BOOL CRotateDlg::OnInitDialog()
{
    CDialog::OnInitDialog();
    SetDlgItemText(IDC_ROTATE_NAME, FromUtf8(m_step.name).c_str());
    SetNumber(this, IDC_ROTATE_ANGLE, m_step.rotate.angle);
    CheckRadioButton(IDC_ROTATE_KEEP, IDC_ROTATE_TOP,
                     m_step.rotate.originAfter == OriginAfterRotate::TopOfStock ? IDC_ROTATE_TOP : IDC_ROTATE_KEEP);
    LoadIndexControls(this, m_step.index);
    SetDlgItemText(IDC_ROTATE_RESULT, m_machineText.c_str());
    return TRUE;
}

void CRotateDlg::OnOK()
{
    CString name;
    GetDlgItemText(IDC_ROTATE_NAME, name);
    name.Trim();

    SequenceStep edited = m_step;
    std::wstring problem;
    if (name.IsEmpty())
        problem = L"The step needs a name.";
    else if (!ReadNumber(this, IDC_ROTATE_ANGLE, edited.rotate.angle))
        problem = L"The angle must be a number.";
    else
    {
        edited.rotate.originAfter = (IsDlgButtonChecked(IDC_ROTATE_TOP) == BST_CHECKED) ? OriginAfterRotate::TopOfStock
                                                                                        : OriginAfterRotate::Keep;
        problem = JoinProblems(ValidateRotate(edited.rotate));
        if (problem.empty())
            ReadIndexControls(this, edited.index, problem);
    }
    if (!problem.empty())
    {
        MessageBoxW(problem.c_str(), L"Rotate", MB_OK | MB_ICONWARNING);
        return;
    }

    edited.name = ToUtf8(static_cast<LPCWSTR>(name));
    m_step = edited;
    CDialog::OnOK();
}
