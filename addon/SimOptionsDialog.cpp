// SimOptionsDialog.cpp : the machine's Sim Options.

#include "stdafx.h"
#include "CAMForAlibre.h"
#include "SimOptionsDialog.h"
#include "Help.h"
#include "ModelReader.h"

#include <cmath>
#include <cwchar>
#include <string>

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

    // Each number box, what it's called in a message, and where it goes.
    struct Field
    {
        int id;
        const wchar_t* label;
        double MotionLimits::*plain;
        AxisLimit MotionLimits::*axis;
        double AxisLimit::*part;

        double& In(MotionLimits& motion) const
        {
            return plain != nullptr ? motion.*plain : (motion.*axis).*part;
        }
    };
    const Field kFields[] = {
        { IDC_SIMOPT_X_VEL, L"X's maximum velocity", nullptr, &MotionLimits::x, &AxisLimit::velocity },
        { IDC_SIMOPT_X_ACC, L"X's maximum acceleration", nullptr, &MotionLimits::x, &AxisLimit::acceleration },
        { IDC_SIMOPT_Y_VEL, L"Y's maximum velocity", nullptr, &MotionLimits::y, &AxisLimit::velocity },
        { IDC_SIMOPT_Y_ACC, L"Y's maximum acceleration", nullptr, &MotionLimits::y, &AxisLimit::acceleration },
        { IDC_SIMOPT_Z_VEL, L"Z's maximum velocity", nullptr, &MotionLimits::z, &AxisLimit::velocity },
        { IDC_SIMOPT_Z_ACC, L"Z's maximum acceleration", nullptr, &MotionLimits::z, &AxisLimit::acceleration },
        { IDC_SIMOPT_ROTARY_VEL, L"The rotary axis's maximum velocity", &MotionLimits::rotaryVelocity, nullptr,
          nullptr },
        { IDC_SIMOPT_ROTARY_ACC, L"The rotary axis's maximum acceleration", &MotionLimits::rotaryAcceleration,
          nullptr, nullptr },
        { IDC_SIMOPT_BLEND, L"The blend tolerance", &MotionLimits::blendTolerance, nullptr, nullptr },
    };
}

BEGIN_MESSAGE_MAP(CSimOptionsDlg, CDialog)
    ON_BN_CLICKED(IDC_HELP_BUTTON, &CSimOptionsDlg::OnHelpButton)
END_MESSAGE_MAP()

CSimOptionsDlg::CSimOptionsDlg(const MotionLimits& motion, int axes, CWnd* pParent)
    : CDialog(IDD, pParent)
    , m_motion(motion)
    , m_axes(axes)
{
}

BOOL CSimOptionsDlg::OnInitDialog()
{
    CDialog::OnInitDialog();
    for (const Field& field : kFields)
        SetNumber(this, field.id, field.In(m_motion));
    const BOOL rotary = (m_axes == 4) ? TRUE : FALSE;
    for (int id : { IDC_SIMOPT_ROTARY_LABEL, IDC_SIMOPT_ROTARY_VEL, IDC_SIMOPT_ROTARY_ACC })
        GetDlgItem(id)->EnableWindow(rotary);
    return TRUE;
}

void CSimOptionsDlg::OnOK()
{
    MotionLimits motion = m_motion;
    for (const Field& field : kFields)
    {
        double value = 0.0;
        if (!ReadNumber(this, field.id, value))
        {
            MessageBox((std::wstring(field.label) + L" must be a number.").c_str(), L"CAMForAlibre",
                       MB_OK | MB_ICONWARNING);
            GotoDlgCtrl(GetDlgItem(field.id));
            return;
        }
        field.In(motion) = value;
    }
    const std::vector<std::string> problems = ValidateMotion(motion);
    if (!problems.empty())
    {
        std::wstring text;
        for (const std::string& problem : problems)
            text += (text.empty() ? L"" : L"\n") + FromUtf8(problem);
        MessageBox(text.c_str(), L"CAMForAlibre", MB_OK | MB_ICONWARNING);
        return;
    }
    m_motion = motion;
    CDialog::OnOK();
}

void CSimOptionsDlg::OnHelpButton()
{
    ShowHelp(L"sim-options");
}
