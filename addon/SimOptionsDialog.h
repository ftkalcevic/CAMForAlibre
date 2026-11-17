// SimOptionsDialog.h : the machine's Sim Options, from the Machine tab: each
// axis's maximum velocity and acceleration, and the corner blend tolerance,
// which time the simulation's moves. See core\MotionTiming.h.

#pragma once

#include "resource.h"
#include "Machine.h"

class CSimOptionsDlg : public CDialog
{
public:
    // axes: the machine's (3 or 4); the rotary row is greyed on 3.
    CSimOptionsDlg(const camcore::MotionLimits& motion, int axes, CWnd* pParent);

    const camcore::MotionLimits& GetMotion() const { return m_motion; }

    enum { IDD = IDD_SIM_OPTIONS };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;

    afx_msg void OnHelpButton();

    DECLARE_MESSAGE_MAP()

private:
    camcore::MotionLimits m_motion;
    int m_axes;
};
