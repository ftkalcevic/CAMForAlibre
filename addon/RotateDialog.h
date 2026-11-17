// RotateDialog.h : the dialog that edits a Rotate step, and the Indexing group it
// shares with the Set Origin dialog.
//
// A Rotate step turns the work about the machine's rotary axis; the steps below
// it run in the turned frame. See core\Rotate.h and core\Rotary.h.

#pragma once

#include <string>

#include "resource.h"
#include "CamJob.h"

// The Indexing group (IDC_INDEX_*): retract height and which way to turn.
void LoadIndexControls(CWnd* pDialog, const camcore::IndexSettings& index);
bool ReadIndexControls(CWnd* pDialog, camcore::IndexSettings& index, std::wstring& problem);
void UpdateIndexControls(CWnd* pDialog);

class CRotateDlg : public CDialog
{
public:
    // machineText says which way the machine's rotary axis runs, or why there's none.
    CRotateDlg(const camcore::SequenceStep& step, const std::wstring& machineText, CWnd* pParent);

    const camcore::SequenceStep& GetStep() const { return m_step; }

    enum { IDD = IDD_ROTATE };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;

    afx_msg void OnAutomaticRetract();
    afx_msg void OnHelpButton();

    DECLARE_MESSAGE_MAP()

private:
    camcore::SequenceStep m_step;
    std::wstring m_machineText;
};
