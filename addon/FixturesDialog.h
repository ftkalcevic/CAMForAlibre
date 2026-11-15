// FixturesDialog.h : the Fixtures step's dialog: the parts that hold the stock, and how close tools may come.

#pragma once

#include <string>
#include <vector>

#include "resource.h"
#include "CamJob.h"

class CCamTabCommand;

// Modeless, so parts can be clicked in the view.
class CFixturesDlg : public CDialog
{
public:
    CFixturesDlg(CCamTabCommand* pTab, const camcore::SequenceStep& step);

    bool CreateModeless(CWnd* pParent);
    void DetachFromTab() { m_pTab = nullptr; }

    // The step as edited so far.
    const camcore::SequenceStep& GetStep() const { return m_step; }

    // Select Parts: each part clicked in the view is added, or removed if it's listed.
    bool IsSelectingParts() const { return m_selecting; }
    void StopSelectingParts();
    void ToggleParts(const std::vector<camcore::StockPart>& parts);
    void ShowResult(const std::wstring& text);

    enum { IDD = IDD_FIXTURES };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    void OnCancel() override;
    void PostNcDestroy() override;

    afx_msg void OnSelectParts();
    afx_msg void OnRemove();
    afx_msg void OnSetClearance();
    afx_msg void OnPartsChanged(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnClose();
    afx_msg void OnHelpButton();

    DECLARE_MESSAGE_MAP()

private:
    void ShowParts();
    std::vector<int> ChosenRows() const;
    bool ReadFields(std::wstring& problem);
    void Finish(bool apply);

    CCamTabCommand* m_pTab;
    camcore::SequenceStep m_step;
    CListCtrl m_parts;
    bool m_selecting;
};
