// FeedsCalculatorDialog.h : the feeds and speeds calculator, which fills a Feeds & Speeds tab.

#pragma once

#include <string>
#include <vector>

#include "resource.h"
#include "FeedsCalculator.h"

// What the calculator works from: given by the operation dialog, or by a tool's
// Speeds and Feeds row.
struct FeedsCalcSource
{
    camcore::Tool tool;                 // in its own unit
    camcore::MaterialList materials;
    std::string materialId;             // the one to start with; may be empty
    bool materialFixed = false;         // the row's material, which can't be changed here
    double maxSpindleRpm = 0.0;         // 0 is no limit
};

class CFeedsCalcDlg : public CDialog
{
public:
    // start is the feeds being replaced; its transfer is kept.
    CFeedsCalcDlg(const FeedsCalcSource& source, const camcore::FeedSettings& start, CWnd* pParent);

    const camcore::FeedSettings& GetFeeds() const { return m_feeds; }

    enum { IDD = IDD_FEEDS_CALC };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    afx_msg void OnHelpButton();
    afx_msg void OnInputChanged();
    afx_msg void OnHScroll(UINT code, UINT position, CScrollBar* pScrollBar);
    afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT ctlColor);

    DECLARE_MESSAGE_MAP()

private:
    bool ReadBox(int id, double& value) const;
    void Recalculate();

    FeedsCalcSource m_source;
    camcore::Tool m_tool;               // in mm
    camcore::FeedSettings m_start;
    camcore::FeedSettings m_feeds;
    std::vector<std::string> m_materialIds;     // in the combo's order
    CComboBox m_material;
    CSliderCtrl m_aggression;
    bool m_ready = false;
    bool m_ok = false;
};
