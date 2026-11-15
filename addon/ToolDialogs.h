// ToolDialogs.h : the dialog for editing one tool, with its speeds and feeds for
// each material, and the small dialogs it and the Machine dialog use.

#pragma once

#include <memory>

#include "resource.h"
#include "Materials.h"
#include "ToolLibrary.h"

class CFeedsPage;

// Asks for a name, such as a new machine's or material's.
class CNamePromptDlg : public CDialog
{
public:
    CNamePromptDlg(const std::wstring& title, const std::wstring& label, const std::wstring& name, CWnd* pParent);

    const std::wstring& GetName() const { return m_name; }

    enum { IDD = IDD_NAME_PROMPT };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;

private:
    std::wstring m_title;
    std::wstring m_label;
    std::wstring m_name;
};

// Chooses one of the materials.
class CChooseMaterialDlg : public CDialog
{
public:
    CChooseMaterialDlg(const camcore::MaterialList& materials, CWnd* pParent);

    const std::string& GetMaterialId() const { return m_id; }

    enum { IDD = IDD_CHOOSE_MATERIAL };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    afx_msg void OnHelpButton();

    DECLARE_MESSAGE_MAP()

private:
    camcore::MaterialList m_materials;
    std::string m_id;
    CComboBox m_list;
};

// A material's name and its cutting values for each tool material, for the feeds calculator.
class CMaterialDlg : public CDialog
{
public:
    // all is the whole list, holding this material, so its name can be checked against the others.
    CMaterialDlg(const camcore::Material& material, const camcore::MaterialList& all, CWnd* pParent);

    const camcore::Material& GetMaterial() const { return m_material; }

    enum { IDD = IDD_MATERIAL };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    afx_msg void OnHelpButton();
    afx_msg void OnSetChanged(UINT id);
    afx_msg void OnApplyTypical();

    DECLARE_MESSAGE_MAP()

private:
    void UpdateFields();

    camcore::Material m_material;
    camcore::MaterialList m_all;
    CComboBox m_typical;
};

// A tool's speeds and feeds for one material: a material, and the Feeds & Speeds tab's fields.
class CToolFeedsDlg : public CDialog
{
public:
    // usedMaterialIds are the materials the tool's other rows already have, so they aren't offered.
    // tool is the tool as typed in the Tool dialog, for the calculator; toolProblem says why it
    // can't be used, if it can't.
    CToolFeedsDlg(const camcore::ToolFeeds& row, const camcore::MaterialList& materials,
                  const std::vector<std::string>& usedMaterialIds, const camcore::Tool& tool,
                  const std::wstring& toolProblem, double maxSpindleRpm, CWnd* pParent);
    ~CToolFeedsDlg() override;

    const camcore::ToolFeeds& GetRow() const { return m_row; }

    enum { IDD = IDD_TOOL_FEEDS };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    afx_msg void OnHelpButton();

    DECLARE_MESSAGE_MAP()

private:
    camcore::ToolFeeds m_row;
    camcore::MaterialList m_materials;
    std::vector<std::string> m_offered;     // material IDs, in the combo's order
    std::vector<std::string> m_used;
    camcore::Tool m_tool;
    std::wstring m_toolProblem;
    double m_maxSpindleRpm;
    CComboBox m_material;
    std::unique_ptr<CFeedsPage> m_page;
};

// The Tool dialog's drawing of what each measurement means.
class CToolDrawingCtrl : public CStatic
{
public:
    // The tool as typed so far, drawn to its own proportions.
    void SetTool(const camcore::Tool& tool);

protected:
    void DrawItem(LPDRAWITEMSTRUCT pDraw) override;

private:
    camcore::Tool m_tool;
};

class CToolEditDlg : public CDialog
{
public:
    // otherTools is used to check that the tool number isn't already taken. maxSpindleRpm is
    // the machine's, for the feeds calculator.
    CToolEditDlg(const camcore::Tool& tool, const camcore::ToolLibrary& otherTools,
                 const camcore::MaterialList& materials, double maxSpindleRpm, CWnd* pParent);

    const camcore::Tool& GetTool() const { return m_tool; }

    enum { IDD = IDD_TOOL_EDIT };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    afx_msg void OnTypeChanged();
    afx_msg void OnUnitChanged();
    afx_msg void OnOwnShank();
    afx_msg void OnSizeChanged();
    afx_msg void OnHelpButton();
    afx_msg void OnAddFeeds();
    afx_msg void OnEditFeeds();
    afx_msg void OnRemoveFeeds();
    afx_msg void OnFeedsDoubleClick(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnFeedsSelectionChanged(NMHDR* pNotify, LRESULT* pResult);

    DECLARE_MESSAGE_MAP()

private:
    camcore::ToolType SelectedType() const;
    void UpdateFieldsForType();
    void SetNumber(int id, double value);
    bool ReadNumber(int id, const wchar_t* label, double& value, std::wstring& problems);
    bool ReadInteger(int id, const wchar_t* label, int& value, std::wstring& problems);
    void FillFeeds();

    // The tool as the fields have it; false, with the problems, if any field is wrong.
    bool ReadTool(camcore::Tool& tool, std::wstring& problems);
    int SelectedFeedsRow() const;
    std::vector<std::string> UsedMaterials(int except) const;

    camcore::Tool m_tool;
    camcore::ToolLibrary m_otherTools;
    camcore::MaterialList m_materials;
    double m_maxSpindleRpm;
    std::vector<camcore::ToolFeeds> m_feeds;    // as edited, saved into m_tool on OK
    CComboBox m_type;
    CComboBox m_unit;
    CComboBox m_toolMaterial;
    CListCtrl m_feedsList;
    CToolDrawingCtrl m_drawing;
};

// The material's name, or a note that it's no longer in the list.
std::wstring MaterialName(const camcore::MaterialList& materials, const std::string& id);
