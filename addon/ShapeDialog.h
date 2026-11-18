// ShapeDialog.h : the dialog that edits a shape typed in an operation: a
// rectangle, circle or polygon in the operation's origin. See core\BoundaryShape.h.

#pragma once

#include <functional>
#include <string>

#include "resource.h"
#include <vector>

#include "BoundaryShape.h"
#include "Origin.h"

class CShapeDlg : public CDialog
{
public:
    // preview, if given, is called with the shape each time the numbers make a
    // usable one, so the view can show it while the dialog is open.
    CShapeDlg(const camcore::TypedShape& shape, CWnd* pParent,
              std::function<void(const camcore::TypedShape&)> preview = nullptr);

    const camcore::TypedShape& GetShape() const { return m_shape; }

    enum { IDD = IDD_SHAPE };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    afx_msg void OnHelpButton();
    afx_msg void OnValueChanged(UINT id);
    afx_msg void OnSpin(UINT id, NMHDR* pNotify, LRESULT* pResult);

    DECLARE_MESSAGE_MAP()

private:
    // The shape from the boxes. False with a problem, in words for the user, if it isn't usable.
    bool ReadShape(camcore::TypedShape& shape, std::wstring& problem);
    void ShowNote(const std::wstring& problem);

    camcore::TypedShape m_shape;
    std::function<void(const camcore::TypedShape&)> m_preview;
    bool m_ready = false;       // the boxes are filled, so their changes are the user's
};

// Chooses Alibre sketches, from ListSketches, to add to an operation's features.
class CChooseSketchDlg : public CDialog
{
public:
    CChooseSketchDlg(const std::vector<camcore::PickedGeometry>& sketches, CWnd* pParent);

    const std::vector<camcore::PickedGeometry>& GetChosen() const { return m_chosen; }

    enum { IDD = IDD_CHOOSE_SKETCH };

protected:
    BOOL OnInitDialog() override;
    void OnOK() override;
    afx_msg void OnHelpButton();

    DECLARE_MESSAGE_MAP()

private:
    std::vector<camcore::PickedGeometry> m_sketches;
    std::vector<camcore::PickedGeometry> m_chosen;
};

// A shape's position and size for the Features tab's list.
std::wstring ShapePositionText(const camcore::TypedShape& shape);
std::wstring ShapeSizeText(const camcore::TypedShape& shape);
